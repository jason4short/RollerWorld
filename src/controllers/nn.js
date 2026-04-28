// Neural-net controller. Replaces the full ArduBalance cascade AND nav's
// velocity-tracking step with a single MLP that maps
// (pitch, pitch_rate, vel_cart, vel_cart_prev, vel_cart_target) → PWM.
// The MLP is trained offline (see trainer.js) to mimic the combined
// nav-tilt + ArduBalance pipeline; at runtime we just evaluate it forward.
//
// Position is intentionally NOT an input. Position-relative-to-target is a
// nav-layer concern; the controller's job is to track a commanded body-frame
// velocity. Yaw is a separate per-controller branch (see innerUpdate).
//
// `vel_cart_prev` (last tick's cart velocity) is the damping channel — the
// NN can learn its own d/dt internally without us hand-rolling a noisy
// finite-difference. Without it, only angular damping (via pitch_rate) is
// available and the bot rocks back and forth on the wheel axis.
//
// Uniform interface:
//   c.setAttitude(tilt, yawRate)   — tilt unused (NN drives via velocity)
//   c.outerUpdate(sensors, gains, dt)            — no-op
//   c.innerUpdate(sensors, gains, dt, motor)
//                          → { torque_left, torque_right }
//
// vel_cart_target is set by Pilot/Rollerbot via a separate property
// (the NN's natural command channel is velocity, not attitude).

export class NNController {
	constructor() {
		this.mlp             = null;   // set by the trainer
		this.tilt_target     = 0;      // unused at inference time
		this.yaw_rate_target = 0;
		this.vel_cart_target = 0;      // commanded body-frame velocity (m/s)
		this.vel_cart_prev   = 0;      // last tick's cart velocity
		this.lastPWM         = 0;
		this.lastForceFwd    = 0;
		this.lastTorqueYaw   = 0;
		this.cruise_active   = false;
		this.cruise_heading  = 0;

		// Must match NNTrainer's normalization (and order).
		//          pitch,             pitch_rate,  vel_cart, vel_cart_prev, vel_cart_target
		this.inScale  = [1 / (Math.PI / 3), 1 / 10, 1 / 3,    1 / 3,         1 / 3];
		this.outScale = 2000;          // denormalize NN output → PWM
	}

	reset() {
		this.lastPWM       = 0;
		this.vel_cart_prev = 0;
		this.lastForceFwd  = 0;
		this.lastTorqueYaw = 0;
	}

	setAttitude(tilt, yawRate) {
		this.tilt_target     = tilt    ?? 0;
		this.yaw_rate_target = yawRate ?? 0;
		this.cruise_active   = false;
	}

	// Cruise: NN's natural command is body-frame velocity, so speed maps
	// directly to vel_cart_target. Heading drives a P-loop in
	// innerUpdate, same shape as ArduBalance's stabilize_yaw port.
	setCruise(speed, heading) {
		this.cruise_active   = true;
		this.vel_cart_target = speed   ?? 0;
		this.cruise_heading  = heading ?? 0;
	}

	outerUpdate() {}

	innerUpdate(sensors, gains, dt, motor) {
		const force_fwd = this._produceForce(sensors, motor);

		// Cruise mode synthesizes yaw_rate from heading P-loop; otherwise
		// use the rate set directly via setAttitude.
		let yaw_rate_target = this.yaw_rate_target;
		if (this.cruise_active) {
			const { Kheading = 2, MaxYawRate = 1.5 } = gains;
			let heading_err = this.cruise_heading - sensors.heading;
			while (heading_err >  Math.PI) heading_err -= 2 * Math.PI;
			while (heading_err < -Math.PI) heading_err += 2 * Math.PI;
			yaw_rate_target = Kheading * heading_err;
			if (yaw_rate_target >  MaxYawRate) yaw_rate_target =  MaxYawRate;
			if (yaw_rate_target < -MaxYawRate) yaw_rate_target = -MaxYawRate;
		}

		const { Kyaw = 0, MaxTauYaw = 5 } = gains;
		let torque_yaw = Kyaw * (yaw_rate_target - sensors.yaw_rate);
		if (torque_yaw >  MaxTauYaw) torque_yaw =  MaxTauYaw;
		if (torque_yaw < -MaxTauYaw) torque_yaw = -MaxTauYaw;

		this.lastForceFwd  = force_fwd;
		this.lastTorqueYaw = torque_yaw;

		const wb = gains.wheelbase ?? motor.wheelbase;
		return {
			torque_left:  (force_fwd - torque_yaw / wb) / 2,
			torque_right: (force_fwd + torque_yaw / wb) / 2,
		};
	}

	_produceForce(sensors, motor) {
		if (!this.mlp) return 0;

		const vel_cart = sensors.vel_cart;
		const x = [
			sensors.pitch        * this.inScale[0],
			sensors.pitch_rate   * this.inScale[1],
			vel_cart             * this.inScale[2],
			this.vel_cart_prev   * this.inScale[3],
			this.vel_cart_target * this.inScale[4],
		];
		const out = this.mlp.forward(x);
		let pwm = out[0] * this.outScale;

		// Deadband compensation — NN's smoothed output is often below the
		// motor deadband, producing zero thrust. Hop over it (same trick
		// ArduBalance's `dead_zone` gain does), but keep zero at zero.
		const EPS = 1e-6;
		if (pwm >  EPS) pwm += motor.deadband;
		if (pwm < -EPS) pwm -= motor.deadband;

		const PM = motor.PWM_max;
		if (pwm >  PM) pwm =  PM;
		if (pwm < -PM) pwm = -PM;
		this.lastPWM       = pwm;
		this.vel_cart_prev = vel_cart;
		return motor.forceFromPWM(pwm, vel_cart);
	}
}
