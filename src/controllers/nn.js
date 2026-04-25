// Neural-net controller. Replaces the full ArduBalance cascade AND nav's
// velocity-tracking step with a single MLP that maps
// (pitch, pitch_rate, vel_cart, vel_cart_prev, vel_cart_target) → PWM.
// The MLP is trained offline (see trainer.js) to mimic the combined
// nav-tilt + ArduBalance pipeline; at runtime we just evaluate it forward.
//
// Position is intentionally NOT an input. Position-relative-to-target is a
// nav-layer concern; the controller's job is to track a commanded body-frame
// velocity. Yaw is handled separately by YawController.
//
// `vel_cart_prev` (last tick's cart velocity) is the damping channel — the
// NN can learn its own d/dt internally without us hand-rolling a noisy
// finite-difference. Without it, only angular damping (via pitch_rate) is
// available and the bot rocks back and forth on the wheel axis.

export class NNController {
	constructor() {
		this.mlp             = null;   // set by the trainer
		this.vel_cart_target = 0;      // commanded body-frame velocity (m/s), set by nav/FBW
		this.vel_cart_prev   = 0;      // last tick's cart velocity (damping channel)
		this.lastPWM         = 0;
		// Must match NNTrainer's normalization (and order).
		//          pitch,             pitch_rate,  vel_cart, vel_cart_prev, vel_cart_target
		this.inScale  = [1 / (Math.PI / 3), 1 / 10, 1 / 3,    1 / 3,         1 / 3];
		this.outScale = 2000;          // denormalize NN output → PWM
	}

	reset() {
		this.lastPWM       = 0;
		this.vel_cart_prev = 0;
	}

	// Single-rate: no outer loop, the MLP replaces the whole cascade.
	updateVelocity() {}

	produceForce(sensors, gains, dt, motor) {
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
		return motor.forceFromPWM(pwm, sensors.vel_cart);
	}
}
