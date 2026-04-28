import { PWMTable } from '../physics/pwm-table.js';

// Cascaded controller inspired by the ArduBalance firmware.
//
// Two loops at different rates:
//
//   OUTER (~100 Hz)   angle PD → commanded cart velocity (vel_command)
//                     plus a slow auto-trim on the IMU zero-offset
//   INNER (~400 Hz)   speed PID + feed-forward → PWM → force
//
// Output is per-wheel torque (force at the contact patch). The motor
// module owns PWM/deadband/back-EMF; this controller's "PWM" stays
// internal, an artifact of the firmware port. The chassis force_fwd it
// produces is differential-mixed with a yaw P-loop into per-wheel
// torque at the boundary.
//
// Uniform interface:
//   c.setAttitude(tilt, yawRate)            — direct: keyboard / raw mode
//   c.setCruise(speed, heading)             — firmware-style FBW + stabilize_yaw
//   c.slowLoop(sensors, gains, dt)
//   c.fastLoop(sensors, gains, dt, motor) → {torque_left, torque_right}
//
// Cruise mode is a faithful port of two firmware functions:
//   - ROLL_PITCH_FBW (ArduBalance.pde:1314): pilot speed adds into the
//     velocity reference; angle PD holds the bot vertical (target=0).
//   - get_stabilize_yaw (Attitude.pde:48): heading P-loop produces a
//     yaw-rate target, clamped, fed into the same yaw branch as the
//     direct yaw_rate path.

export class ArduBalanceController {
	constructor() {
		this.target_angle    = 0;
		this.yaw_rate_target = 0;
		this.pwmTable        = new PWMTable();   // FF curve (linear until calibrated)

		// Cruise mode — set by setCruise(speed, heading). When active,
		// cruise_speed adds into vel_command in the outer loop and
		// cruise_heading drives a P-loop for yaw_rate_target each tick.
		this.cruise_active   = false;
		this.cruise_speed    = 0;
		this.cruise_heading  = 0;

		this.reset();
	}

	reset() {
		this.balance_offset 	= 0;   // learned IMU zero-offset (rad)
		this.vel_command    	= 0;   // outer-loop output: commanded cart velocity (m/s)
		this.speed_I        	= 0;   // inner integrator (m·s because err·dt)
		this.last_vel_cart_meas = 0;   // for derivative-on-measurement in inner loop
		this.speed_d_lpf		= 0;   // low-passed derivative estimate
		this.lastPWM            = 0;
		this.lastForceFwd       = 0;
		this.lastTorqueYaw      = 0;
	}

	setAttitude(tilt, yawRate) {
		this.target_angle    = tilt    ?? 0;
		this.yaw_rate_target = yawRate ?? 0;
		this.cruise_active   = false;
	}

	// Firmware-style cruise: pilot commands speed (m/s) and absolute
	// heading (rad). Angle target stays vertical; the speed feeds the
	// outer loop directly, and a heading P-loop synthesizes yaw_rate.
	setCruise(speed, heading) {
		this.cruise_active  = true;
		this.cruise_speed   = speed   ?? 0;
		this.cruise_heading = heading ?? 0;
		this.target_angle   = 0;
	}

	// Called at outerHz by the bot — angle PD → vel_command, plus the
	// firmware-style cruise speed offset when cruise mode is active.
	slowLoop(sensors, gains, dt) {
		this._updateVelocity(sensors, gains, dt);
		if (this.cruise_active) this.vel_command += this.cruise_speed;
	}

	// Called at innerHz — speed PID + FF → chassis PWM → force_fwd, plus
	// a yaw P-loop, plus differential mix to per-wheel torque.
	fastLoop(sensors, gains, dt, motor) {
		const force_fwd = this._produceForce(sensors, gains, dt, motor);

		// In cruise mode, derive yaw_rate_target from the heading P-loop
		// (firmware get_stabilize_yaw). Otherwise use the rate set by
		// setAttitude directly.
		let yaw_rate_target = this.yaw_rate_target;
		if (this.cruise_active) {
			yaw_rate_target = this._stabilizeYaw(sensors, gains);
		}

		// Yaw P-loop on rate error. Torque is the differential force
		// times the wheelbase; the diff mix below converts to per-wheel.
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

	// Firmware get_stabilize_yaw: heading P → yaw_rate target, clamped.
	// Same shape as the original Attitude.pde:48 helper, just expressed
	// in rad/s instead of cm/s converted via wheel_ratio. Uses Kheading
	// + MaxYawRate from the controller's gain bag (merged in by
	// Rollerbot.currentGains; see UI.readNavGains).
	_stabilizeYaw(sensors, gains) {
		const { Kheading = 2, MaxYawRate = 1.5 } = gains;
		let heading_err = this.cruise_heading - sensors.heading;
		while (heading_err >  Math.PI) heading_err -= 2 * Math.PI;
		while (heading_err < -Math.PI) heading_err += 2 * Math.PI;
		let rate = Kheading * heading_err;
		if (rate >  MaxYawRate) rate =  MaxYawRate;
		if (rate < -MaxYawRate) rate = -MaxYawRate;
		return rate;
	}

	// ------------------------------------------------------------------------
	// Outer angle loop — commands a cart velocity to catch the lean.
	// ------------------------------------------------------------------------
	_updateVelocity(sensors, gains, dt) {
		const { bal_P, bal_D, bal_I, p_vel } = gains;

		// Apply learned sensor offset so the controller sees "true" tilt.
		const pitch     = sensors.pitch + this.balance_offset;
		const angle_err = pitch - this.target_angle;

		// Auto-trim the IMU zero: slowly integrate angle error WHEN quiet (near
		// upright and not being driven). The original firmware gated this on
		// `target_angle == 0` only — which meant long drives would leave the
		// trim frozen. Here we also check that the error itself is small, and
		// leak toward zero so it can't drift off during transients.
		const quiet = Math.abs(this.target_angle) < 0.01
			         && Math.abs(angle_err)       < 0.04;   // ~2.3°

		if (quiet) this.balance_offset += bal_I * angle_err * dt;
		this.balance_offset *= (1 - dt / 60);   // ~60 s leak time constant

		// D on raw gyro, not d(err)/dt — avoids a derivative kick when the
		// pilot snaps target_angle to a new value. (The original firmware did
		// the same thing, probably by accident: its PID helper consumed gyro
		// directly as the rate input.)
		//
		// Combined effect: vel_command = PD(angle) + p_vel · v_measured.
		this.vel_command = bal_P * angle_err + bal_D * sensors.pitch_rate + p_vel * sensors.vel_cart;
	}

	// ------------------------------------------------------------------------
	// Inner speed loop — tracks vel_command with PID + feed-forward.
	// ------------------------------------------------------------------------
	_produceForce(sensors, gains, dt, motor) {
		const { wheel_P, wheel_I, wheel_D, ff_per_mps, PWM_max, dead_zone } = gains;

		// Substituting vel_command's definition:
		//   speed_err = vel_command − v = bal_PD + (p_vel − 1) · v
		// So p_vel controls the inner loop's effective velocity coefficient:
		//   p_vel < 1  → net braking on wheel speed
		//   p_vel = 1  → neutral (pure angle tracking)
		//   p_vel > 1  → net boosting (amplifies outer-loop authority; must be
		//                matched by enough bal_P to stay stable)
		const speed_err = this.vel_command - sensors.vel_cart;

		// Integrator with anti-windup: stop accumulating while PWM is saturated.
		const saturated = Math.abs(this.lastPWM) >= PWM_max - 1;
		if (!saturated) this.speed_I += speed_err * dt;

		// Derivative on measurement (not error) — no kick when vel_command jumps.
		// Low-passed because the sensor samples slower than the inner loop.
		const raw_d = -(sensors.vel_cart - this.last_vel_cart_meas) / dt;
		this.last_vel_cart_meas = sensors.vel_cart;
		const time_constant = 0.02;
		const alpha = dt / (time_constant + dt);
		this.speed_d_lpf = (1 - alpha) * this.speed_d_lpf + alpha * raw_d;

		// Feed-forward from the PWM table or linear fallback.
		this.pwmTable.linearSlope = ff_per_mps;
		const ff = this.pwmTable.pwmFromSpeed(this.vel_command);
		let pwm = ff
				+ wheel_P * speed_err
				+ wheel_I * this.speed_I
				+ wheel_D * this.speed_d_lpf;

		// Motor driver deadband compensation: hops over the "stuck" PWM region
		// around zero. Applied AFTER the feedback path so zero command is still
		// zero PWM.
		if (pwm > 0) pwm += dead_zone;
		if (pwm < 0) pwm -= dead_zone;

		pwm = Math.max(-PWM_max, Math.min(PWM_max, pwm));
		this.lastPWM = pwm;
		return motor.forceFromPWM(pwm, sensors.vel_cart);
	}
}
