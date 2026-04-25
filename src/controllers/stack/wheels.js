// Wheels — chassis-level (force_fwd, torque_yaw) in, per-wheel PWM out.
// All motor math lives here and only here. If you're debugging "is the
// motor model right?" — this is the only file that matters.
//
// Two halves:
//
//   1. Differential mix.  Convert chassis commands to per-wheel forces:
//
//          F_left  = (force_fwd − torque_yaw / wheelbase) / 2
//          F_right = (force_fwd + torque_yaw / wheelbase) / 2
//
//      A pure forward command sends equal force to both wheels.
//      A pure yaw torque sends opposite forces.
//
//   2. Per-wheel force tracking.  For each wheel, drive the motor so the
//      realized force matches F_target.
//
//      Feed-forward from the inverse motor model:
//
//          F = Km · duty − Kv · v_wheel        (motor.js)
//          ⇒ duty_ff = (F_target + Kv · v_wheel) / Km
//          ⇒ pwm_ff  = duty_ff · PWM_max
//
//      Then close a small loop on force error to absorb deadband, friction,
//      and any other modeling gap:
//
//          F_realized = motor.forceFromPWM(pwm, v_wheel)
//          F_err      = F_target − F_realized
//          pwm       = pwm_ff + force_P · F_err + force_I · ∫F_err
//
//      Per-wheel speed is derived from body sensors:
//
//          v_left  = vel_cart − yaw_rate · wheelbase / 2
//          v_right = vel_cart + yaw_rate · wheelbase / 2
//
//      Real hardware would have wheel encoders. The sim uses the body's
//      yaw-rate gyro and forward velocity, which is equivalent.
//
// The plant's API is still chassis-level (force_fwd, torque_yaw), so we
// also report the realized chassis quantities computed from the per-wheel
// PWMs we actually applied. That keeps the simulator honest: the plant
// sees what the motors actually do, not what we wish they'd do.

class WheelChannel {
	constructor() {
		this.force_I = 0;   // integrator on force error (PWM units accumulated)
		this.lastPWM = 0;
	}
	reset() { this.force_I = 0; this.lastPWM = 0; }
}

export class Wheels {
	constructor() {
		this.left  = new WheelChannel();
		this.right = new WheelChannel();
		this.lastForceFwdActual  = 0;
		this.lastTorqueYawActual = 0;
	}

	reset() { this.left.reset(); this.right.reset(); }

	// attOut:  { force_fwd, torque_yaw }
	// sensors: { vel_cart, yaw_rate }
	// gains:   { wheelbase, force_P, force_I, force_I_max, deadband_extra,
	//            PWM_max }    (Km, Kv, native deadband come from `motor`)
	// motor:   the Motor instance (so we share one source of truth for
	//          the motor model — Wheels and the plant agree on Km/Kv/deadband)
	update(attOut, sensors, gains, dt, motor) {
		const half_wb = gains.wheelbase / 2;

		const F_left  = (attOut.force_fwd - attOut.torque_yaw / gains.wheelbase) / 2;
		const F_right = (attOut.force_fwd + attOut.torque_yaw / gains.wheelbase) / 2;

		const v_left  = sensors.vel_cart - sensors.yaw_rate * half_wb;
		const v_right = sensors.vel_cart + sensors.yaw_rate * half_wb;

		const pwm_left  = this._wheelLoop(this.left,  F_left,  v_left,  gains, dt, motor);
		const pwm_right = this._wheelLoop(this.right, F_right, v_right, gains, dt, motor);

		// Realized chassis-level quantities — what the plant will actually
		// see. Computed at the per-wheel velocities and PWMs we applied.
		const fL = motor.forceFromPWM(pwm_left,  v_left);
		const fR = motor.forceFromPWM(pwm_right, v_right);
		const force_fwd_actual  = fL + fR;
		const torque_yaw_actual = (fR - fL) * half_wb;

		this.lastForceFwdActual  = force_fwd_actual;
		this.lastTorqueYawActual = torque_yaw_actual;

		return {
			pwm_left, pwm_right,
			F_left, F_right,
			force_fwd_actual, torque_yaw_actual,
		};
	}

	_wheelLoop(ch, F_target, v_wheel, gains, dt, motor) {
		const { force_P, force_I, force_I_max = 500,
		        deadband_extra = 0, PWM_max } = gains;

		// Feed-forward from inverse motor model. F = Km·duty − Kv·v_wheel
		// ⇒ duty = (F + Kv·v_wheel) / Km.
		const duty_ff = (F_target + motor.Kv * v_wheel) / motor.Km;
		const pwm_ff  = duty_ff * motor.PWM_max;

		// Force tracking: realized force at last tick's PWM tells us how
		// far off the FF was; integrate the error to absorb deadband etc.
		const F_realized = motor.forceFromPWM(ch.lastPWM, v_wheel);
		const F_err      = F_target - F_realized;
		ch.force_I += F_err * dt;
		if (ch.force_I >  force_I_max) ch.force_I =  force_I_max;
		if (ch.force_I < -force_I_max) ch.force_I = -force_I_max;

		let pwm = pwm_ff + force_P * F_err + force_I * ch.force_I;

		// Deadband-jump compensation: motor.forceFromPWM already models the
		// dead PWM region. If we want to actually move at small force
		// commands, hop over it. Apply AFTER the PI sum so zero command
		// stays zero PWM.
		const EPS = 1e-6;
		const native_db = motor.deadband + deadband_extra;
		if (pwm >  EPS) pwm += native_db;
		if (pwm < -EPS) pwm -= native_db;

		const PM = PWM_max ?? motor.PWM_max;
		if (pwm >  PM) pwm =  PM;
		if (pwm < -PM) pwm = -PM;

		ch.lastPWM = pwm;
		return pwm;
	}
}
