// Drivetrain — DC motor pair, deadband, PWM saturation, force-tracking
// PI loop. Everything between "controller asks for torque on each wheel"
// and "what force the wheels actually deliver to the chassis" lives here.
//
// Per-motor model:
//
//   F = Km · duty − Kv · v_wheel        duty = PWM / PWM_max
//
// Km lumps V_bat, torque constant, wheel radius, motor count.
// Kv is back-EMF-induced drag at the wheel referred to wheel linear speed.
// Not modeled: winding inductance, current limit, gear backlash, cogging.
//
// Two callers today:
//   forceFromPWM(pwm, v)             — single-channel chassis-level shim
//                                      used by legacy (non-cascade) paths.
//   applyTorque({tL, tR}, …) → {…}   — per-wheel torque in, chassis force
//                                      and yaw torque out. New uniform
//                                      drivetrain API for refactored
//                                      controllers.
//
// applyTorque holds its own per-wheel integrator state (force_I) and
// last-PWM cache so the force-tracking PI absorbs deadband, friction,
// and any other modeling gap not covered by the FF inverse model. Same
// math as the cascade's old Wheels layer; lifted here so every
// controller can hand off torque commands without re-deriving PWM.

import { PWMTable } from './pwm-table.js';

class WheelChannel {
	constructor() {
		this.force_I = 0;   // PWM·s accumulated by the force-tracking PI
		this.lastPWM = 0;
	}
	reset() { this.force_I = 0; this.lastPWM = 0; }
}

export class Motor {
	constructor({ Km = 60, Kv = 10, PWM_max = 2000, deadband = 80, wheelbase = 0.3 } = {}) {
		this.Km        = Km;
		this.Kv        = Kv;
		this.PWM_max   = PWM_max;
		// deadband (in PWM counts): drive-train friction + driver stiction
		// means the cart doesn't move at all until |pwm| exceeds this.
		// Real ArduRoller-class hardware had deadbands around 5–10% of
		// PWM_max.
		this.deadband  = deadband;
		// Distance between wheel contact patches (m). Lives on the motor
		// because both the diff-mix (controllers → per-wheel torque) and
		// the inverse-mix in applyTorque (per-wheel realized force →
		// chassis yaw torque) need it.
		this.wheelbase = wheelbase;

		// Calibration table for single-channel feed-forward (used by
		// ArduBalance's chassis-level inner loop). Linear until calibrated.
		this.pwmTable = new PWMTable();

		// Per-wheel state for applyTorque's PI tracking.
		this.left  = new WheelChannel();
		this.right = new WheelChannel();
	}

	reset() {
		this.left.reset();
		this.right.reset();
	}

	// Single-channel: PWM in, chassis force out. Models deadband + back-EMF.
	// Used by callers that still output chassis-level PWM (ArduBalance,
	// NN). Stays here as a peer of applyTorque so all motor math is in
	// one place.
	forceFromPWM(pwm, v) {
		const abs = Math.abs(pwm);
		const eff = abs <= this.deadband ? 0 : Math.sign(pwm) * (abs - this.deadband);
		const duty = eff / this.PWM_max;
		return this.Km * duty - this.Kv * v;
	}

	// Per-wheel torque → chassis force + yaw torque.
	//
	//   torque_left, torque_right   — per-wheel actuator commands (N at
	//                                 the contact patch; treat as wheel
	//                                 torque divided by wheel radius)
	//   sensors  { vel_bot, yaw_rate }  — body frame
	//   dt                          — seconds since last applyTorque call
	//   gains    { wheelbase, force_P, force_I, force_I_max,
	//              deadband_extra, PWM_max }
	//
	// Returns:
	//   { force, yaw_torque,        — chassis-level realized output
	//     pwm_left, pwm_right,      — for telemetry / NN training
	//     F_left, F_right }         — commanded per-wheel forces
	applyTorque({ torque_left, torque_right }, sensors, dt, gains) {
		const wb      = gains.wheelbase ?? this.wheelbase;
		const half_wb = wb / 2;
		const v_left  = sensors.vel_bot - sensors.yaw_rate * half_wb;
		const v_right = sensors.vel_bot + sensors.yaw_rate * half_wb;

		const pwm_left  = this._wheelLoop(this.left,  torque_left,  v_left,  gains, dt);
		const pwm_right = this._wheelLoop(this.right, torque_right, v_right, gains, dt);

		// Realized chassis quantities — what the plant will actually see.
		const fL = this.forceFromPWM(pwm_left,  v_left);
		const fR = this.forceFromPWM(pwm_right, v_right);
		const force      = fL + fR;
		const yaw_torque = (fR - fL) * (wb / 2);

		return {
			force, yaw_torque,
			pwm_left, pwm_right,
			F_left: torque_left, F_right: torque_right,
		};
	}

	// Per-wheel inner loop: inverse-model FF + force-tracking PI +
	// deadband-jump + saturation. Matches the cascade's old Wheels layer
	// math, lifted here so controllers don't carry it.
	_wheelLoop(ch, F_target, v_wheel, gains, dt) {
		const { force_P, force_I, force_I_max = 500,
		        deadband_extra = 0, PWM_max } = gains;

		// FF from inverse motor model. F = Km·duty − Kv·v_wheel
		// ⇒ duty = (F + Kv·v_wheel) / Km.
		const duty_ff = (F_target + this.Kv * v_wheel) / this.Km;
		const pwm_ff  = duty_ff * this.PWM_max;

		// Force tracking — realized force at last tick's PWM tells us how
		// far off the FF was; integrate the error to absorb deadband etc.
		const F_realized = this.forceFromPWM(ch.lastPWM, v_wheel);
		const F_err      = F_target - F_realized;
		ch.force_I += F_err * dt;
		if (ch.force_I >  force_I_max) ch.force_I =  force_I_max;
		if (ch.force_I < -force_I_max) ch.force_I = -force_I_max;

		let pwm = pwm_ff + force_P * F_err + force_I * ch.force_I;

		// Deadband-jump compensation: forceFromPWM models the dead PWM
		// region. To actually move at small force commands, hop over it.
		// Apply AFTER the PI sum so zero command stays zero PWM.
		const EPS = 1e-6;
		const native_db = this.deadband + deadband_extra;
		if (pwm >  EPS) pwm += native_db;
		if (pwm < -EPS) pwm -= native_db;

		const PM = PWM_max ?? this.PWM_max;
		if (pwm >  PM) pwm =  PM;
		if (pwm < -PM) pwm = -PM;

		ch.lastPWM = pwm;
		return pwm;
	}
}
