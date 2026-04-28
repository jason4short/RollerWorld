// PitchHold — single-loop PD-on-pitch (with optional integrator and
// position terms). The simplest of the four controllers in the bot:
// pitch error in, force out, no outer velocity loop.
//
// "PitchHold" names what it does, not what it's built from — every
// controller in here uses PID-style math, so naming this one "PID"
// would be misleading.
//
// Uniform controller surface (every controller implements this):
//   c.setAttitude(tilt, yawRate)            — from Pilot (raw mode) or
//                                              from a higher-level loop
//   c.slowLoop(sensors, gains, dt)       — slow loop (no-op here)
//   c.fastLoop(sensors, gains, dt, motor)
//                          → { torque_left, torque_right }
//
// gains: pitch params + yaw params + (optional) wheelbase override.
//   Kp, Ki, Kd, Kx, Kv, Fmax    — pitch / position
//   Kyaw, MaxTauYaw             — yaw P-loop on yaw_rate
// wheelbase comes from `motor.wheelbase` unless gains.wheelbase is set.

export class PitchHoldController {
	constructor() {
		this.pitch_integral  = 0;
		this.tilt_target     = 0;
		this.yaw_rate_target = 0;
		this.lastForceFwd    = 0;   // diagnostics
		this.lastTorqueYaw   = 0;
	}

	reset() {
		this.pitch_integral  = 0;
		this.tilt_target     = 0;
		this.yaw_rate_target = 0;
		this.lastForceFwd    = 0;
		this.lastTorqueYaw   = 0;
	}

	setAttitude(tilt, yawRate) {
		this.tilt_target     = tilt     ?? 0;
		this.yaw_rate_target = yawRate  ?? 0;
	}

	// Single-rate controller — no slow loop.
	slowLoop() {}

	fastLoop(sensors, gains, dt, motor) {
		const { Kp, Ki, Kd, Kx, Kv, Fmax,
		        Kyaw = 0, MaxTauYaw = 5 } = gains;

		// Pitch error referenced to the target tilt. Old code did this by
		// pre-biasing measured.pitch in the bot; now lives where it
		// belongs.
		const pitch_err = sensors.pitch - this.tilt_target;
		this.pitch_integral += pitch_err * dt;

		// F = Kp·pitch_err + Kd·pitch_rate + Ki·∫pitch_err + Kx·x + Kv·ẋ
		// Catching a positive lean requires +F (push the cart toward the
		// lean). Position term keeps the bot from drifting; uses
		// body-frame x (encoder) so yaw doesn't strand a stale world-x
		// error.
		const x_pos = sensors.x_body ?? sensors.x;
		let force_fwd = (Kp * pitch_err + Kd * sensors.pitch_rate + Ki * this.pitch_integral)
		              + (Kx * x_pos     + Kv * sensors.vel_cart);
		if (force_fwd >  Fmax) force_fwd =  Fmax;
		if (force_fwd < -Fmax) force_fwd = -Fmax;

		// Yaw P-loop on rate error. Each controller owns its yaw branch
		// now — no shared YawController helper.
		let torque_yaw = Kyaw * (this.yaw_rate_target - sensors.yaw_rate);
		if (torque_yaw >  MaxTauYaw) torque_yaw =  MaxTauYaw;
		if (torque_yaw < -MaxTauYaw) torque_yaw = -MaxTauYaw;

		this.lastForceFwd  = force_fwd;
		this.lastTorqueYaw = torque_yaw;

		// Differential mix: pure forward → equal both wheels; pure yaw →
		// opposite. Motor.applyTorque does the inverse mix on the
		// realized side.
		const wb = gains.wheelbase ?? motor.wheelbase;
		return {
			torque_left:  (force_fwd - torque_yaw / wb) / 2,
			torque_right: (force_fwd + torque_yaw / wb) / 2,
		};
	}
}
