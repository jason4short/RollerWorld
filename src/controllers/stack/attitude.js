// Attitude — angle targets in, body-frame chassis commands out.
//
// Two arms, structurally similar (target angle → angle error → rate target →
// torque/force), but they output different physical things because of how
// the platform is built:
//
//   Pitch arm  → force_fwd   (Newtons)
//                Why force, not torque? On a balance bot, "pitch torque on
//                the body" is not something the wheels can produce
//                directly — there is no body-mounted pitch reaction wheel.
//                Pitch is controlled by accelerating the wheels, which
//                creates a longitudinal force on the chassis. So the pitch
//                arm honestly outputs the force the chassis needs.
//
//   Yaw arm    → torque_yaw  (N·m)
//                Yaw really is a torque: a positive yaw torque comes from
//                the wheels pushing in opposite directions, and that's
//                handled at the Wheels layer's differential mix.
//
// Pitch is PD with a slow auto-trim of the IMU zero (pitch_I); yaw is the
// classic heading→rate→torque cascade.
//
// Why D on raw gyro instead of d(error)/dt? When the target angle steps
// (pilot snaps the stick, mixer recomputes), d(error)/dt has a derivative
// kick that slams the actuator. d(measurement)/dt is the same signal in
// steady state with no kick on setpoint changes. Free win.

export class Attitude {
	constructor() {
		this.balance_offset = 0;   // learned IMU zero-offset (rad)
		this.lastForceFwd   = 0;   // diagnostics
		this.lastTorqueYaw  = 0;
		this.lastYawRateRef = 0;
	}

	reset() {
		this.balance_offset = 0;
		this.lastForceFwd   = 0;
		this.lastTorqueYaw  = 0;
		this.lastYawRateRef = 0;
	}

	// mixerOut: { pitch_target, yaw_target }
	// sensors:  { pitch, pitch_rate, heading, yaw_rate }
	// gains:    pitch arm:  { pitch_P, pitch_D, pitch_I, force_max }
	//           yaw arm:    { heading_P, yaw_rate_max, yaw_rate_P, torque_max }
	update(mixerOut, sensors, gains, dt) {
		const force_fwd  = this._pitchArm(mixerOut.pitch_target, sensors, gains, dt);
		const torque_yaw = this._yawArm  (mixerOut.yaw_target,   sensors, gains);
		this.lastForceFwd  = force_fwd;
		this.lastTorqueYaw = torque_yaw;
		return { force_fwd, torque_yaw };
	}

	// Pitch: PD on (measured_pitch + balance_offset − target), with the
	// balance_offset slowly absorbing IMU bias when the bot is quiet.
	_pitchArm(pitch_target, sensors, gains, dt) {
		const { pitch_P, pitch_D, pitch_I, force_max } = gains;

		const pitch_meas = sensors.pitch + this.balance_offset;
		const pitch_err  = pitch_meas - pitch_target;

		// Auto-trim only when quiet — near upright AND not commanded —
		// otherwise the integrator absorbs real lean as bias and gradually
		// pushes the bot over. Leak slowly so a stale offset can't survive.
		const quiet = Math.abs(pitch_target) < 0.01
		           && Math.abs(pitch_err)    < 0.04;   // ~2.3°
		if (quiet) this.balance_offset += pitch_I * pitch_err * dt;
		this.balance_offset *= (1 - dt / 60);   // 60-s leak time constant

		// PD: P on err, D on raw gyro (avoids derivative kick on target steps).
		let force = pitch_P * pitch_err + pitch_D * sensors.pitch_rate;
		if (force >  force_max) force =  force_max;
		if (force < -force_max) force = -force_max;
		return force;
	}

	// Yaw: heading_err → desired yaw rate (P, clamped) → torque (P on rate
	// error, clamped). Standard angle-then-rate cascade.
	_yawArm(yaw_target, sensors, gains) {
		const { heading_P, yaw_rate_max, yaw_rate_P, torque_max } = gains;

		let heading_err = yaw_target - sensors.heading;
		while (heading_err >  Math.PI) heading_err -= 2 * Math.PI;
		while (heading_err < -Math.PI) heading_err += 2 * Math.PI;

		let yaw_rate_ref = heading_P * heading_err;
		if (yaw_rate_ref >  yaw_rate_max) yaw_rate_ref =  yaw_rate_max;
		if (yaw_rate_ref < -yaw_rate_max) yaw_rate_ref = -yaw_rate_max;
		this.lastYawRateRef = yaw_rate_ref;

		let torque = yaw_rate_P * (yaw_rate_ref - sensors.yaw_rate);
		if (torque >  torque_max) torque =  torque_max;
		if (torque < -torque_max) torque = -torque_max;
		return torque;
	}
}
