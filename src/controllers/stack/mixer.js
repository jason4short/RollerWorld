// Nav Mixer — turns Nav's "where I want to be going" into Attitude's
// "what angle I should be at."
//
// Nav says: "I want body-frame velocity 1.2 m/s and heading π/2."
// Attitude wants: "tilt me to +0.06 rad and aim heading at π/2."
//
// The mixer is the small loop that closes velocity error using lean. Yaw
// passes through unchanged — Attitude does the heading→rate cascade.
//
// Three things happen here, in order:
//
//   1. Smooth the velocity feedback.   The encoder is noisy; an LPF gives
//                                       Kvel something stable to react to.
//   2. Slew the velocity setpoint.     Nav can step (clicked target,
//                                       stick yank). The mixer's setpoint
//                                       follows at most a_max·dt per tick.
//   3. Velocity error → tilt command.  pitch_target = Kvel · (vel_target − vel_lpf),
//                                       clamped to ±tiltLimit.
//
// On a balance bot you can't just "command a velocity" — you can only lean,
// and physics turns lean into acceleration. So this layer is where the
// physical reality of the platform meets the abstract "go this fast" command.

export class NavMixer {
	constructor() {
		this.vel_lpf      = 0;   // smoothed cart velocity (the feedback signal for Kvel)
		this.vel_target   = 0;   // post-slew velocity setpoint
		this.pitch_target = 0;   // last output (exposed for plotting / NN training)
		this.yaw_target   = 0;
	}

	reset() {
		this.vel_lpf      = 0;
		this.vel_target   = 0;
		this.pitch_target = 0;
		this.yaw_target   = 0;
	}

	// navOut:  { vel_target_body, heading_target }
	// sensors: { vel_cart, heading, ... }
	// gains:   { Kvel, tiltLimit, a_max, vel_lpf_tc }
	update(navOut, sensors, gains, dt) {
		this._smoothVelocityFeedback(sensors, gains, dt);
		this._slewVelocitySetpoint(navOut.vel_target_body ?? 0, gains, dt);
		this._velocityErrorToTilt(gains);
		this.yaw_target = navOut.heading_target ?? sensors.heading;
		return { pitch_target: this.pitch_target, yaw_target: this.yaw_target };
	}

	_smoothVelocityFeedback(sensors, gains, dt) {
		const tc    = gains.vel_lpf_tc ?? 0.1;
		const alpha = dt / (tc + dt);
		this.vel_lpf = (1 - alpha) * this.vel_lpf + alpha * sensors.vel_cart;
	}

	_slewVelocitySetpoint(vel_in, gains, dt) {
		const dv_max = (gains.a_max ?? 1.5) * dt;
		if      (vel_in > this.vel_target + dv_max) this.vel_target += dv_max;
		else if (vel_in < this.vel_target - dv_max) this.vel_target -= dv_max;
		else                                        this.vel_target  = vel_in;
	}

	_velocityErrorToTilt(gains) {
		const { Kvel, tiltLimit } = gains;
		let pitch = Kvel * (this.vel_target - this.vel_lpf);
		if (pitch >  tiltLimit) pitch =  tiltLimit;
		if (pitch < -tiltLimit) pitch = -tiltLimit;
		this.pitch_target = pitch;
	}
}
