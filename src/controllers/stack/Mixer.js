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
//
// Rule vs NN
// ----------
// The actual velocity-error → tilt math is a 2-input function, the smallest
// learnable layer in the stack. setMode('rule') uses `Kvel · err` clamped
// to ±tiltLimit; setMode('nn', mlp) routes the same two inputs through an
// MLP. LPF and slew always run as preprocessing — they're stateful utility,
// not the function we're learning.

const NN_INPUT_SCALES = [1 / 3, 1 / 3];   // vel_lpf, vel_target — both ±3 m/s
const NN_OUTPUT_SCALE = Math.PI / 6;       // ±1 → ±30° tilt

export class NavMixer {
	constructor() {
		this.vel_lpf      = 0;   // smoothed cart velocity (the feedback signal for Kvel)
		this.vel_target   = 0;   // post-slew velocity setpoint
		this.pitch_target = 0;   // last output (exposed for plotting / NN training)
		this.yaw_target   = 0;
		this.mode         = 'rule';
		this.mlp          = null;
	}

	setMode(mode, mlp = null) {
		this.mode = mode;
		this.mlp  = mlp;
	}

	static get NN_INPUT_SCALES() { return NN_INPUT_SCALES; }
	static get NN_OUTPUT_SCALE() { return NN_OUTPUT_SCALE; }

	// Stateless rule: the function the NN trainer queries as teacher and
	// the rule branch evaluates at runtime. Single source of truth.
	static computeTilt(vel_lpf, vel_target, gains) {
		const { Kvel, tiltLimit } = gains;
		let pitch = Kvel * (vel_target - vel_lpf);
		if (pitch >  tiltLimit) pitch =  tiltLimit;
		if (pitch < -tiltLimit) pitch = -tiltLimit;
		return pitch;
	}

	reset() {
		this.vel_lpf      = 0;
		this.vel_target   = 0;
		this.pitch_target = 0;
		this.yaw_target   = 0;
	}

	// navOut:  { vel_target_body, heading_target }
	// sensors: { vel_bot, heading, ... }
	// gains:   { Kvel, tiltLimit, a_max, vel_lpf_tc }
	update(navOut, sensors, gains, dt) {
		this._smoothVelocityFeedback(sensors, gains, dt);
		this._slewVelocitySetpoint(navOut.vel_target_body ?? 0, gains, dt);
		this._velocityErrorToTilt(gains);
		
		this.yaw_target      		= navOut.heading_target  ?? sensors.heading;
		this.heading_rate_ff 		= navOut.heading_rate_ff ?? 0;
		
		return {
			pitch_target:    		this.pitch_target,
			yaw_target:      		this.yaw_target,
			heading_rate_ff: 		this.heading_rate_ff,
		};
	}

	_smoothVelocityFeedback(sensors, gains, dt) {
		const tc    = gains.vel_lpf_tc ?? 0.1;
		const alpha = dt / (tc + dt);
		this.vel_lpf = (1 - alpha) * this.vel_lpf + alpha * sensors.vel_bot;
	}

	_slewVelocitySetpoint(vel_in, gains, dt) {
		const dv_max = (gains.a_max ?? 1.5) * dt;
		if      (vel_in > this.vel_target + dv_max) this.vel_target += dv_max;
		else if (vel_in < this.vel_target - dv_max) this.vel_target -= dv_max;
		else                                        this.vel_target  = vel_in;
	}

	_velocityErrorToTilt(gains) {
		this.pitch_target = (this.mode === 'nn' && this.mlp)
			? this._tiltFromNN(gains)
			: NavMixer.computeTilt(this.vel_lpf, this.vel_target, gains);
	}

	_tiltFromNN(gains) {
		const x = [
			this.vel_lpf    * NN_INPUT_SCALES[0],
			this.vel_target * NN_INPUT_SCALES[1],
		];
		const y = this.mlp.forward(x);
		let pitch = y[0] * NN_OUTPUT_SCALE;
		if (pitch >  gains.tiltLimit) pitch =  gains.tiltLimit;
		if (pitch < -gains.tiltLimit) pitch = -gains.tiltLimit;
		return pitch;
	}
}
