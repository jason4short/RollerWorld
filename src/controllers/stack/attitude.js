// Attitude — angle targets in, body-frame chassis commands out.
//
// Two branches, structurally similar (target angle → angle error → rate target →
// torque/force), but they output different physical things because of how
// the platform is built:
//
//   Pitch  → force_fwd   (Newtons)
//                Why force, not torque? On a balance bot, "pitch torque on
//                the body" is not something the wheels can produce
//                directly — there is no body-mounted pitch reaction wheel.
//                Pitch is controlled by accelerating the wheels, which
//                creates a longitudinal force on the chassis. So the pitch
//                pitch branch honestly outputs the force the chassis needs.
//
//   Yaw    → torque_yaw  (N·m)
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
//
// Pitch — rule vs NN
// ----------------------
// The pitch is the most interesting layer to teach with a neural net:
// small input space (pitch, pitch_rate, pitch_target → force_fwd), high
// nonlinearity in tuning, immediately visible behavior. Attitude can
// swap its pitch between the hand-written PD and a trained MLP via
// `setPitchMode('rule')` or `setPitchMode('nn', mlp)`. Yaw is unchanged —
// it's a clean cascade and there's not much to learn there.

const PITCH_INPUT_SCALES  = [1 / (Math.PI / 3), 1 / 10, 1 / (Math.PI / 6)];
const PITCH_OUTPUT_SCALE  = 60;          // ±1 → ±60 N (matches typical force_max)
const YAW_INPUT_SCALES    = [1 / Math.PI, 1 / 10];   // heading_err (±π), yaw_rate
const YAW_OUTPUT_SCALE    = 2;             // ±1 → ±2 N·m (matches typical torque_max)

export class Attitude {
	constructor() {
		this.balance_offset = 0;   // learned IMU zero-offset (rad)
		this.lastForceFwd   = 0;   // diagnostics
		this.lastTorqueYaw  = 0;
		this.lastYawRateRef = 0;
		this.lastHeadingErr = 0;

		// Per-branch pluggability: 'rule' (hand-written) or 'nn' (MLP).
		this.pitchMode = 'rule';
		this.pitchMlp  = null;
		this.yawMode   = 'rule';
		this.yawMlp    = null;
	}

	setPitchMode(mode, mlp = null) {
		this.pitchMode = mode;
		this.pitchMlp  = mlp;
	}

	setYawMode(mode, mlp = null) {
		this.yawMode = mode;
		this.yawMlp  = mlp;
	}

	// Normalization shared with the trainer — the layer and the trainer
	// must agree on input/output scaling or the network is meaningless.
	static get PITCH_INPUT_SCALES()  { return PITCH_INPUT_SCALES; }
	static get PITCH_OUTPUT_SCALE()  { return PITCH_OUTPUT_SCALE; }
	static get YAW_INPUT_SCALES()    { return YAW_INPUT_SCALES; }
	static get YAW_OUTPUT_SCALE()    { return YAW_OUTPUT_SCALE; }

	reset() {
		this.balance_offset = 0;
		this.lastForceFwd   = 0;
		this.lastTorqueYaw  = 0;
		this.lastYawRateRef = 0;
	}

	// mixerOut: { pitch_target, yaw_target, heading_rate_ff }
	// sensors:  { pitch, pitch_rate, heading, yaw_rate }
	// gains:    pitch:  { pitch_P, pitch_D, pitch_I, force_max }
	//           yaw:    { heading_P, yaw_rate_max, yaw_rate_P, torque_max }
	update(mixerOut, sensors, gains, dt) {
		const force_fwd  = this._pitch(mixerOut.pitch_target, sensors, gains, dt);
		const torque_yaw = this._yaw  (
			mixerOut.yaw_target, mixerOut.heading_rate_ff ?? 0, sensors, gains,
		);
		this.lastForceFwd  = force_fwd;
		this.lastTorqueYaw = torque_yaw;
		return { force_fwd, torque_yaw };
	}

	_pitch(pitch_target, sensors, gains, dt) {
		if (this.pitchMode === 'nn' && this.pitchMlp) {
			return this._pitchNN(pitch_target, sensors, gains);
		}
		return this._pitchRule(pitch_target, sensors, gains, dt);
	}

	// Pitch (rule): PD on (measured_pitch + balance_offset − target), with
	// the balance_offset slowly absorbing IMU bias when the bot is quiet.
	_pitchRule(pitch_target, sensors, gains, dt) {
		const pitch_meas = sensors.pitch + this.balance_offset;
		const pitch_err  = pitch_meas - pitch_target;

		// Auto-trim only when quiet — near upright AND not commanded —
		// otherwise the integrator absorbs real lean as bias and gradually
		// pushes the bot over. Leak slowly so a stale offset can't survive.
		const quiet = Math.abs(pitch_target) < 0.01
		           && Math.abs(pitch_err)    < 0.04;   // ~2.3°
		if (quiet) this.balance_offset += gains.pitch_I * pitch_err * dt;
		this.balance_offset *= (1 - dt / 60);   // 60-s leak time constant

		// Delegate the PD math to a static helper so the NN trainer can
		// query the exact same function as a teacher.
		return Attitude.computePitchRule(
			{ pitch: pitch_meas, pitch_rate: sensors.pitch_rate, pitch_target },
			gains,
		);
	}

	// Stateless pitch math. The rule branch and the NN trainer both
	// call this so they agree on the function being approximated.
	// (pitch already includes balance_offset if you want it baked in.)
	static computePitchRule({ pitch, pitch_rate, pitch_target }, gains) {
		const { pitch_P, pitch_D, force_max } = gains;
		const pitch_err = pitch - pitch_target;
		let force = pitch_P * pitch_err + pitch_D * pitch_rate;
		if (force >  force_max) force =  force_max;
		if (force < -force_max) force = -force_max;
		return force;
	}

	// Pitch (NN): three inputs (pitch, pitch_rate, pitch_target) → one
	// output (force_fwd). The MLP is trained offline by NNTrainer to
	// imitate the rule-based pitch at random states. No auto-trim
	// here — it would require persistent state outside the network. If
	// IMU bias matters, train with `balance_offset` baked into the
	// teacher's pitch reading.
	_pitchNN(pitch_target, sensors, gains) {
		const inScale  = PITCH_INPUT_SCALES;
		const x = [
			sensors.pitch      * inScale[0],
			sensors.pitch_rate * inScale[1],
			pitch_target       * inScale[2],
		];
		const y = this.pitchMlp.forward(x);
		let force = y[0] * PITCH_OUTPUT_SCALE;
		const fmax = gains.force_max;
		if (force >  fmax) force =  fmax;
		if (force < -fmax) force = -fmax;
		return force;
	}

	// Yaw: heading_err → desired yaw rate (P, clamped) → torque (P on rate
	// error, clamped). Standard angle-then-rate cascade.
	//
	// heading_rate_ff is a feedforward term: the rate the upstream layer is
	// already commanding (e.g., FBW stick.yaw). Adding `yaw_rate_P · ff` to
	// the controller output is equivalent to including ff in yaw_rate_ref —
	// it keeps the bot rotating smoothly between Nav firings even though
	// Nav's heading_target is a 60 Hz staircase. Without FF, Attitude
	// catches each step in a few ms then idles, producing a stutter.
	_yaw(yaw_target, heading_rate_ff, sensors, gains) {
		let heading_err = yaw_target - sensors.heading;
		while (heading_err >  Math.PI) heading_err -= 2 * Math.PI;
		while (heading_err < -Math.PI) heading_err += 2 * Math.PI;
		this.lastHeadingErr = heading_err;

		// Track diagnostic for plotting (matches old field for compat).
		const { heading_P, yaw_rate_max, yaw_rate_P, torque_max } = gains;
		let ref = heading_P * heading_err;
		if (ref >  yaw_rate_max) ref =  yaw_rate_max;
		if (ref < -yaw_rate_max) ref = -yaw_rate_max;
		this.lastYawRateRef = ref;

		// Inner P-on-rate-error: rule or NN computes the torque from heading_err
		// + yaw_rate. Either way we add the FF term outside, so the rule/NN
		// stays a 2-input function the trainer can imitate.
		let torque;
		if (this.yawMode === 'nn' && this.yawMlp) {
			torque = this._yawNN(heading_err, sensors.yaw_rate, gains);
		} else {
			torque = Attitude.computeYawRule(
				{ heading_err, yaw_rate: sensors.yaw_rate }, gains,
			);
		}

		torque += yaw_rate_P * heading_rate_ff;
		if (torque >  torque_max) torque =  torque_max;
		if (torque < -torque_max) torque = -torque_max;
		return torque;
	}

	// Stateless yaw math. Same single-source-of-truth pattern as the
	// pitch — rule branch and trainer both call this.
	static computeYawRule({ heading_err, yaw_rate }, gains) {
		const { heading_P, yaw_rate_max, yaw_rate_P, torque_max } = gains;
		let yaw_rate_ref = heading_P * heading_err;
		if (yaw_rate_ref >  yaw_rate_max) yaw_rate_ref =  yaw_rate_max;
		if (yaw_rate_ref < -yaw_rate_max) yaw_rate_ref = -yaw_rate_max;
		let torque = yaw_rate_P * (yaw_rate_ref - yaw_rate);
		if (torque >  torque_max) torque =  torque_max;
		if (torque < -torque_max) torque = -torque_max;
		return torque;
	}

	_yawNN(heading_err, yaw_rate, gains) {
		const inScale = YAW_INPUT_SCALES;
		const x = [heading_err * inScale[0], yaw_rate * inScale[1]];
		const y = this.yawMlp.forward(x);
		let torque = y[0] * YAW_OUTPUT_SCALE;
		const tmax = gains.torque_max;
		if (torque >  tmax) torque =  tmax;
		if (torque < -tmax) torque = -tmax;
		return torque;
	}
}
