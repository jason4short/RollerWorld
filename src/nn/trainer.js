import { ArduBalanceController } from '../controllers/ardubalance.js';
import { Attitude } from '../controllers/stack/attitude.js';
import { NavMixer } from '../controllers/stack/mixer.js';

// Supervised-learning trainer.
//
// The NN takes (pitch, pitch_rate, vel_bot, vel_bot_prev, vel_bot_target)
// and outputs PWM. The teacher is the combined pipeline of nav's velocity-
// tracking step and the full ArduBalance cascade:
//
//     tilt = clamp(Kvel * (vel_bot_target - vel_bot), ±tiltLimit)
//     ab.target_angle = tilt
//     pwm = ab.updateVelocity + ab.produceForce
//
// Crucially, we initialize ArduBalance's inner-loop D-term state from the
// implied cart acceleration (vel_bot - vel_bot_prev)/dt so the NN sees
// meaningful wheel-D damping in its training targets. Without this the
// NN learns a controller with no linear damping and rocks on the wheel axis.
//
// Two modes:
//   random — generate uniformly random input states and query the teacher.
//            No recording needed. Covers the whole input envelope evenly,
//            so the NN is robust to distribution shift when it takes control.
//   recorded — train on what the recorder captured (classic imitation
//              learning; biased toward the teacher's steady-state trajectory).

export class NNTrainer {
	constructor() {
		// Input normalization — divide by expected max, bringing values to ±1.
		//          pitch,             pitch_rate,  vel_bot, vel_bot_prev, vel_bot_target
		this.inScale  = [1 / (Math.PI / 3), 1 / 10, 1 / 3,    1 / 3,         1 / 3];
		this.outScale = 1 / 2000;   // normalize PWM to ±1
	}

	// Build sample ranges from current nav gains. vel_bot_target is bounded
	// by nav.v_max so the NN trains over the full commandable envelope.
	_ranges(navGains) {
		const v_max = navGains?.v_max ?? 3;
		return {
			pitch:           [-Math.PI / 3, Math.PI / 3],   // ±60°
			pitch_rate:      [-10, 10],                     // ±10 rad/s
			vel_bot:        [-3, 3],                       // ±3 m/s (encoder envelope)
			vel_bot_prev:   [-3, 3],                       // sampled independently
			vel_bot_target: [-v_max, v_max],
		};
	}

	// Evaluate the teacher at a given state: nav's velocity-tracking step
	// produces a target_angle, which the ArduBalance cascade converts to PWM.
	queryTeacher(state, gains, navGains, motor, dt) {
		const Kvel      = navGains?.Kvel      ?? 0.4;
		const tiltLimit = navGains?.tiltLimit ?? Math.PI / 6;

		let tilt = Kvel * (state.vel_bot_target - state.vel_bot);
		if (tilt >  tiltLimit) tilt =  tiltLimit;
		if (tilt < -tiltLimit) tilt = -tiltLimit;

		const ab = new ArduBalanceController();
		ab.target_angle = tilt;
		// Init inner-loop D-state from the implied cart acceleration so the
		// teacher's wheel_D term contributes meaningfully. ArduBalance computes
		// raw_d = -(v - last_vmeas)/dt, and uses the LPF'd version. Settle the
		// LPF at raw_d so steady-state response is captured in one call.
		ab.last_vel_bot_meas = state.vel_bot_prev;
		const raw_d    = -(state.vel_bot - state.vel_bot_prev) / dt;
		ab.speed_d_lpf = raw_d;

		const sensors = {
			pitch:      state.pitch,
			pitch_rate: state.pitch_rate,
			vel_bot:   state.vel_bot,
		};
		ab.updateVelocity(sensors, gains, dt);
		ab.produceForce(sensors, gains, dt, motor);
		return ab.lastPWM;
	}

	// Generate a batch of random (state → teacher PWM) samples.
	generateRandomBatch(n, gains, navGains, motor, dt) {
		const ranges = this._ranges(navGains);
		const inputs  = new Array(n);
		const targets = new Array(n);
		const r = (lo, hi) => lo + Math.random() * (hi - lo);
		for (let i = 0; i < n; i++) {
			const state = {
				pitch:           r(...ranges.pitch),
				pitch_rate:      r(...ranges.pitch_rate),
				vel_bot:        r(...ranges.vel_bot),
				vel_bot_prev:   r(...ranges.vel_bot_prev),
				vel_bot_target: r(...ranges.vel_bot_target),
			};
			const pwm = this.queryTeacher(state, gains, navGains, motor, dt);
			const row = new Float64Array(5);
			row[0] = state.pitch           * this.inScale[0];
			row[1] = state.pitch_rate      * this.inScale[1];
			row[2] = state.vel_bot        * this.inScale[2];
			row[3] = state.vel_bot_prev   * this.inScale[3];
			row[4] = state.vel_bot_target * this.inScale[4];
			inputs[i]  = row;
			targets[i] = [pwm * this.outScale];
		}
		return { inputs, targets };
	}

	// Convert a recorded dataset to training-ready (normalized) arrays.
	// Buffer is shared across recording flavors; filter to ArduBalance.
	prepareDataset(data) {
		const rows = data.filter(d => d.kind === 'ardubalance');
		const inputs  = new Array(rows.length);
		const targets = new Array(rows.length);
		for (let n = 0; n < rows.length; n++) {
			const src = rows[n].inputs;
			const row = new Float64Array(5);
			for (let i = 0; i < 5; i++) row[i] = src[i] * this.inScale[i];
			inputs[n]  = row;
			targets[n] = [rows[n].output * this.outScale];
		}
		return { inputs, targets };
	}

	// ── Attitude pitch distillation ───────────────────────────────────
	// Tighter problem than whole-stack: 3 inputs, 1 output, no inner-loop
	// state to imitate. The teacher is `Attitude.computePitchRule`, the
	// stateless PD that the rule-based pitch delegates to. The MLP
	// learns to approximate that function over the input envelope.
	//
	// Why this is a better-shaped NN problem than the legacy whole-stack:
	//   - The function is genuinely a function (no hidden state).
	//   - The input envelope is small (pitch ±60°, pitch_rate ±10, target ±30°).
	//   - The output range is bounded (±force_max).
	// The same data → loss → tuning intuitions transfer directly to harder
	// learned-controller problems, but the failure modes are easy to see.

	generateAttitudePitchBatch(n, attGains) {
		const inScale  = Attitude.PITCH_INPUT_SCALES;
		const outScale = 1 / Attitude.PITCH_OUTPUT_SCALE;
		const r = (lo, hi) => lo + Math.random() * (hi - lo);

		const inputs  = new Array(n);
		const targets = new Array(n);
		for (let i = 0; i < n; i++) {
			const pitch        = r(-Math.PI / 3, Math.PI / 3);   // ±60°
			const pitch_rate   = r(-10, 10);                     // ±10 rad/s
			const pitch_target = r(-Math.PI / 6, Math.PI / 6);   // ±30°
			const force = Attitude.computePitchRule(
				{ pitch, pitch_rate, pitch_target }, attGains,
			);
			const row = new Float64Array(3);
			row[0] = pitch        * inScale[0];
			row[1] = pitch_rate   * inScale[1];
			row[2] = pitch_target * inScale[2];
			inputs[i]  = row;
			targets[i] = [force * outScale];
		}
		return { inputs, targets };
	}

	// ── Mixer distillation ────────────────────────────────────────────────
	// The smallest layer to learn: 2 inputs (vel_lpf, vel_target),
	// 1 output (pitch_target). The teacher is a saturating linear
	// function — `Kvel · err` clamped to ±tiltLimit. With even 4 hidden
	// units an MLP nails it in a few hundred epochs. Useful as the
	// "trivial" end of the pedagogical spectrum opposite the legacy
	// whole-stack distillation.

	generateMixerBatch(n, mixerGains) {
		const inScale  = NavMixer.NN_INPUT_SCALES;
		const outScale = 1 / NavMixer.NN_OUTPUT_SCALE;
		const r = (lo, hi) => lo + Math.random() * (hi - lo);

		const inputs  = new Array(n);
		const targets = new Array(n);
		for (let i = 0; i < n; i++) {
			const vel_lpf    = r(-3, 3);
			const vel_target = r(-3, 3);
			const tilt = NavMixer.computeTilt(vel_lpf, vel_target, mixerGains);
			const row = new Float64Array(2);
			row[0] = vel_lpf    * inScale[0];
			row[1] = vel_target * inScale[1];
			inputs[i]  = row;
			targets[i] = [tilt * outScale];
		}
		return { inputs, targets };
	}

	// Build a fixed batch from recorded cascade_mixer rows. Trainers
	// replay the same batch each epoch — no fresh sampling.
	prepareMixerRecorded(data) {
		const inScale  = NavMixer.NN_INPUT_SCALES;
		const outScale = 1 / NavMixer.NN_OUTPUT_SCALE;
		const rows = data.filter(d => d.kind === 'cascade_mixer');
		const inputs  = new Array(rows.length);
		const targets = new Array(rows.length);
		for (let i = 0; i < rows.length; i++) {
			const r = rows[i];
			const row = new Float64Array(2);
			row[0] = r.vel_lpf    * inScale[0];
			row[1] = r.vel_target * inScale[1];
			inputs[i]  = row;
			targets[i] = [r.pitch_target * outScale];
		}
		return { inputs, targets };
	}

	async trainMixer({ mlp, mixerGains, mode = 'random', data = null,
	                   epochs = 500, samplesPerEpoch = 500,
	                   lr = 0.02, momentum = 0.9, onProgress }) {
		const losses = [];
		const recorded = mode === 'recorded' ? this.prepareMixerRecorded(data) : null;

		for (let e = 0; e < epochs; e++) {
			const batch = recorded ?? this.generateMixerBatch(samplesPerEpoch, mixerGains);
			const loss  = mlp.trainEpoch(batch.inputs, batch.targets, lr, momentum);
			losses.push(loss);
			if (onProgress && (e % 5 === 0 || e === epochs - 1)) {
				onProgress(e, loss);
				await new Promise(r => setTimeout(r, 0));
			}
		}
		return losses;
	}

	prepareAttitudePitchRecorded(data) {
		const inScale  = Attitude.PITCH_INPUT_SCALES;
		const outScale = 1 / Attitude.PITCH_OUTPUT_SCALE;
		const rows = data.filter(d => d.kind === 'cascade_pitch');
		const inputs  = new Array(rows.length);
		const targets = new Array(rows.length);
		for (let i = 0; i < rows.length; i++) {
			const r = rows[i];
			const row = new Float64Array(3);
			row[0] = r.pitch        * inScale[0];
			row[1] = r.pitch_rate   * inScale[1];
			row[2] = r.pitch_target * inScale[2];
			inputs[i]  = row;
			targets[i] = [r.force_fwd * outScale];
		}
		return { inputs, targets };
	}

	// ── Attitude yaw distillation ─────────────────────────────────────
	// Inputs: heading_err (wrapped to ±π), yaw_rate. Output: torque_yaw.
	// Even simpler than the pitch — no auto-trim, just an angle→rate→
	// torque cascade with two clamps.

	generateAttitudeYawBatch(n, attGains) {
		const inScale  = Attitude.YAW_INPUT_SCALES;
		const outScale = 1 / Attitude.YAW_OUTPUT_SCALE;
		const r = (lo, hi) => lo + Math.random() * (hi - lo);

		const inputs  = new Array(n);
		const targets = new Array(n);
		for (let i = 0; i < n; i++) {
			const heading_err = r(-Math.PI, Math.PI);
			const yaw_rate    = r(-10, 10);
			const torque = Attitude.computeYawRule(
				{ heading_err, yaw_rate }, attGains,
			);
			const row = new Float64Array(2);
			row[0] = heading_err * inScale[0];
			row[1] = yaw_rate    * inScale[1];
			inputs[i]  = row;
			targets[i] = [torque * outScale];
		}
		return { inputs, targets };
	}

	prepareAttitudeYawRecorded(data) {
		const inScale  = Attitude.YAW_INPUT_SCALES;
		const outScale = 1 / Attitude.YAW_OUTPUT_SCALE;
		const rows = data.filter(d => d.kind === 'cascade_yaw');
		const inputs  = new Array(rows.length);
		const targets = new Array(rows.length);
		for (let i = 0; i < rows.length; i++) {
			const r = rows[i];
			const row = new Float64Array(2);
			row[0] = r.heading_err * inScale[0];
			row[1] = r.yaw_rate    * inScale[1];
			inputs[i]  = row;
			targets[i] = [r.torque_yaw * outScale];
		}
		return { inputs, targets };
	}

	async trainAttitudeYaw({ mlp, attGains, mode = 'random', data = null,
	                         epochs = 1000, samplesPerEpoch = 1000,
	                         lr = 0.02, momentum = 0.9, onProgress }) {
		const losses = [];
		const recorded = mode === 'recorded' ? this.prepareAttitudeYawRecorded(data) : null;

		for (let e = 0; e < epochs; e++) {
			const batch = recorded ?? this.generateAttitudeYawBatch(samplesPerEpoch, attGains);
			const loss  = mlp.trainEpoch(batch.inputs, batch.targets, lr, momentum);
			losses.push(loss);
			if (onProgress && (e % 5 === 0 || e === epochs - 1)) {
				onProgress(e, loss);
				await new Promise(r => setTimeout(r, 0));
			}
		}
		return losses;
	}

	async trainAttitudePitch({ mlp, attGains, mode = 'random', data = null,
	                           epochs = 1000, samplesPerEpoch = 1000,
	                           lr = 0.02, momentum = 0.9, onProgress }) {
		const losses = [];
		const recorded = mode === 'recorded' ? this.prepareAttitudePitchRecorded(data) : null;

		for (let e = 0; e < epochs; e++) {
			const batch = recorded ?? this.generateAttitudePitchBatch(samplesPerEpoch, attGains);
			const loss  = mlp.trainEpoch(batch.inputs, batch.targets, lr, momentum);
			losses.push(loss);
			if (onProgress && (e % 5 === 0 || e === epochs - 1)) {
				onProgress(e, loss);
				await new Promise(r => setTimeout(r, 0));
			}
		}
		return losses;
	}

	// Train the RNN pitch controller via BPTT. Each "epoch" is `episodes`
	// short sequences; the loss is summed across the sequence and
	// loss.backward() walks the whole unrolled graph, accumulating
	// gradients into the shared weights automatically.
	//
	// FIRST-CUT TRAINING DATA: each step samples a fresh random state
	// from the same envelope as the feedforward NN, with the rule's
	// force as the target. This doesn't really exercise the hidden
	// state — there's no temporal structure to learn. The point of
	// this first version is to confirm the BPTT wiring works and that
	// the trained RNN matches the rule controller. The follow-on demo
	// (bias-drift sequences with rule+auto-trim as teacher) is where
	// the hidden state earns its keep.
	async trainAttitudePitchRNN({ rnn, attGains,
	                              epochs = 200, episodes = 20, seqLen = 8,
	                              lr = 0.05, gradClip = 1.0, onProgress }) {
		const inScale  = Attitude.PITCH_INPUT_SCALES;
		const outScale = 1 / Attitude.PITCH_OUTPUT_SCALE;
		const r = (lo, hi) => lo + Math.random() * (hi - lo);

		const losses = [];
		for (let e = 0; e < epochs; e++) {
			let epochLoss = 0;
			for (let ep = 0; ep < episodes; ep++) {
				rnn.reset();
				let loss = null;
				for (let t = 0; t < seqLen; t++) {
					const pitch        = r(-Math.PI / 3, Math.PI / 3);
					const pitch_rate   = r(-10, 10);
					const pitch_target = r(-Math.PI / 6, Math.PI / 6);
					const x = [
						pitch        * inScale[0],
						pitch_rate   * inScale[1],
						pitch_target * inScale[2],
					];
					const target = Attitude.computePitchRule(
						{ pitch, pitch_rate, pitch_target }, attGains,
					) * outScale;

					const out = rnn.step(x);
					const err = out[0].sub(target);
					const sq  = err.mul(err);
					loss = loss === null ? sq : loss.add(sq);
				}
				// Mean over sequence — keeps gradient magnitudes
				// independent of seqLen so lr stays meaningful.
				loss = loss.mul(1 / seqLen);

				for (const p of rnn.parameters()) p.grad = 0;
				loss.backward();

				// Per-element gradient clipping. Shared weights accumulate
				// gradient over every step; one bad step will spike them.
				for (const p of rnn.parameters()) {
					if (p.grad >  gradClip) p.grad =  gradClip;
					if (p.grad < -gradClip) p.grad = -gradClip;
				}
				for (const p of rnn.parameters()) p.data -= lr * p.grad;
				epochLoss += loss.data;
			}
			epochLoss /= episodes;
			losses.push(epochLoss);
			if (onProgress && (e % 2 === 0 || e === epochs - 1)) {
				onProgress(e, epochLoss);
				await new Promise(r => setTimeout(r, 0));
			}
		}
		return losses;
	}

	// Train asynchronously — yields to the event loop so the UI can update.
	//
	// Random mode regenerates a fresh batch each epoch (infinite data).
	// Recorded mode loops over the captured buffer.
	async train({ mlp, mode = 'random',
	              data = null, gains = null, navGains = null, motor = null, dt = 1 / 400,
	              epochs = 3000, samplesPerEpoch = 1000,
	              lr = 0.02, momentum = 0.9, onProgress }) {
		const losses = [];
		let prepared = null;
		if (mode === 'recorded') prepared = this.prepareDataset(data);

		for (let e = 0; e < epochs; e++) {
			const batch = mode === 'random'
				? this.generateRandomBatch(samplesPerEpoch, gains, navGains, motor, dt)
				: prepared;
			const loss = mlp.trainEpoch(batch.inputs, batch.targets, lr, momentum);
			losses.push(loss);
			if (onProgress && (e % 5 === 0 || e === epochs - 1)) {
				onProgress(e, loss);
				await new Promise(r => setTimeout(r, 0));
			}
		}
		return losses;
	}
}
