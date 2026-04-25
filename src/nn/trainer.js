import { ArduBalanceController } from '../controllers/ardubalance.js';
import { Attitude } from '../controllers/stack/attitude.js';
import { NavMixer } from '../controllers/stack/mixer.js';

// Supervised-learning trainer.
//
// The NN takes (pitch, pitch_rate, vel_cart, vel_cart_prev, vel_cart_target)
// and outputs PWM. The teacher is the combined pipeline of nav's velocity-
// tracking step and the full ArduBalance cascade:
//
//     tilt = clamp(Kvel * (vel_cart_target - vel_cart), ±tiltLimit)
//     ab.target_angle = tilt
//     pwm = ab.updateVelocity + ab.produceForce
//
// Crucially, we initialize ArduBalance's inner-loop D-term state from the
// implied cart acceleration (vel_cart - vel_cart_prev)/dt so the NN sees
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
		//          pitch,             pitch_rate,  vel_cart, vel_cart_prev, vel_cart_target
		this.inScale  = [1 / (Math.PI / 3), 1 / 10, 1 / 3,    1 / 3,         1 / 3];
		this.outScale = 1 / 2000;   // normalize PWM to ±1
	}

	// Build sample ranges from current nav gains. vel_cart_target is bounded
	// by nav.v_max so the NN trains over the full commandable envelope.
	_ranges(navGains) {
		const v_max = navGains?.v_max ?? 3;
		return {
			pitch:           [-Math.PI / 3, Math.PI / 3],   // ±60°
			pitch_rate:      [-10, 10],                     // ±10 rad/s
			vel_cart:        [-3, 3],                       // ±3 m/s (encoder envelope)
			vel_cart_prev:   [-3, 3],                       // sampled independently
			vel_cart_target: [-v_max, v_max],
		};
	}

	// Evaluate the teacher at a given state: nav's velocity-tracking step
	// produces a target_angle, which the ArduBalance cascade converts to PWM.
	queryTeacher(state, gains, navGains, motor, dt) {
		const Kvel      = navGains?.Kvel      ?? 0.4;
		const tiltLimit = navGains?.tiltLimit ?? Math.PI / 6;

		let tilt = Kvel * (state.vel_cart_target - state.vel_cart);
		if (tilt >  tiltLimit) tilt =  tiltLimit;
		if (tilt < -tiltLimit) tilt = -tiltLimit;

		const ab = new ArduBalanceController();
		ab.target_angle = tilt;
		// Init inner-loop D-state from the implied cart acceleration so the
		// teacher's wheel_D term contributes meaningfully. ArduBalance computes
		// raw_d = -(v - last_vmeas)/dt, and uses the LPF'd version. Settle the
		// LPF at raw_d so steady-state response is captured in one call.
		ab.last_vel_cart_meas = state.vel_cart_prev;
		const raw_d    = -(state.vel_cart - state.vel_cart_prev) / dt;
		ab.speed_d_lpf = raw_d;

		const sensors = {
			pitch:      state.pitch,
			pitch_rate: state.pitch_rate,
			vel_cart:   state.vel_cart,
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
				vel_cart:        r(...ranges.vel_cart),
				vel_cart_prev:   r(...ranges.vel_cart_prev),
				vel_cart_target: r(...ranges.vel_cart_target),
			};
			const pwm = this.queryTeacher(state, gains, navGains, motor, dt);
			const row = new Float64Array(5);
			row[0] = state.pitch           * this.inScale[0];
			row[1] = state.pitch_rate      * this.inScale[1];
			row[2] = state.vel_cart        * this.inScale[2];
			row[3] = state.vel_cart_prev   * this.inScale[3];
			row[4] = state.vel_cart_target * this.inScale[4];
			inputs[i]  = row;
			targets[i] = [pwm * this.outScale];
		}
		return { inputs, targets };
	}

	// Convert a recorded dataset to training-ready (normalized) arrays.
	// Recorder format: inputs = [pitch, pitch_rate, vel_cart, vel_cart_prev, vel_cart_target].
	prepareDataset(data) {
		const inputs  = new Array(data.length);
		const targets = new Array(data.length);
		for (let n = 0; n < data.length; n++) {
			const src = data[n].inputs;
			const row = new Float64Array(5);
			for (let i = 0; i < 5; i++) row[i] = src[i] * this.inScale[i];
			inputs[n]  = row;
			targets[n] = [data[n].output * this.outScale];
		}
		return { inputs, targets };
	}

	// ── Attitude pitch-arm distillation ───────────────────────────────────
	// Tighter problem than whole-stack: 3 inputs, 1 output, no inner-loop
	// state to imitate. The teacher is `Attitude.computePitchArmRule`, the
	// stateless PD that the rule-based pitch arm delegates to. The MLP
	// learns to approximate that function over the input envelope.
	//
	// Why this is a better-shaped NN problem than the legacy whole-stack:
	//   - The function is genuinely a function (no hidden state).
	//   - The input envelope is small (pitch ±60°, pitch_rate ±10, target ±30°).
	//   - The output range is bounded (±force_max).
	// The same data → loss → tuning intuitions transfer directly to harder
	// learned-controller problems, but the failure modes are easy to see.

	generateAttitudePitchBatch(n, attGains) {
		const inScale  = Attitude.PITCH_ARM_INPUT_SCALES;
		const outScale = 1 / Attitude.PITCH_ARM_OUTPUT_SCALE;
		const r = (lo, hi) => lo + Math.random() * (hi - lo);

		const inputs  = new Array(n);
		const targets = new Array(n);
		for (let i = 0; i < n; i++) {
			const pitch        = r(-Math.PI / 3, Math.PI / 3);   // ±60°
			const pitch_rate   = r(-10, 10);                     // ±10 rad/s
			const pitch_target = r(-Math.PI / 6, Math.PI / 6);   // ±30°
			const force = Attitude.computePitchArmRule(
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

	async trainMixer({ mlp, mixerGains, epochs = 500, samplesPerEpoch = 500,
	                   lr = 0.02, momentum = 0.9, onProgress }) {
		const losses = [];
		for (let e = 0; e < epochs; e++) {
			const batch = this.generateMixerBatch(samplesPerEpoch, mixerGains);
			const loss  = mlp.trainEpoch(batch.inputs, batch.targets, lr, momentum);
			losses.push(loss);
			if (onProgress && (e % 5 === 0 || e === epochs - 1)) {
				onProgress(e, loss);
				await new Promise(r => setTimeout(r, 0));
			}
		}
		return losses;
	}

	async trainAttitudePitch({ mlp, attGains, epochs = 1000, samplesPerEpoch = 1000,
	                           lr = 0.02, momentum = 0.9, onProgress }) {
		const losses = [];
		for (let e = 0; e < epochs; e++) {
			const batch = this.generateAttitudePitchBatch(samplesPerEpoch, attGains);
			const loss  = mlp.trainEpoch(batch.inputs, batch.targets, lr, momentum);
			losses.push(loss);
			if (onProgress && (e % 5 === 0 || e === epochs - 1)) {
				onProgress(e, loss);
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
