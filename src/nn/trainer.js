import { ArduBalanceController } from '../controllers/ardubalance.js';

// Supervised-learning trainer.
//
// Two modes:
//   random — generate uniformly random input states and query the teacher
//            (ArduBalance) for the "right" PWM. No recording needed. Covers
//            the whole input envelope evenly, so the NN is robust to
//            distribution shift when it takes control.
//   recorded — train on what the recorder captured (classic imitation
//              learning; biased toward the teacher's steady-state trajectory).
//
// The random mode is what Jason's car project uses in Car.train(), and it
// works better for the same reason: the NN that has to drive at inference
// time visits states the teacher wouldn't have, and it needs to know what
// to do there.

export class NNTrainer {
	constructor() {
		// Input normalization — divide by expected max, bringing values to ±1.
		//            th,             w,      x,      v,     target_angle,     dv
		this.inScale  = [1 / (Math.PI / 3), 1 / 10, 1 / 5, 1 / 3, 1 / (Math.PI / 6), 1 / 10];
		this.outScale = 1 / 2000;   // normalize PWM to ±1

		// Ranges over which we uniformly sample for random-mode training.
		// Wider than the teacher's steady-state envelope so the NN learns the
		// full response surface.
		this.ranges = {
			th:           [-Math.PI / 3, Math.PI / 3],   // ±60°
			w:            [-10, 10],                       // ±10 rad/s
			x:            [-5, 5],                          // ±5 m
			v:            [-3, 3],                          // ±3 m/s
			target_angle: [-Math.PI / 6, Math.PI / 6],   // ±30°
			dv:           [-10, 10],                       // cart accel, ±10 m/s²
		};
	}

	// Evaluate the teacher at a given state, setting its internal D-filter
	// so that ArduBalance sees the claimed cart acceleration `dv`. This
	// makes the teacher's PWM output actually depend on `dv`, so the NN
	// has a reason to learn to use that input.
	queryTeacher(state, gains, motor, dt) {
		const ab = new ArduBalanceController();
		ab.target_angle = state.target_angle;
		// raw_d = -(v - last_vmeas) / dt. Solve for last_vmeas such that
		// raw_d = -state.dv (so d(v)/dt = state.dv).
		ab.last_vmeas   = state.v - state.dv * dt;
		// Put the LPF at the instantaneous value so speed_d_lpf = raw_d = -dv.
		ab.speed_d_lpf  = -state.dv;
		const sensors = { th: state.th, w: state.w, x: state.x, v: state.v };
		ab.updateOuter(sensors, gains, dt);
		ab.produceForce(sensors, gains, dt, motor);
		return ab.lastPWM;
	}

	// Generate a batch of random (state → teacher PWM) samples.
	generateRandomBatch(n, gains, motor, dt) {
		const inputs  = new Array(n);
		const targets = new Array(n);
		const r = (lo, hi) => lo + Math.random() * (hi - lo);
		for (let i = 0; i < n; i++) {
			const state = {
				th:           r(...this.ranges.th),
				w:            r(...this.ranges.w),
				x:            r(...this.ranges.x),
				v:            r(...this.ranges.v),
				target_angle: r(...this.ranges.target_angle),
				dv:           r(...this.ranges.dv),
			};
			const pwm = this.queryTeacher(state, gains, motor, dt);
			const row = new Float64Array(6);
			row[0] = state.th           * this.inScale[0];
			row[1] = state.w            * this.inScale[1];
			row[2] = state.x            * this.inScale[2];
			row[3] = state.v            * this.inScale[3];
			row[4] = state.target_angle * this.inScale[4];
			row[5] = state.dv           * this.inScale[5];
			inputs[i]  = row;
			targets[i] = [pwm * this.outScale];
		}
		return { inputs, targets };
	}

	// Convert a recorded dataset to training-ready (normalized) arrays.
	// `dv` is derived by finite-differencing successive `v` samples at dt.
	prepareDataset(data, dt) {
		const inputs  = new Array(data.length);
		const targets = new Array(data.length);
		for (let n = 0; n < data.length; n++) {
			const src  = data[n].inputs;           // [th, w, x, v, target_angle]
			const prev = n > 0 ? data[n - 1].inputs : src;
			const dv   = (src[3] - prev[3]) / dt;
			const row  = new Float64Array(6);
			for (let i = 0; i < 5; i++) row[i] = src[i] * this.inScale[i];
			row[5] = dv * this.inScale[5];
			inputs[n]  = row;
			targets[n] = [data[n].output * this.outScale];
		}
		return { inputs, targets };
	}

	// Train asynchronously — yields to the event loop so the UI can update.
	//
	// Random mode regenerates a fresh batch each epoch (infinite data).
	// Recorded mode loops over the captured buffer.
	async train({ mlp, mode = 'random',
	              data = null, gains = null, motor = null, dt = 1 / 400,
	              epochs = 200, samplesPerEpoch = 1000,
	              lr = 0.02, momentum = 0.9, onProgress }) {
		const losses = [];
		let prepared = null;
		if (mode === 'recorded') prepared = this.prepareDataset(data, dt);

		for (let e = 0; e < epochs; e++) {
			const batch = mode === 'random'
				? this.generateRandomBatch(samplesPerEpoch, gains, motor, dt)
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
