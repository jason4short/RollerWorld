// Training — distillation UI. Seven async methods, each trains a small
// MLP (or one RNN) to imitate a hand-tuned controller, then installs
// the trained model into the running cascade so the visitor can watch
// a learned approximation drive the bot side-by-side with the rule.
//
// One method per layer because each layer's I/O is different — the
// mixer takes (vel, vel_target) → tilt; the pitch attitude takes
// (pitch, pitch_rate, pitch_target) → force; etc. Same trainer class
// underneath, different shape per call.
//
//   trainNN              — distill ArduBalance whole-stack: 5→1
//   trainMixerNN         — cascade Mixer:        2→1   (vel/target → tilt)
//   trainNavNN           — cascade Nav:          4→2   (dx/dz/heading/vel → vel/heading)
//   trainAttitudePitchNN — cascade Attitude:     3→1   (pitch/rate/target → force)
//   trainAttitudeYawNN   — cascade Attitude:     2→1   (heading_err/yaw_rate → torque)
//   trainWheelsNN        — cascade Wheels:       4→2   (force/torque/vel/yaw → PWM L/R)
//   trainAttitudePitchRNN — same as pitch but RNN; learned hidden state instead of integrator
//
// All seven follow the same shape:
//   1. Read hyperparameters from the shared training UI (#nnHidden, #nnEpochs, etc.)
//   2. Check 'recorded' mode has enough samples; refuse otherwise.
//   3. Build a fresh model, run NNTrainer.train{Layer}() with progress
//      callback that updates the per-layer stats element + loss plot.
//   4. Stash the trained model on Training (so a select dropdown can
//      flip the layer to NN later) and call the layer's setMode('nn', model).
//
// Training has no scheduling or sim awareness — it's strictly UI work
// that mutates Rollerbot's installed sub-models and reads Sim's motor for
// the actuator-aware trainers.

import { MLP } from './nn/mlp.js';
import { RNN } from './nn/rnn.js';
import { NNTrainer } from './nn/trainer.js';

export class Training {
	constructor({ ui, recorder, robot, sim, drawLossPlot }) {
		this.ui            = ui;
		this.recorder      = recorder;
		this.robot         = robot;
		this.sim           = sim;
		this.drawLossPlot  = drawLossPlot;
		this.nnTrainer     = new NNTrainer();

		// Slots for trained models — the per-layer mode dropdowns on App
		// read these to decide whether 'nn' / 'rnn' is selectable yet.
		this.attitudePitchMlp = null;
		this.attitudeYawMlp   = null;
		this.mixerMlp         = null;
		this.navMlp           = null;
		this.wheelsMlp        = null;
		this.attitudePitchRnn = null;
	}

	// Distill the legacy ArduBalance controller. 5 inputs (pitch, pitch_rate,
	// vel, vel_target, plus one reserve) → 1 output (PWM). Installs the
	// trained MLP onto the legacy NN controller — the visitor switches
	// ctrlType to 'nn' to drive with the learned policy.
	async trainNN() {
		const mode    = document.getElementById('nnMode').value;
		const hidden  = +document.getElementById('nnHidden').value;
		const epochs  = +document.getElementById('nnEpochs').value;
		const samples = +document.getElementById('nnSamples').value;
		const lr      = +document.getElementById('nnLR').value;
		const nnStats = document.getElementById('nnStats');

		const arduCount = this.recorder.count('ardubalance');
		if (mode === 'recorded' && arduCount < 50) {
			this.ui.log(this.sim.simElapsedTime, `need more recorded ArduBalance samples (have ${arduCount}, want ≥50)`);
			return;
		}

		const mlp      = new MLP(5, hidden, 1);
		const gains    = this.ui.readArduGains();
		const navGains = this.ui.readNavGains();
		const dtInner  = 1 / Math.max(1, this.ui.readRates().innerHz);
		const srcDesc  = mode === 'random'
			? `${samples} random samples/epoch`
			: `${this.recorder.data.length} recorded samples`;
		nnStats.textContent = `training: 0/${epochs}, ${srcDesc}, ${mlp.paramCount()} params`;
		this.ui.log(this.sim.simElapsedTime, `training NN (${mode}, ${hidden} hidden, ${mlp.paramCount()} params)`);

		const lossHistory = [];
		const t0 = performance.now();
		await this.nnTrainer.train({
			mlp, mode, data: this.recorder.data, gains, navGains,
			motor: this.sim.motor, dt: dtInner,
			epochs, samplesPerEpoch: samples, lr, momentum: 0.9,
			onProgress: (e, loss) => {
				lossHistory.push(loss);
				nnStats.textContent = `epoch ${e + 1}/${epochs}	loss=${loss.toExponential(3)}`;
				this.drawLossPlot(lossHistory);
			},
		});
		const dt = (performance.now() - t0) / 1000;

		this.robot.controllers.nn.mlp = mlp;
		const finalLoss = lossHistory.at(-1);
		nnStats.textContent = `done: loss=${finalLoss.toExponential(3)}	(${dt.toFixed(1)}s)`;
		this.ui.log(this.sim.simElapsedTime, `NN trained: final loss=${finalLoss.toExponential(3)} in ${dt.toFixed(1)}s`);
	}

	// Train a small MLP to imitate the rule-based mixer (vel → tilt). Two
	// inputs, one output — converges quickly and shows the cleanest
	// example of distillation in the project.
	async trainMixerNN() {
		const hidden  = +document.getElementById('nnHidden').value;
		const epochs  = Math.max(50, +document.getElementById('nnEpochs').value);
		const samples = +document.getElementById('nnSamples').value;
		const lr      = +document.getElementById('nnLR').value;
		const stats   = document.getElementById('mixerNNStats');

		const mode = document.getElementById('nnMode').value;
		const recordedCount = this.recorder.count('cascade_mixer');
		if (mode === 'recorded' && recordedCount < 50) {
			this.ui.log(this.sim.simElapsedTime, `need more recorded cascade_mixer samples (have ${recordedCount}, want ≥50)`);
			return;
		}

		const mlp        = new MLP(2, hidden, 1);
		const mixerGains = this.ui.readCascadeGains().mixer;
		const srcDesc    = mode === 'random' ? `${samples} random/epoch` : `${recordedCount} recorded`;
		stats.textContent = `training (${mode}): 0/${epochs}, ${srcDesc}, ${mlp.paramCount()} params`;
		this.ui.log(this.sim.simElapsedTime, `training mixer NN (${mode}, ${hidden} hidden, ${mlp.paramCount()} params)`);

		const lossHistory = [];
		const t0 = performance.now();
		await this.nnTrainer.trainMixer({
			mlp, mixerGains, mode, data: this.recorder.data,
			epochs, samplesPerEpoch: samples, lr, momentum: 0.9,
			onProgress: (e, loss) => {
				lossHistory.push(loss);
				stats.textContent = `epoch ${e + 1}/${epochs}	loss=${loss.toExponential(3)}`;
				this.drawLossPlot(lossHistory);
			},
		});
		const dt = (performance.now() - t0) / 1000;

		this.mixerMlp = mlp;
		this.robot.stack.mixer.setMode('nn', mlp);
		document.getElementById('mixer_mode').value = 'nn';
		const finalLoss = lossHistory.at(-1);
		stats.textContent = `done: loss=${finalLoss.toExponential(3)}	(${dt.toFixed(1)}s) — using NN`;
		this.ui.log(this.sim.simElapsedTime, `mixer NN trained: final loss=${finalLoss.toExponential(3)} in ${dt.toFixed(1)}s, switched to NN`);
	}

	async trainNavNN() {
		const hidden  = +document.getElementById('nnHidden').value;
		const epochs  = Math.max(50, +document.getElementById('nnEpochs').value);
		const samples = +document.getElementById('nnSamples').value;
		const lr      = +document.getElementById('nnLR').value;
		const stats   = document.getElementById('navNNStats');

		const mode = document.getElementById('nnMode').value;
		const recordedCount = this.recorder.count('cascade_nav');
		if (mode === 'recorded' && recordedCount < 50) {
			this.ui.log(this.sim.simElapsedTime, `need more recorded cascade_nav samples (have ${recordedCount}, want ≥50)`);
			return;
		}

		const mlp      = new MLP(4, hidden, 2);
		const navGains = this.ui.readCascadeGains().nav;
		const srcDesc  = mode === 'random' ? `${samples} random/epoch` : `${recordedCount} recorded`;
		stats.textContent = `training (${mode}): 0/${epochs}, ${srcDesc}, ${mlp.paramCount()} params`;
		this.ui.log(this.sim.simElapsedTime, `training nav NN (${mode}, ${hidden} hidden, ${mlp.paramCount()} params)`);

		const lossHistory = [];
		const t0 = performance.now();
		await this.nnTrainer.trainNav({
			mlp, navGains, mode, data: this.recorder.data,
			epochs, samplesPerEpoch: samples, lr, momentum: 0.9,
			onProgress: (e, loss) => {
				lossHistory.push(loss);
				stats.textContent = `epoch ${e + 1}/${epochs}	loss=${loss.toExponential(3)}`;
				this.drawLossPlot(lossHistory);
			},
		});
		const dt = (performance.now() - t0) / 1000;

		this.navMlp = mlp;
		this.robot.stack.nav.setAutoMode('nn', mlp);
		document.getElementById('nav_mode_nn').value = 'nn';
		const finalLoss = lossHistory.at(-1);
		stats.textContent = `done: loss=${finalLoss.toExponential(3)}	(${dt.toFixed(1)}s) — using NN`;
		this.ui.log(this.sim.simElapsedTime, `nav NN trained: final loss=${finalLoss.toExponential(3)} in ${dt.toFixed(1)}s, switched to NN`);
	}

	async trainWheelsNN() {
		const hidden  = +document.getElementById('nnHidden').value;
		const epochs  = Math.max(50, +document.getElementById('nnEpochs').value);
		const samples = +document.getElementById('nnSamples').value;
		const lr      = +document.getElementById('nnLR').value;
		const stats   = document.getElementById('wheelsNNStats');

		const mode = document.getElementById('nnMode').value;
		const recordedCount = this.recorder.count('cascade_wheels');
		if (mode === 'recorded' && recordedCount < 50) {
			this.ui.log(this.sim.simElapsedTime, `need more recorded cascade_wheels samples (have ${recordedCount}, want ≥50)`);
			return;
		}

		const mlp        = new MLP(4, hidden, 2);
		const wheelGains = this.ui.readCascadeGains().wheels;
		const srcDesc    = mode === 'random' ? `${samples} random/epoch` : `${recordedCount} recorded`;
		stats.textContent = `training (${mode}): 0/${epochs}, ${srcDesc}, ${mlp.paramCount()} params`;
		this.ui.log(this.sim.simElapsedTime, `training wheels NN (${mode}, ${hidden} hidden, ${mlp.paramCount()} params)`);

		const lossHistory = [];
		const t0 = performance.now();
		await this.nnTrainer.trainWheels({
			mlp, wheelGains, motor: this.sim.motor,
			mode, data: this.recorder.data,
			epochs, samplesPerEpoch: samples, lr, momentum: 0.9,
			onProgress: (e, loss) => {
				lossHistory.push(loss);
				stats.textContent = `epoch ${e + 1}/${epochs}	loss=${loss.toExponential(3)}`;
				this.drawLossPlot(lossHistory);
			},
		});
		const dt = (performance.now() - t0) / 1000;

		this.wheelsMlp = mlp;
		this.robot.stack.wheels.setMode('nn', mlp);
		document.getElementById('wheels_mode').value = 'nn';
		const finalLoss = lossHistory.at(-1);
		stats.textContent = `done: loss=${finalLoss.toExponential(3)}	(${dt.toFixed(1)}s) — using NN (FF only)`;
		this.ui.log(this.sim.simElapsedTime, `wheels NN trained: final loss=${finalLoss.toExponential(3)} in ${dt.toFixed(1)}s, switched to NN`);
	}

	async trainAttitudeYawNN() {
		const hidden  = +document.getElementById('nnHidden').value;
		const epochs  = Math.max(50, +document.getElementById('nnEpochs').value);
		const samples = +document.getElementById('nnSamples').value;
		const lr      = +document.getElementById('nnLR').value;
		const stats   = document.getElementById('yawNNStats');

		const mode = document.getElementById('nnMode').value;
		const recordedCount = this.recorder.count('cascade_yaw');
		if (mode === 'recorded' && recordedCount < 50) {
			this.ui.log(this.sim.simElapsedTime, `need more recorded cascade_yaw samples (have ${recordedCount}, want ≥50)`);
			return;
		}

		const mlp      = new MLP(2, hidden, 1);
		const attGains = this.ui.readCascadeGains().attitude;
		const srcDesc  = mode === 'random' ? `${samples} random/epoch` : `${recordedCount} recorded`;
		stats.textContent = `training (${mode}): 0/${epochs}, ${srcDesc}, ${mlp.paramCount()} params`;
		this.ui.log(this.sim.simElapsedTime, `training yaw NN (${mode}, ${hidden} hidden, ${mlp.paramCount()} params)`);

		const lossHistory = [];
		const t0 = performance.now();
		await this.nnTrainer.trainAttitudeYaw({
			mlp, attGains, mode, data: this.recorder.data,
			epochs, samplesPerEpoch: samples, lr, momentum: 0.9,
			onProgress: (e, loss) => {
				lossHistory.push(loss);
				stats.textContent = `epoch ${e + 1}/${epochs}	loss=${loss.toExponential(3)}`;
				this.drawLossPlot(lossHistory);
			},
		});
		const dt = (performance.now() - t0) / 1000;

		this.attitudeYawMlp = mlp;
		this.robot.stack.attitude.setYawMode('nn', mlp);
		document.getElementById('yaw_mode').value = 'nn';
		const finalLoss = lossHistory.at(-1);
		stats.textContent = `done: loss=${finalLoss.toExponential(3)}	(${dt.toFixed(1)}s) — using NN`;
		this.ui.log(this.sim.simElapsedTime, `yaw NN trained: final loss=${finalLoss.toExponential(3)} in ${dt.toFixed(1)}s, switched to NN`);
	}

	// Train a small MLP to imitate the rule-based pitch of Attitude.
	// Reuses the NN-training UI's hidden-units / epochs / samples / LR
	// fields so a student can vary network capacity and watch loss vs
	// fidelity. On success, the trained MLP is auto-installed and the
	// pitch select flips to 'nn'.
	async trainAttitudePitchNN() {
		const hidden  = +document.getElementById('nnHidden').value;
		const epochs  = Math.max(50, +document.getElementById('nnEpochs').value);
		const samples = +document.getElementById('nnSamples').value;
		const lr      = +document.getElementById('nnLR').value;
		const stats   = document.getElementById('pitchNNStats');

		const mode = document.getElementById('nnMode').value;
		const recordedCount = this.recorder.count('cascade_pitch');
		if (mode === 'recorded' && recordedCount < 50) {
			this.ui.log(this.sim.simElapsedTime, `need more recorded cascade_pitch samples (have ${recordedCount}, want ≥50)`);
			return;
		}

		const mlp      = new MLP(3, hidden, 1);
		const attGains = this.ui.readCascadeGains().attitude;
		const srcDesc  = mode === 'random' ? `${samples} random/epoch` : `${recordedCount} recorded`;
		stats.textContent = `training (${mode}): 0/${epochs}, ${srcDesc}, ${mlp.paramCount()} params`;
		this.ui.log(this.sim.simElapsedTime, `training pitch NN (${mode}, ${hidden} hidden, ${mlp.paramCount()} params)`);

		const lossHistory = [];
		const t0 = performance.now();
		await this.nnTrainer.trainAttitudePitch({
			mlp, attGains, mode, data: this.recorder.data,
			epochs, samplesPerEpoch: samples, lr, momentum: 0.9,
			onProgress: (e, loss) => {
				lossHistory.push(loss);
				stats.textContent = `epoch ${e + 1}/${epochs}	loss=${loss.toExponential(3)}`;
				this.drawLossPlot(lossHistory);
			},
		});
		const dt = (performance.now() - t0) / 1000;

		this.attitudePitchMlp = mlp;
		this.robot.stack.attitude.setPitchMode('nn', mlp);
		document.getElementById('pitch_mode').value = 'nn';
		const finalLoss = lossHistory.at(-1);
		stats.textContent = `done: loss=${finalLoss.toExponential(3)}	(${dt.toFixed(1)}s) — using NN`;
		this.ui.log(this.sim.simElapsedTime, `pitch NN trained: final loss=${finalLoss.toExponential(3)} in ${dt.toFixed(1)}s, switched to NN`);
	}

	async trainAttitudePitchRNN() {
		const hidden  = +document.getElementById('rnnHidden').value || 12;
		const epochs  = +document.getElementById('rnnEpochs').value || 200;
		const seqLen  = +document.getElementById('rnnSeqLen').value || 8;
		const lr      = +document.getElementById('rnnLR').value     || 0.05;
		const stats   = document.getElementById('pitchRNNStats');

		// 3 inputs (pitch, pitch_rate, pitch_target) → 1 output (force).
		// Hidden state is what the rule controller's auto-trim integrator
		// is — but learned, not hand-coded. (For now the training data
		// has no temporal structure to learn; that's the next demo.)
		const rnn      = new RNN(3, hidden, 1);
		const attGains = this.ui.readCascadeGains().attitude;
		stats.textContent = `training RNN: 0/${epochs}, ${rnn.paramCount()} params, hidden=${hidden}, seq=${seqLen}`;
		this.ui.log(this.sim.simElapsedTime, `training pitch RNN (${hidden} hidden, seq=${seqLen}, ${rnn.paramCount()} params)`);

		const lossHistory = [];
		const t0 = performance.now();
		await this.nnTrainer.trainAttitudePitchRNN({
			rnn, attGains,
			epochs, episodes: 20, seqLen, lr, gradClip: 1.0,
			onProgress: (e, loss) => {
				lossHistory.push(loss);
				stats.textContent = `epoch ${e + 1}/${epochs}	loss=${loss.toExponential(3)}`;
				this.drawLossPlot(lossHistory);
			},
		});
		const dt = (performance.now() - t0) / 1000;

		this.attitudePitchRnn = rnn;
		this.robot.stack.attitude.setPitchMode('rnn', rnn);
		document.getElementById('pitch_mode').value = 'rnn';
		const finalLoss = lossHistory.at(-1);
		stats.textContent = `done: loss=${finalLoss.toExponential(3)}	(${dt.toFixed(1)}s) — using RNN`;
		this.ui.log(this.sim.simElapsedTime, `pitch RNN trained: final loss=${finalLoss.toExponential(3)} in ${dt.toFixed(1)}s, switched to RNN`);
	}
}
