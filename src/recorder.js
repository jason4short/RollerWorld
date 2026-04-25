// Data recorder for controller distillation.
//
// Captures (inputs → output) tuples at the inner-loop rate while a
// controller is running. The buffered data becomes the supervised
// training set for an imitation-learning NN.
//
// Two recording flavors share one buffer; each entry is tagged with `kind`
// so trainers can filter:
//
//   kind: 'ardubalance'      legacy whole-stack distillation
//     inputs: [pitch, pitch_rate, vel_cart, vel_cart_prev, vel_cart_target]
//     output: pwm
//
//   kind: 'cascade_mixer'    distill the Mixer's velocity → tilt step
//     vel_lpf, vel_target, pitch_target
//
//   kind: 'cascade_pitch'    distill Attitude's pitch arm
//     pitch, pitch_rate, pitch_target, force_fwd
//
// The pedagogy: random sampling covers the whole input envelope and the NN
// learns the whole function. Recorded sampling only covers the trajectories
// the bot actually visited — so the NN balances fine on the rehearsed
// flight but falls down on disturbances it never saw. That's the classic
// imitation-learning failure mode, made visible.

export class Recorder {
	constructor() {
		this.data          = [];
		this.recording     = false;
		this.vel_cart_prev = 0;   // tracks last sample's vel_cart so we can record it next tick
	}

	start()  { this.data.length = 0; this.recording = true; this.vel_cart_prev = 0; }
	stop()   { this.recording = false; }
	clear()  { this.data.length = 0; }
	size()   { return this.data.length; }
	count(kind) { return this.data.reduce((n, d) => n + (d.kind === kind ? 1 : 0), 0); }

	// ArduBalance whole-stack: sensor state → PWM.
	record({ pitch, pitch_rate, vel_cart, vel_cart_target, pwm }) {
		if (!this.recording) return;
		this.data.push({
			kind: 'ardubalance',
			inputs: [pitch, pitch_rate, vel_cart, this.vel_cart_prev, vel_cart_target],
			output: pwm,
		});
		this.vel_cart_prev = vel_cart;
	}

	// Cascade Mixer: velocity error → tilt command.
	recordCascadeMixer({ vel_lpf, vel_target, pitch_target }) {
		if (!this.recording) return;
		this.data.push({
			kind: 'cascade_mixer',
			vel_lpf, vel_target, pitch_target,
		});
	}

	// Cascade Attitude pitch arm: pitch state + target → force.
	recordCascadePitch({ pitch, pitch_rate, pitch_target, force_fwd }) {
		if (!this.recording) return;
		this.data.push({
			kind: 'cascade_pitch',
			pitch, pitch_rate, pitch_target, force_fwd,
		});
	}

	// Cascade Attitude yaw arm: heading_err + yaw_rate → torque.
	recordCascadeYaw({ heading_err, yaw_rate, torque_yaw }) {
		if (!this.recording) return;
		this.data.push({
			kind: 'cascade_yaw',
			heading_err, yaw_rate, torque_yaw,
		});
	}

	// Cascade Wheels: chassis force/torque + state → per-wheel PWM.
	recordCascadeWheels({ force_fwd, torque_yaw, vel_cart, yaw_rate, pwm_left, pwm_right }) {
		if (!this.recording) return;
		this.data.push({
			kind: 'cascade_wheels',
			force_fwd, torque_yaw, vel_cart, yaw_rate, pwm_left, pwm_right,
		});
	}

	// Bulk serialize — useful for offline training or inspection.
	toJSON() {
		return JSON.stringify({
			data: this.data,
		});
	}

	download() {
		const blob = new Blob([this.toJSON()], { type: 'application/json' });
		const url  = URL.createObjectURL(blob);
		const a    = document.createElement('a');
		a.href     = url;
		a.download = `roller-recording-${new Date().toISOString().slice(0, 19).replace(/[:T]/g, '-')}.json`;
		a.click();
		URL.revokeObjectURL(url);
	}
}
