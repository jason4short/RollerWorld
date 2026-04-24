// Data recorder for controller distillation.
//
// Captures (inputs, output) pairs at the inner-loop rate while the
// ArduBalance controller is running. The buffered data becomes the
// supervised training set for the NN replacement.
//
// Input vector (5): { th, w, x, v, target_angle }
//   th, w, x, v   — sensor state (what the bot observes)
//   target_angle  — pilot/nav setpoint (the "goal" side of the mapping)
// Output (1): PWM
//
// Note: we deliberately exclude v_cmd (the outer loop's output) because
// at NN inference time there is no outer loop — the NN has to produce PWM
// from the same information the raw bot has.

export class Recorder {
	constructor() {
		this.data      = [];
		this.recording = false;
	}

	start()  { this.data.length = 0; this.recording = true; }
	stop()   { this.recording = false; }
	clear()  { this.data.length = 0; }
	size()   { return this.data.length; }

	// Call this from App.tick() at each inner-loop tick while a controller
	// is running. We record the inputs the NN will be given at inference
	// time, paired with the PWM that ArduBalance chose to output.
	record({ th, w, x, v, target_angle, pwm }) {
		if (!this.recording) return;
		this.data.push({
			inputs: [th, w, x, v, target_angle],
			output: pwm,
		});
	}

	// Bulk serialize — useful for offline training or inspection.
	toJSON() {
		return JSON.stringify({
			keys: ['th', 'w', 'x', 'v', 'target_angle'],
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
