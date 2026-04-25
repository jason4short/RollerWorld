// Data recorder for controller distillation.
//
// Captures (inputs, output) pairs at the inner-loop rate while the
// ArduBalance controller is running. The buffered data becomes the
// supervised training set for the NN replacement.
//
// Input vector (5): { pitch, pitch_rate, vel_cart, vel_cart_prev, vel_cart_target }
//   pitch, pitch_rate, vel_cart  — sensor state (what the bot observes)
//   vel_cart_prev                — last tick's cart velocity (damping channel)
//   vel_cart_target              — commanded body-frame velocity from nav/FBW
// Output (1): PWM
//
// Note: we deliberately exclude target_angle and vel_command (intermediate
// outputs of the cascade) because at NN inference time the NN replaces the
// whole pipeline and only sees vel_cart_target.

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

	// Call this from App.tick() at each inner-loop tick while a controller
	// is running. We record the inputs the NN will be given at inference
	// time, paired with the PWM that ArduBalance chose to output.
	record({ pitch, pitch_rate, vel_cart, vel_cart_target, pwm }) {
		if (!this.recording) return;
		this.data.push({
			inputs: [pitch, pitch_rate, vel_cart, this.vel_cart_prev, vel_cart_target],
			output: pwm,
		});
		this.vel_cart_prev = vel_cart;
	}

	// Bulk serialize — useful for offline training or inspection.
	toJSON() {
		return JSON.stringify({
			keys: ['pitch', 'pitch_rate', 'vel_cart', 'vel_cart_prev', 'vel_cart_target'],
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
