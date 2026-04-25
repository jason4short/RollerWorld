// Rolling time-series plot. Draws any subset of configurable series over
// the last windowSec seconds. Each series is {key, color, scale, label},
// where `scale` is the multiplier that maps the physical value to ±1 for
// plotting (so ±1 fills half the canvas height).

export class Plotter {
	constructor(canvas, windowSec = 10) {
		this.canvas = canvas;
		this.ctx    = canvas.getContext('2d');
		this.window = windowSec;
	}

	// series: [{key, color, scale, label}, ...]
	draw(history, series) {
		const ctx = this.ctx;
		const W = this.canvas.width, H = this.canvas.height;
		ctx.clearRect(0, 0, W, H);
		ctx.fillStyle = '#0a0a0a'; ctx.fillRect(0, 0, W, H);
		ctx.strokeStyle = '#222';
		ctx.beginPath(); ctx.moveTo(0, H / 2); ctx.lineTo(W, H / 2); ctx.stroke();
		if (history.length < 2 || !series || series.length === 0) return;

		const T    = history[history.length - 1].t;
		const T0   = Math.max(0, T - this.window);
		const span = (T - T0) || 1;
		const xAt  = t => (t - T0) / span * W;

		for (const s of series) {
			ctx.strokeStyle = s.color; ctx.lineWidth = 1.5;
			ctx.beginPath();
			let first = true;
			for (const h of history) {
				if (h.t < T0) continue;
				const val = h[s.key];
				if (val === undefined || val === null || Number.isNaN(val)) continue;
				const x = xAt(h.t);
				const y = H / 2 - val * s.scale * (H / 2 - 10);
				if (first) { ctx.moveTo(x, y); first = false; } else ctx.lineTo(x, y);
			}
			ctx.stroke();
		}

		// Legend
		ctx.font = '11px ui-monospace'; ctx.textBaseline = 'top';
		let lx = 8;
		for (const s of series) {
			ctx.fillStyle = s.color;
			ctx.fillText(s.label, lx, 6);
			lx += ctx.measureText(s.label).width + 14;
		}
	}
}

// Available signals — key maps to a frame field written in App.tick().
// scale is 1/(max plottable value) so ±max fills half the canvas.
export const PLOT_SIGNALS = {
	th:        { label: 'Tilt angle (±60°)',        scale: 1 / (Math.PI / 3) },
	w:         { label: 'Tilt rate (±6 rad/s)',     scale: 1 / 6 },
	x:         { label: 'Cart position (±2 m)',     scale: 1 / 2 },
	v:         { label: 'Wheel/cart velocity (±3 m/s)', scale: 1 / 3 },
	v_CoM:     { label: 'Body (CoM) velocity (±3 m/s)', scale: 1 / 3 },
	x_CoM:     { label: 'Body (CoM) position (±2 m)',   scale: 1 / 2 },
	F:         { label: 'Force on cart (±Fmax)',    scale: null },   // filled in at render time
	pwm:          { label: 'Motor PWM (±max)',            scale: null },
	pwm_nn:       { label: 'NN PWM shadow (±max)',        scale: null },
	pwm_residual: { label: 'NN residual: actual − NN',    scale: null },
	vel_command:     { label: 'Balance v-command (±5)',   scale: 1 / 5 },
	v_desired: { label: 'Nav target velocity (±3)', scale: 1 / 3 },
	err_x:     { label: 'Position error (±2 m)',    scale: 1 / 2 },
	tilt_sp:   { label: 'Target tilt (±30°)',       scale: 1 / (Math.PI / 6) },
};
