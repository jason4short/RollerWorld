// Feed-forward PWM lookup — maps a commanded wheel speed (m/s) to the
// PWM needed to achieve it at steady state. Mirrors the firmware's
// pwm_LUT_L[] / pwm_LUT_R[] tables.
//
// Why a table? Real motor + drivetrain is nonlinear near zero (deadband,
// stiction) and saturating at the top. A linear ff_per_mps is fine in
// the middle and wrong at both ends. Calibrating a piecewise-linear table
// matches the "ramp PWM, record speed" tuning you'd do on the real bot.
//
// The table is asymmetric-capable but this version mirrors positive-side
// calibration data for negative speeds (real bots often have identical
// forward/reverse response; if not, calibrate in both directions).

export class PWMTable {
	// Construct with an explicit points array, or pass a linearSlope number
	// to fall back to a linear `slope * speed` mapping.
	constructor({ points = null, linearSlope = 250 } = {}) {
		this.points = points;			 // array of {speed, pwm} sorted ascending, speed >= 0
		this.linearSlope = linearSlope;
	}

	get isCalibrated() { return this.points !== null && this.points.length >= 2; }

	// Replace the table from a raw series of calibration points.
	// points: [{pwm, speed}, ...]	with pwm >= 0 (we mirror for negative).
	setFromCalibration(points) {
		const sorted = points
			.filter(p => p.pwm >= 0)
			.slice()
			.sort((a, b) => a.speed - b.speed);
		// Ensure a (0,0) anchor so interpolation through origin is clean.
		if (sorted.length === 0 || sorted[0].speed > 1e-6) {
			sorted.unshift({ speed: 0, pwm: 0 });
		}
		this.points = sorted;
	}

	clear() { this.points = null; }

	// Interpolate PWM for a given speed. Extrapolates by holding the endpoints.
	pwmFromSpeed(speed) {
		if (!this.isCalibrated) return this.linearSlope * speed;
		const sign = speed < 0 ? -1 : 1;
		const s = Math.abs(speed);
		const pts = this.points;
		if (s <= pts[0].speed) return sign * pts[0].pwm;
		if (s >= pts[pts.length - 1].speed) return sign * pts[pts.length - 1].pwm;
		for (let i = 0; i < pts.length - 1; i++) {
			const a = pts[i], b = pts[i + 1];
			if (s >= a.speed && s <= b.speed) {
				const t = (s - a.speed) / (b.speed - a.speed);
				return sign * (a.pwm + t * (b.pwm - a.pwm));
			}
		}
		return 0;
	}
}
