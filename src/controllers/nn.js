// Neural-net controller. Replaces the full ArduBalance cascade with a
// single MLP that maps raw sensor state → PWM. The MLP is trained
// offline (see trainer.js) to mimic ArduBalance; at runtime we just
// evaluate it forward.

export class NNController {
	constructor() {
		this.mlp          = null;               // set by the trainer
		this.target_angle = 0;                  // pilot/nav setpoint (like ArduBalance)
		this.lastPWM      = 0;
		this.last_v       = 0;                  // for finite-differencing dv/dt
		// These must match NNTrainer's normalization (and order).
		//           th,              w,      x,      v,     target_angle,        dv
		this.inScale  = [1 / (Math.PI / 3), 1 / 10, 1 / 5, 1 / 3, 1 / (Math.PI / 6), 1 / 10];
		this.outScale = 2000;   // denormalize NN output → PWM
	}

	reset() {
		this.lastPWM = 0;
		this.last_v  = 0;
	}

	// Single-rate: no outer loop, the MLP replaces the whole cascade.
	updateOuter() {}

	produceForce(sensors, gains, dt, motor) {
		if (!this.mlp) return 0;

		// Finite-difference cart acceleration from successive encoder velocity
		// samples. This is the D-term input that the stateless NN needs to
		// replicate ArduBalance's `wheel_D * d(v)/dt` behavior.
		const dv = (sensors.v - this.last_v) / dt;
		this.last_v = sensors.v;

		const x = [
			sensors.th           * this.inScale[0],
			sensors.w            * this.inScale[1],
			sensors.x            * this.inScale[2],
			sensors.v            * this.inScale[3],
			this.target_angle    * this.inScale[4],
			dv                   * this.inScale[5],
		];
		const out = this.mlp.forward(x);
		let pwm = out[0] * this.outScale;

		// Deadband compensation — NN's smoothed output is often below the
		// motor deadband, producing zero thrust. Hop over it (same trick
		// ArduBalance's `dead_zone` gain does), but keep zero at zero.
		const EPS = 1e-6;
		if (pwm >  EPS) pwm += motor.deadband;
		if (pwm < -EPS) pwm -= motor.deadband;

		const PM = motor.PWM_max;
		if (pwm >  PM) pwm =  PM;
		if (pwm < -PM) pwm = -PM;
		this.lastPWM = pwm;
		return motor.forceFromPWM(pwm, sensors.v);
	}
}
