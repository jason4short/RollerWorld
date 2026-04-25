// Simulates what the firmware actually observes:
//   - IMU pitch angle + gyro rate, with Gaussian noise
//   - Wheel encoder: tick-quantized BODY-FRAME distance (matches physical
//     reality — encoders read wheel rotation, not world position). Speed
//     is derived from tick delta over the sensor period.
//   - Yaw heading (compass) + yaw rate (gyro Z), with Gaussian noise.
//
// Without this layer, controllers see "god-mode" state and tune too hot.

export class Sensors {
	constructor() {
		this.reset();
	}

	reset() {
		this.bodyDistance = 0;   // integrated body-frame distance (Sensor's own dead reckoning)
		this.lastTicks    = 0;
		this.lastTime     = 0;
		this.lastSpeed    = 0;
	}

	// Gaussian via Box–Muller.
	_randn() {
		const u = Math.max(1e-12, Math.random());
		const v = Math.random();
		return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
	}

	// params: plant params (we need R and L)
	// cfg:    { ticks_per_rev, imu_noise, gyro_noise }
	sample(plantState, params, cfg, now) {
		const { R, L } = params;
		const circ = 2 * Math.PI * R;

		// Integrate body-frame velocity into our own distance accumulator.
		// This is what the physical encoder counts: wheel rotation = body
		// forward distance, regardless of which world-frame direction the
		// bot is moving in.
		const dt = now - this.lastTime;
		let speed = this.lastSpeed;
		if (dt > 1e-9) {
			this.bodyDistance += plantState.v * dt;
			const ticks = Math.round((this.bodyDistance / circ) * cfg.ticks_per_rev);
			const dTicks = ticks - this.lastTicks;
			speed = (dTicks * circ / cfg.ticks_per_rev) / dt;
			this.lastTicks = ticks;
			this.lastTime  = now;
			this.lastSpeed = speed;
		}

		// World-frame position (truth from plant — no encoder dead-reckoning
		// drift in the sim yet; nav uses these for distance/heading to target).
		const x = plantState.x;
		const z = plantState.z;
		const v = speed;   // body-frame velocity (encoder-quantized)

		const th = plantState.th + this._randn() * cfg.imu_noise;
		const w  = plantState.w  + this._randn() * cfg.gyro_noise;

		// CoM position in world (small-θ correction); useful for 1D nav,
		// kept for backward compat though 2D nav uses x/z directly.
		const cs = Math.cos(th);
		const sn = Math.sin(th);
		const x_CoM = x + L * sn;
		const v_CoM = v + L * cs * w;

		const psi     = plantState.psi;
		const yawRate = plantState.yawRate + this._randn() * cfg.gyro_noise;

		// `x_body` is the integrated body-frame distance the wheel encoder
		// has actually measured — independent of heading. The pitch
		// controller's "drift correction" term uses this so a yaw turn
		// doesn't leave a stale world-x error pulling the bot over.
		const x_body = this.bodyDistance;

		return { x, v, th, w, x_CoM, v_CoM, psi, yawRate, z, x_body };
	}
}
