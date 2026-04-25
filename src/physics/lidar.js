// Lidar — fan of raycasts from the bot, returning the distance to the
// nearest wall along each ray (or maxRange if nothing was hit).
//
// This is a sensor in the same sense as the IMU and encoder: it's a
// noisy, finite-resolution view of the world, distinct from ground
// truth. For now, no noise — distances are exact. Optional Gaussian
// noise lives behind cfg.noise so a future lesson can show how range
// noise propagates into perception → planning.
//
// What controllers do with these readings is a separate decision. The
// first use is purely visual (the Renderer draws each ray); later
// stones — dynamic obstacle avoidance, occupancy grids, learned
// perception — will consume the array directly.

export class Lidar {
	constructor({ rays = 24, maxRange = 5, fov = 2 * Math.PI, noise = 0 } = {}) {
		this.rays     = rays;
		this.maxRange = maxRange;
		this.fov      = fov;       // 2π = full circle around the bot
		this.noise    = noise;     // 1-sigma Gaussian (m), 0 disables
		// Pre-allocated output buffer reused each scan.
		this.last = new Array(rays).fill(null).map(() => ({
			angle: 0, dist: 0, hit_x: 0, hit_z: 0,
		}));
	}

	// state: { x, z, heading } — bot pose in world frame.
	// obstacles: an Obstacles instance with .castRay().
	scan(state, obstacles) {
		const { rays, maxRange, fov } = this;
		const step = fov / rays;
		const start = -fov / 2;   // ray 0 is fov/2 to the left of forward
		// Bot's forward in world coords is (cos h, -sin h) for (x, z).
		for (let i = 0; i < rays; i++) {
			const a    = start + i * step;            // body-frame ray angle
			const wa   = state.heading + a;            // world-frame ray angle
			const dirX =  Math.cos(wa);
			const dirZ = -Math.sin(wa);
			let dist = obstacles.castRay(
				{ x: state.x, z: state.z ?? 0 },
				{ x: dirX, z: dirZ },
				maxRange,
			);
			if (this.noise > 0) dist += Lidar._randn() * this.noise;
			if (dist < 0)         dist = 0;
			if (dist > maxRange)  dist = maxRange;

			const out = this.last[i];
			out.angle = wa;
			out.dist  = dist;
			out.hit_x = state.x + dirX * dist;
			out.hit_z = (state.z ?? 0) + dirZ * dist;
		}
		return this.last;
	}

	static _randn() {
		const u = Math.max(1e-12, Math.random());
		const v = Math.random();
		return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
	}
}
