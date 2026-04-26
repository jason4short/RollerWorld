// Road sensor — a "color camera" that returns distance-to-road-edge
// per ray. Mirrors `Lidar`'s interface so the existing `RoadController`
// (WSum / FTG) can switch sensor families without code changes.
//
// Mechanism: from the bot's position, walk along each ray pixel-by-pixel
// through the painted road canvas. Stop at the first non-road pixel,
// return that distance. Bot is assumed to be on-road when the scan
// starts; if not (driving off the road), every ray returns 0 and the
// reactive controller sees a pure dead-zone, which is the right
// behavior — a self-driving car that lost its lane should slow down,
// not steer aggressively.
//
// This is the bot's analogue of a downward-looking RGB camera doing
// lane detection. The colors in the scene are deliberately picked so
// "is this pixel road?" is a single-channel threshold — no ML, no
// convolution, just darkness vs grass. Future lessons can swap the
// classifier for something more interesting.

export class RoadSensor {
	constructor({ rays = 24, maxRange = 8, fov = 2 * Math.PI } = {}) {
		this.rays     = rays;
		this.maxRange = maxRange;
		this.fov      = fov;
		this.last = new Array(rays).fill(null).map(() => ({
			angle: 0, dist: 0, hit_x: 0, hit_z: 0,
		}));
	}

	// state: { x, z, heading } — bot pose in world frame.
	// roadCanvas: a RoadCanvas instance. We use its `pxPerMeter` to size
	// our walking step (≈ one pixel per step) and `isRoadAt` for the
	// per-pixel lookup.
	scan(state, roadCanvas) {
		const { rays, maxRange, fov } = this;
		const step  = fov / rays;
		const start = -fov / 2;
		// Step ~one canvas pixel per loop. Slightly larger than 1/pxPerMeter
		// so on shallow rays we don't miss the edge by re-sampling the same
		// pixel. Trades a tiny accuracy hit for speed.
		const stride = 1.1 / roadCanvas.pxPerMeter;

		for (let i = 0; i < rays; i++) {
			const a    = start + i * step;
			const wa   = state.heading + a;
			const dirX =  Math.cos(wa);
			const dirZ = -Math.sin(wa);

			let dist = maxRange;
			// Start a hair off the bot's center to skip its own footprint.
			for (let d = 0.15; d < maxRange; d += stride) {
				const sx = state.x + dirX * d;
				const sz = (state.z ?? 0) + dirZ * d;
				if (!roadCanvas.isRoadAt(sx, sz)) {
					dist = d;
					break;
				}
			}

			const out = this.last[i];
			out.angle = wa;
			out.dist  = dist;
			out.hit_x = state.x + dirX * dist;
			out.hit_z = (state.z ?? 0) + dirZ * dist;
		}
		return this.last;
	}
}
