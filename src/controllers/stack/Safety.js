// Safety — modulates Nav's velocity command based on forward lidar.
//
// Sits between Nav and Mixer in the cascade. When something is close in
// the bot's forward arc, scales `vel_target_body` toward zero so the bot
// brakes before hitting it. The closer it gets, the slower it goes;
// inside `distMin` the scale is 0 (full stop).
//
// Only modulates forward velocity. Reverse commands pass through
// unchanged — the lidar's forward arc says nothing about what's behind.
//
// Pedagogical note: in real flight stacks this is the "object avoidance"
// or "geofence" layer. It doesn't decide where to go (Nav's job) or how
// to lean (Mixer's job) — it just clips the velocity command to keep
// the bot out of trouble. Same locality-of-failure trick: when the bot
// brakes mysteriously, you know to look HERE.

export class Safety {
	constructor() {
		this.lastScale = 1;       // last applied factor, exposed for plotting
		this.lastMinDist = 0;     // last min forward range observed
	}

	// Compute the forward-clearance scale [0, 1] for a given lidar scan.
	// `forwardArc` is the half-angle (radians) on either side of heading
	// that we treat as "ahead." 45° (π/4) is generous enough for steering
	// margin; tighter would only brake on direct collisions.
	static computeScale({ lidar, heading, forwardArc, distMin, distMax }) {
		if (!lidar || lidar.length === 0) return { scale: 1, minDist: Infinity };
		let minDist = Infinity;
		for (const r of lidar) {
			let aw = r.angle - heading;
			while (aw >  Math.PI) aw -= 2 * Math.PI;
			while (aw < -Math.PI) aw += 2 * Math.PI;
			if (Math.abs(aw) > forwardArc) continue;
			if (r.dist < minDist) minDist = r.dist;
		}
		if (minDist === Infinity) return { scale: 1, minDist };
		// Linear ramp: 0 at distMin, 1 at distMax.
		const scale = Math.max(0, Math.min(1, (minDist - distMin) / (distMax - distMin)));
		return { scale, minDist };
	}

	// navOut: { vel_target_body, heading_target, ... }
	// sensors: { lidar, heading, ... }
	// gains: { distMin, distMax, forwardArc, enabled }
	apply(navOut, sensors, gains = {}) {
		const enabled = gains.enabled ?? true;
		const vel = navOut.vel_target_body ?? 0;
		// Reverse or zero command — no safety. Forward only.
		if (!enabled || vel <= 0 || !sensors.lidar) {
			this.lastScale   = 1;
			this.lastMinDist = sensors.lidar ? this._minForward(sensors, gains) : Infinity;
			return navOut;
		}
		const { scale, minDist } = Safety.computeScale({
			lidar:      sensors.lidar,
			heading:    sensors.heading,
			forwardArc: gains.forwardArc ?? Math.PI / 4,
			distMin:    gains.distMin    ?? 0.5,
			distMax:    gains.distMax    ?? 2.5,
		});
		this.lastScale   = scale;
		this.lastMinDist = minDist;
		return { ...navOut, vel_target_body: vel * scale };
	}

	// Diagnostic helper — exposes min-forward distance even when not
	// modulating (e.g. user stopped, bot stationary). Useful for plotting.
	_minForward(sensors, gains) {
		const r = Safety.computeScale({
			lidar:      sensors.lidar,
			heading:    sensors.heading,
			forwardArc: gains.forwardArc ?? Math.PI / 4,
			distMin:    0, distMax: 1,    // values irrelevant — we want minDist
		});
		return r.minDist;
	}
}
