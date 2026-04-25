// Reactive lidar pilot — "follow the road" using only the live lidar scan.
// No map, no path, no goal. Just: long rays on a side ⇒ steer that way,
// open road ahead ⇒ go faster, wall ahead ⇒ slow down.
//
// Two algorithms behind a switch:
//
//   'wsum' — Weighted-sum steering (Jason's car-sim approach).
//     Each ray in a forward arc votes on the steering with a weight that's
//     linear in its body-frame angle. The sum is naturally symmetric and
//     smooth: equal walls left and right cancel out, the bot drives
//     straight; one side opens up, the bot drifts that way. Beautiful for
//     corridor following. Fails when a single obstacle splits the road —
//     the two open sides cancel and the bot drives into the obstacle.
//
//   'ftg'  — Follow-the-Gap.
//     Bubble out the rays within a safety radius of the closest hit (so we
//     never graze the nearest hazard), find the longest contiguous run of
//     "clear" rays, aim at the deepest ray inside it. Picks one fork at a
//     Y-junction instead of averaging into the wall between them. Can
//     chatter when two gaps are nearly equal.
//
// Both algorithms emit a virtual joystick {fwd, yaw} ∈ [-1, +1]² in the
// same convention as the FBW pilot, so the cascade's existing FBW path
// turns it into target_velocity + target_yaw_rate. The bot's pitch and
// yaw loops do the actual driving — this controller only sets WHERE.

// Body-angle width over which we accept rays for the steering decision.
// ±75° matches Jason's car sim (9 rays from -75° to +75°).
const STEER_ARC = (75 * Math.PI) / 180;

// Wider arc for FTG's gap search — we want to consider sideways escape
// routes too, not just the road dead ahead.
const GAP_ARC = Math.PI / 2;          // ±90°

// FTG bubble radius. Rays whose hit point falls within this distance of
// the closest hit get zeroed out before gap search.
const BUBBLE_RADIUS = 0.45;           // meters

// FTG "clear" threshold — rays shorter than this don't count as part of a
// gap. Tuned for our 5 m maxRange + 2.5 m corridor; a ray under this is
// hitting either a corridor wall or an obstacle, both bad.
const GAP_CLEAR_THRESH = 1.8;         // meters

// Minimum forward stick — the bot always creeps a little so it can keep
// sensing and recover from a stop. Mirrors Jason's `speed.min`.
const MIN_FWD = 0.15;

// Speed-curve exponent. >1 compresses near walls (slow down hard when
// close) and expands when the road opens up (eager throttle in clear
// stretches). Jason used 1.5; same here.
const SPEED_EXPONENT = 1.5;

export class RoadController {
	constructor() {
		this.algorithm = 'wsum';     // 'wsum' | 'ftg'
		this.lastFwd   = 0;
		this.lastYaw   = 0;
	}

	setAlgorithm(name) {
		if (name === 'wsum' || name === 'ftg') this.algorithm = name;
	}

	// rays:        Lidar.scan() output — array of { angle, dist, hit_x, hit_z }.
	//              `angle` is world-frame; we subtract heading to get body angle.
	// heading:     bot's world-frame heading (rad).
	// maxRange:    Lidar's max range (m), used to normalize ray distances.
	//
	// Returns { fwd, yaw } in [-1, +1] joystick convention. fwd>0 = forward,
	// yaw>0 = turn left (positive yaw_rate, matching FBW's stick.yaw sign).
	update(rays, heading, maxRange) {
		if (!rays || rays.length === 0) {
			this.lastFwd = 0;
			this.lastYaw = 0;
			return { fwd: 0, yaw: 0 };
		}
		const out = this.algorithm === 'ftg'
			? this._followGap(rays, heading, maxRange)
			: this._weightedSum(rays, heading, maxRange);
		this.lastFwd = out.fwd;
		this.lastYaw = out.yaw;
		return out;
	}

	// Wrap a body angle into [-π, +π]. Lidar rays already span that range,
	// but heading subtraction can push them outside.
	_bodyAngle(worldAngle, heading) {
		let a = worldAngle - heading;
		while (a >  Math.PI) a -= 2 * Math.PI;
		while (a < -Math.PI) a += 2 * Math.PI;
		return a;
	}

	// --- Algorithm 1: Weighted-sum ----------------------------------------
	//
	//   yaw  = Σ (body_angle_i / STEER_ARC) · (dist_i / maxRange)   over forward arc
	//   fwd  = (center_dist / maxRange) ^ 1.5                       (creep floor)
	//
	// Each ray's vote weight is its body angle, normalized to [-1, +1] over
	// the steering arc. Ray on the LEFT (body angle > 0 in our convention)
	// gets a positive weight; long-left ray pushes yaw positive → turn left.
	_weightedSum(rays, heading, maxRange) {
		// Forward speed is set by the ray closest to dead-ahead. We don't
		// just take ray index 12 because the lidar's forward ray and the
		// bot's heading drift relative to each other when yaw is nonzero
		// (they're recomputed each scan, but using the closest-to-zero
		// body angle makes the controller robust to ray-count changes).
		let centerDist     = 0;
		let centerBestAbs  = Infinity;

		// Steering accumulator.
		let steerNum = 0;
		let steerDen = 0;

		for (const r of rays) {
			const ba = this._bodyAngle(r.angle, heading);

			// Center-ray pick.
			const absBa = Math.abs(ba);
			if (absBa < centerBestAbs) {
				centerBestAbs = absBa;
				centerDist    = r.dist;
			}

			// Steering vote — only rays inside the forward arc.
			if (absBa > STEER_ARC) continue;
			const weight = ba / STEER_ARC;       // [-1, +1]
			const dnorm  = r.dist / maxRange;    // [0, 1]
			steerNum += weight * dnorm;
			steerDen += Math.abs(weight);
		}

		const speedFrac = Math.pow(centerDist / maxRange, SPEED_EXPONENT);
		const fwd       = Math.max(MIN_FWD, speedFrac);

		// Steering normalized so a fully-asymmetric scan would yield ±1.
		// Without this normalization, a 5 m wide-open right side would
		// only push yaw to ~0.5 even though the road is begging for it.
		const yaw_raw = steerDen > 0 ? steerNum / steerDen : 0;
		const yaw     = Math.max(-1, Math.min(1, yaw_raw));

		return { fwd, yaw };
	}

	// --- Algorithm 2: Follow-the-Gap --------------------------------------
	//
	//   1) Restrict to forward arc (±90°).
	//   2) Find closest ray. Zero out all rays whose hit-point falls inside
	//      a BUBBLE_RADIUS sphere around it (angular approximation: rays
	//      whose body angle is within asin(R/d) of the closest ray's).
	//   3) Find the longest contiguous run of rays with dist >= GAP_CLEAR.
	//   4) Aim at the DEEPEST ray inside that gap. (Center-of-gap is more
	//      stable but deepest gives a more goal-directed turn.)
	//   5) Forward speed scaled the same way as wsum.
	//
	// References: Sezer & Gokasan, "A novel obstacle avoidance algorithm"
	// (the original FTG paper). F1TENTH adopted this as a baseline.
	_followGap(rays, heading, maxRange) {
		// 1) Forward-arc rays, ordered left-to-right by body angle.
		const arc = [];
		for (const r of rays) {
			const ba = this._bodyAngle(r.angle, heading);
			if (Math.abs(ba) <= GAP_ARC) arc.push({ ba, dist: r.dist });
		}
		arc.sort((a, b) => a.ba - b.ba);
		if (arc.length < 2) return { fwd: MIN_FWD, yaw: 0 };

		// 2) Bubble around the closest hit.
		let closestIdx  = 0;
		let closestDist = Infinity;
		for (let i = 0; i < arc.length; i++) {
			if (arc[i].dist < closestDist) {
				closestDist = arc[i].dist;
				closestIdx  = i;
			}
		}
		if (closestDist > 0 && closestDist < maxRange) {
			// Angular half-width of a sphere of radius R seen from distance d.
			const halfWidth = Math.atan(BUBBLE_RADIUS / Math.max(0.05, closestDist));
			const ba0 = arc[closestIdx].ba;
			for (let i = 0; i < arc.length; i++) {
				if (Math.abs(arc[i].ba - ba0) <= halfWidth) arc[i].dist = 0;
			}
		}

		// 3) Longest contiguous gap of rays >= GAP_CLEAR_THRESH.
		let bestStart = -1, bestLen = 0;
		let curStart  = -1, curLen  = 0;
		for (let i = 0; i < arc.length; i++) {
			if (arc[i].dist >= GAP_CLEAR_THRESH) {
				if (curStart < 0) curStart = i;
				curLen++;
				if (curLen > bestLen) { bestLen = curLen; bestStart = curStart; }
			} else {
				curStart = -1;
				curLen   = 0;
			}
		}

		// 4) Pick a heading. Aim at the gap's CENTER (mean body angle) rather
		// than its deepest ray — the deepest ray jumps around as the bot
		// moves through curves and as one chord-segment seam happens to leak
		// a longer reading than its neighbors. Center is the average over
		// the whole gap, so transient outliers don't move the target much.
		// If no gap clears the threshold (everything is too close), fall
		// back to the deepest ray overall — desperation, but at least it
		// keeps moving.
		let targetBa;
		let targetDist;
		if (bestLen > 0) {
			let baSum = 0;
			let distSum = 0;
			for (let i = bestStart; i < bestStart + bestLen; i++) {
				baSum   += arc[i].ba;
				distSum += arc[i].dist;
			}
			targetBa   = baSum / bestLen;
			targetDist = distSum / bestLen;
		} else {
			let deepest = -1;
			let deepestIdx = closestIdx;
			for (let i = 0; i < arc.length; i++) {
				if (arc[i].dist > deepest) {
					deepest    = arc[i].dist;
					deepestIdx = i;
				}
			}
			targetBa   = arc[deepestIdx].ba;
			targetDist = arc[deepestIdx].dist;
		}

		const yaw       = Math.max(-1, Math.min(1, targetBa / GAP_ARC));
		const speedFrac = Math.pow(targetDist / maxRange, SPEED_EXPONENT);
		const fwd       = Math.max(MIN_FWD, speedFrac);

		return { fwd, yaw };
	}
}
