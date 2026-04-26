// Goal-biased FTG. Same Follow-the-Gap mechanics as RoadController,
// but the gap selection is biased toward the direction of the current
// waypoint instead of just picking the largest opening. Lets the bot
// weave around obstacles while still heading where it's been told.
//
// This bridges the deliberative–reactive gap from the README:
//   - Pure A*       → plan a path on a known map, follow it. Needs map.
//   - Pure FTG      → react to local sensor data, no goal. Drives forever.
//   - Reactive nav  → react locally, but with a "I want to go that way"
//                     pull on the gap-selection score. No map required.
//
// Same {fwd, yaw} virtual-stick interface as RoadController so it plugs
// into the existing FBW path through the cascade.

const FORWARD_ARC      = Math.PI / 2;       // ±90° gap search
const BUBBLE_RADIUS    = 0.45;              // m — closest-hit safety bubble
const GAP_CLEAR_THRESH = 1.8;               // m — rays shorter than this don't count as "clear"
const MIN_FWD          = 0.15;              // creep floor
const SPEED_EXPONENT   = 1.5;               // shape of distance → speed curve
const GOAL_BIAS_GAIN   = 1.5;               // higher = stronger pull toward goal direction
const ARRIVAL_RADIUS   = 0.6;               // m — within this, stop

export class ReactiveNav {
	// rays:    Lidar.scan() output — array of { angle, dist, hit_x, hit_z }.
	// pose:    bot pose { x, z, heading }.
	// target:  goal in world frame { x, z }.
	// maxRange: lidar max range (m), used to normalize distances.
	//
	// Returns { fwd, yaw } in [-1, +1] joystick convention.
	update(rays, pose, target, maxRange) {
		if (!rays || rays.length === 0 || !target) return { fwd: 0, yaw: 0 };

		// Goal vector in world; bail if we're already on top of it.
		const dx = target.x - pose.x;
		const dz = (target.z ?? 0) - (pose.z ?? 0);
		const goalDist = Math.hypot(dx, dz);
		if (goalDist < ARRIVAL_RADIUS) return { fwd: 0, yaw: 0 };

		// Goal angle in body frame. Lidar angles use the same world-frame
		// convention (atan2 of (-dz, dx)), so the arithmetic matches.
		const goalWorldAngle = Math.atan2(-dz, dx);
		let goalBa = goalWorldAngle - pose.heading;
		while (goalBa >  Math.PI) goalBa -= 2 * Math.PI;
		while (goalBa < -Math.PI) goalBa += 2 * Math.PI;

		// 1) Forward-arc rays sorted left-to-right by body angle.
		const arc = [];
		for (const r of rays) {
			let ba = r.angle - pose.heading;
			while (ba >  Math.PI) ba -= 2 * Math.PI;
			while (ba < -Math.PI) ba += 2 * Math.PI;
			if (Math.abs(ba) <= FORWARD_ARC) arc.push({ ba, dist: r.dist });
		}
		arc.sort((a, b) => a.ba - b.ba);
		if (arc.length < 2) return { fwd: MIN_FWD, yaw: 0 };

		// 2) Bubble around the closest hit.
		let closestIdx = 0;
		let closestDist = Infinity;
		for (let i = 0; i < arc.length; i++) {
			if (arc[i].dist < closestDist) {
				closestDist = arc[i].dist;
				closestIdx  = i;
			}
		}
		if (closestDist > 0 && closestDist < maxRange) {
			const halfWidth = Math.atan(BUBBLE_RADIUS / Math.max(0.05, closestDist));
			const ba0 = arc[closestIdx].ba;
			for (let i = 0; i < arc.length; i++) {
				if (Math.abs(arc[i].ba - ba0) <= halfWidth) arc[i].dist = 0;
			}
		}

		// 3) Find every contiguous run of rays clearing the threshold.
		const gaps = [];
		let curStart = -1;
		for (let i = 0; i < arc.length; i++) {
			if (arc[i].dist >= GAP_CLEAR_THRESH) {
				if (curStart < 0) curStart = i;
			} else if (curStart >= 0) {
				gaps.push({ start: curStart, end: i - 1 });
				curStart = -1;
			}
		}
		if (curStart >= 0) gaps.push({ start: curStart, end: arc.length - 1 });

		// 4) Score each gap. The aim point inside a gap is the goal
		// direction CLAMPED to the gap's range — so when the goal falls
		// inside an open gap, the bot heads straight at the goal instead
		// of at the gap's geometric center. (The previous version aimed
		// at gap center regardless of goal — that's why the bot drove
		// dead ahead even on clear stretches with the goal off to one
		// side.) Score is alignment of the aim point with the goal,
		// plus a small width bonus to prefer roomier gaps when there's
		// a tie.
		let bestGap = null;
		let bestScore = -Infinity;
		for (const g of gaps) {
			const startBa  = arc[g.start].ba;
			const endBa    = arc[g.end].ba;
			const width    = endBa - startBa;
			const aimBa    = Math.max(startBa, Math.min(endBa, goalBa));
			const alignment = Math.cos(aimBa - goalBa);
			// alignment is the dominant term; width is a tiebreaker.
			const score = GOAL_BIAS_GAIN * alignment + 0.2 * width;
			if (score > bestScore) {
				bestScore = score;
				bestGap   = { aimBa, ...g };
			}
		}

		if (!bestGap) {
			// Everything's blocked. Slow creep, no steering — better than
			// freezing, and the goal is still pulling so a small heading
			// change next tick may open a gap up.
			return { fwd: MIN_FWD, yaw: 0 };
		}

		// 5) Output: aim at the chosen gap's clamped-to-goal direction.
		// Speed scaled by deepest ray inside the chosen gap.
		let deepest = 0;
		for (let i = bestGap.start; i <= bestGap.end; i++) {
			if (arc[i].dist > deepest) deepest = arc[i].dist;
		}
		const yaw = Math.max(-1, Math.min(1, bestGap.aimBa / FORWARD_ARC));
		const speedFrac = Math.pow(deepest / maxRange, SPEED_EXPONENT);
		const fwd = Math.max(MIN_FWD, speedFrac);
		return { fwd, yaw };
	}
}
