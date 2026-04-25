// OccupancyGrid — log-odds 2D map built up from lidar scans.
//
// Each cell stores a log-odds score:
//   lo > +threshold  → "occupied" (something has been seen here)
//   lo < -threshold  → "free"     (a ray passed through here)
//   |lo| ≤ threshold → "unknown"  (never observed)
//
// Each scan integrates as: cells along the ray get nudged toward free,
// the cell at the ray's hit point gets nudged toward occupied. Repeat
// observations sharpen the belief; a single random hit doesn't lock a
// cell as occupied forever.
//
// This is exactly what real SLAM-style mapping does, minus the
// localization side (we cheat — we use ground-truth bot pose). For a
// teaching sim the mapping half is the interesting half: how does the
// bot's view of the world DIFFER from the world?
//
// Duck-types as Obstacles for the Planner (isBlocked, hasLineOfSight),
// so it slots into the existing A* without a separate code path. The
// Planner doesn't care whether walls are ground truth or discovered.

const FREE_INC = -0.4;     // log-odds delta for a free observation
const OCC_INC  = +0.85;    // log-odds delta for an occupied observation
const LO_CLAMP =  4.0;     // saturation, both signs
const OCC_THRESH = 0.5;    // |lo| > this → confident

export class OccupancyGrid {
	// Origin is the world coordinate of cell (0, 0)'s lower corner.
	// Width × height in meters; cellSize controls resolution.
	constructor({ originX = -5, originZ = -4, width = 25, height = 8, cellSize = 0.25 } = {}) {
		this.originX  = originX;
		this.originZ  = originZ;
		this.width    = width;
		this.height   = height;
		this.cellSize = cellSize;
		this.cols = Math.ceil(width  / cellSize);
		this.rows = Math.ceil(height / cellSize);
		this.lo = new Float32Array(this.cols * this.rows);   // init 0 (unknown)
	}

	clear() { this.lo.fill(0); }

	cellAt(x, z) {
		return {
			i: Math.floor((x - this.originX) / this.cellSize),
			j: Math.floor((z - this.originZ) / this.cellSize),
		};
	}

	inBounds(i, j) { return i >= 0 && i < this.cols && j >= 0 && j < this.rows; }
	idx(i, j)       { return j * this.cols + i; }

	_bump(i, j, delta) {
		if (!this.inBounds(i, j)) return;
		const k = this.idx(i, j);
		let v = this.lo[k] + delta;
		if (v >  LO_CLAMP) v =  LO_CLAMP;
		if (v < -LO_CLAMP) v = -LO_CLAMP;
		this.lo[k] = v;
	}

	// Walk a single ray and update cells along it. Cells from origin to hit
	// get the free nudge; the hit cell gets the occupied nudge (if the ray
	// hit something — at maxRange the ray went into the void, so all cells
	// along it are free, no occupied stamp).
	integrate(origin, hit, maxRange) {
		const dx = hit.x - origin.x, dz = hit.z - origin.z;
		const dist = Math.hypot(dx, dz);
		if (dist < 1e-6) return;
		const wasHit = dist < maxRange - 1e-3;

		const step = this.cellSize * 0.5;
		const n = Math.max(1, Math.floor(dist / step));
		const ux = dx / dist, uz = dz / dist;
		let lastK = -1;
		for (let s = 0; s < n; s++) {
			const t = s * step;
			const x = origin.x + ux * t;
			const z = origin.z + uz * t;
			const c = this.cellAt(x, z);
			if (!this.inBounds(c.i, c.j)) continue;
			const k = this.idx(c.i, c.j);
			if (k === lastK) continue;   // don't double-bump the same cell
			lastK = k;
			this._bump(c.i, c.j, FREE_INC);
		}
		if (wasHit) {
			const c = this.cellAt(hit.x, hit.z);
			this._bump(c.i, c.j, OCC_INC);
		}
	}

	integrateScan(origin, rays, maxRange) {
		for (const r of rays) this.integrate(origin, { x: r.hit_x, z: r.hit_z }, maxRange);
	}

	// ── Planner-facing duck-type with Obstacles ────────────────────────────
	// Cells are only blocked when CONFIDENTLY occupied. Unknown cells are
	// optimistic-traversable (the bot will plan into them and discover them
	// as it goes); confidently-free cells are obviously fine.
	//
	// `pad` inflates walls by that distance — a cell is blocked if itself
	// or any neighbor within `pad` meters is occupied. Without inflation,
	// A* would route paths through cells immediately next to walls, and
	// the Safety governor would then brake the bot before it could follow.
	// Pad must be ≥ Safety.distMin to keep the planner out of the
	// brake zone.
	isBlocked(x, z, pad = 0) {
		const c = this.cellAt(x, z);
		if (!this.inBounds(c.i, c.j)) return false;
		const r = Math.ceil(pad / this.cellSize);
		for (let dj = -r; dj <= r; dj++) {
			for (let di = -r; di <= r; di++) {
				const ci = c.i + di, cj = c.j + dj;
				if (!this.inBounds(ci, cj)) continue;
				if (this.lo[this.idx(ci, cj)] > OCC_THRESH) return true;
			}
		}
		return false;
	}

	hasLineOfSight(a, b, pad = 0) {
		const dist = Math.hypot(b.x - a.x, b.z - a.z);
		const steps = Math.max(2, Math.ceil(dist / 0.05));
		for (let i = 1; i < steps; i++) {
			const t = i / steps;
			if (this.isBlocked(a.x + (b.x - a.x) * t, a.z + (b.z - a.z) * t, pad)) return false;
		}
		return true;
	}
}
