// Planner — turns "go to point X" into "follow this path of waypoints."
//
// Sits above Nav in the cascade. Nav doesn't change: it still chases one
// waypoint at a time. The Planner just decides which sequence Nav sees.
//
// Two modes:
//
//   direct — current behavior. Path = [goal]. Nav drives in a straight line,
//            which works fine when nothing is in the way.
//
//   astar  — A* search over a coarse grid; obstacle cells are blocked; the
//            resulting path is simplified by line-of-sight smoothing so
//            you get corner-turning waypoints, not a zigzag along grid edges.
//
// Pedagogical pairing with the layer NNs: planning is *all branches* —
// graph search, discrete decisions, sharp gates. A* in 60 lines beats any
// MLP planner with 60 hidden units. Same lesson as Nav-NN-vs-rule, but
// flipped: classical wins decisively where the problem is fundamentally
// branchy. ML earns its keep on the heuristic, not the search.

const DIRS = [   // 8-connected neighbors with diagonal cost √2
	{ dx:  1, dz:  0, cost: 1     },
	{ dx: -1, dz:  0, cost: 1     },
	{ dx:  0, dz:  1, cost: 1     },
	{ dx:  0, dz: -1, cost: 1     },
	{ dx:  1, dz:  1, cost: 1.414 },
	{ dx:  1, dz: -1, cost: 1.414 },
	{ dx: -1, dz:  1, cost: 1.414 },
	{ dx: -1, dz: -1, cost: 1.414 },
];

export class Planner {
	constructor() {
		this.mode = 'direct';   // 'direct' | 'astar'
		this.goal = null;       // last commanded goal {x, z}
		this.path = [];         // current planned path as {x, z}[]
	}

	setMode(mode) { this.mode = mode; }

	// Compute a path. Returns the array (also stored on the instance).
	// world: { bot: {x, z}, obstacles, res?, pad? }
	plan(start, goal, world) {
		this.goal = { x: goal.x, z: goal.z };
		// Both 'astar' (ground truth) and 'lidar_astar' (discovered map) run
		// A* — the only difference is which `obstacles` was passed in. The
		// search is identical; the world model is the experimental variable.
		const useAstar = (this.mode === 'astar' || this.mode === 'lidar_astar')
		              && world?.obstacles;
		if (useAstar) {
			const raw = Planner.astar(start, goal, world.obstacles,
				world.res ?? 0.25, world.pad ?? 0.25);
			this.path = raw.length > 0
				? Planner.smooth(raw, world.obstacles, world.pad ?? 0.25)
				: [{ x: goal.x, z: goal.z }];   // unreachable: fall back to direct
		} else {
			this.path = [{ x: goal.x, z: goal.z }];
		}
		return this.path;
	}

	// ── A* on a uniform grid ───────────────────────────────────────────────
	// Cells indexed by (i, j) where x = i·res, z = j·res. Open set is a
	// plain array sorted by f-score on each pop — fine for the scale of
	// problems we plan (a few hundred cells). Swap in a heap if it bites.
	static astar(start, goal, obstacles, res = 0.25, pad = 0.25) {
		const toCell    = p => ({ i: Math.round(p.x / res), j: Math.round(p.z / res) });
		const toWorld   = c => ({ x: c.i * res, z: c.j * res });
		const cellKey   = c => `${c.i},${c.j}`;
		const cellOpen  = c => !obstacles.isBlocked(c.i * res, c.j * res, pad);

		const sCell = toCell(start), gCell = toCell(goal);
		if (!cellOpen(gCell)) return [];   // goal itself is blocked

		const heur = c => Math.hypot(c.i - gCell.i, c.j - gCell.j) * res;

		const open = [{ ...sCell, g: 0, f: heur(sCell), parent: null }];
		const seen = new Map();   // cellKey → best g-score
		seen.set(cellKey(sCell), 0);

		// Bound the search so a clicked-into-the-void target doesn't expand
		// forever. Real maps would have a navmesh boundary; we have a
		// roughly-rectangular operating area, so cap the explored cell count.
		const MAX_NODES = 4000;
		let popped = 0;

		while (open.length && popped < MAX_NODES) {
			// Pop lowest-f node — linear scan, sufficient for these sizes.
			let bestIdx = 0;
			for (let k = 1; k < open.length; k++) {
				if (open[k].f < open[bestIdx].f) bestIdx = k;
			}
			const cur = open.splice(bestIdx, 1)[0];
			popped++;

			if (cur.i === gCell.i && cur.j === gCell.j) {
				// Reconstruct path back through parents.
				const cells = [];
				for (let n = cur; n; n = n.parent) cells.push(n);
				cells.reverse();
				return cells.map(toWorld);
			}

			for (const d of DIRS) {
				const next = { i: cur.i + d.dx, j: cur.j + d.dz };
				if (!cellOpen(next)) continue;
				const g = cur.g + d.cost * res;
				const key = cellKey(next);
				if (g >= (seen.get(key) ?? Infinity)) continue;
				seen.set(key, g);
				open.push({ ...next, g, f: g + heur(next), parent: cur });
			}
		}
		return [];   // exhausted or capped — caller falls back
	}

	// Theta*-style smoothing. Walk the path; from each kept waypoint, jump
	// as far ahead as line-of-sight allows. Result: small number of
	// corner-turning waypoints rather than a grid-aligned zigzag.
	static smooth(path, obstacles, pad = 0.25) {
		if (path.length <= 2) return path;
		const out = [path[0]];
		let i = 0;
		while (i < path.length - 1) {
			let j = path.length - 1;
			// Find the farthest j visible from i. Scan back if blocked.
			while (j > i + 1 && !obstacles.hasLineOfSight(path[i], path[j], pad)) j--;
			out.push(path[j]);
			i = j;
		}
		return out;
	}
}
