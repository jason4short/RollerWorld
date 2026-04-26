import * as THREE from 'three';

// Visual-only obstacles (collision/lidar will come later). Builds a course
// of walls, ring gates, tunnels, and flag markers as a single group that
// the renderer can drop into its scene.
//
// Each `add*` method returns the mesh/group it created so the caller can
// keep references for collision detection later.

export class Obstacles {
	constructor() {
		this.group = new THREE.Group();
		this.items = [];   // [{ type, position, ...meta }] — for future collision/lidar use
	}

	clear() {
		while (this.group.children.length) this.group.remove(this.group.children[0]);
		this.items.length = 0;
	}

	// --- Walls --------------------------------------------------------------
	// Chunky cream-colored block, axis-aligned or yaw-rotated.
	addWall({ x, z, length = 1, height = 0.35, thickness = 0.18, yaw = 0, color = 0xf2e6cf }) {
		const mat  = new THREE.MeshStandardMaterial({ color, roughness: 0.95 });
		const mesh = new THREE.Mesh(new THREE.BoxGeometry(length, height, thickness), mat);
		mesh.position.set(x, height / 2, z);
		mesh.rotation.y = yaw;
		mesh.castShadow = true;
		mesh.receiveShadow = true;
		this.group.add(mesh);
		this.items.push({ type: 'wall', position: [x, z], length, thickness, yaw });
		return mesh;
	}

	// --- Hoop (inverted-U arch) --------------------------------------------
	// Like a croquet wicket: feet on the ground straddling the bot's path,
	// arch curving overhead. Bot drives through the opening underneath.
	addRing({ x, z, radius = 0.8, tubeRadius = 0.12, yaw = 0, color = 0xcc3322 }) {
		// Inverted-U arch in cream/red, chunky-poly to match the world style.
		const mat = new THREE.MeshStandardMaterial({ color, roughness: 0.95 });
		const geom = new THREE.TorusGeometry(radius, tubeRadius, 8, 16, Math.PI);
		const arch = new THREE.Mesh(geom, mat);
		arch.rotation.y = Math.PI / 2;
		arch.castShadow = true;
		const wrap = new THREE.Group();
		wrap.add(arch);
		wrap.position.set(x, 0, z);
		wrap.rotation.y = yaw;
		this.group.add(wrap);
		this.items.push({ type: 'hoop', position: [x, z], radius, yaw });
		return wrap;
	}

	// --- Tunnel -------------------------------------------------------------
	// Half-cylinder arch the bot drives through. Like a covered bridge.
	addTunnel({ x, z, length = 1.5, radius = 0.9, yaw = 0, color = 0xf2e6cf }) {
		const mat = new THREE.MeshStandardMaterial({
			color, side: THREE.DoubleSide, roughness: 0.95,
		});
		// CylinderGeometry open on top half (thetaStart, thetaLength) gives the arch.
		const geom = new THREE.CylinderGeometry(
			radius, radius, length, 24, 1, true,   // open ends
			0, Math.PI,                              // half cylinder
		);
		const mesh = new THREE.Mesh(geom, mat);
		mesh.rotation.x = -Math.PI / 2;
		mesh.rotation.z = -Math.PI / 2;
		mesh.castShadow = true;
		mesh.receiveShadow = true;
		const wrap = new THREE.Group();
		wrap.add(mesh);
		wrap.position.set(x, 0, z);
		wrap.rotation.y = yaw;
		this.group.add(wrap);
		this.items.push({ type: 'tunnel', position: [x, z], length, radius, yaw });
		return wrap;
	}

	// --- Flag (decorative) --------------------------------------------------
	// Use for course markers / waypoints / start-finish. Distinct from the
	// nav target flag (which is animated).
	addFlag({ x, z, color = 0xcc3322, height = 1.1 }) {
		const g = new THREE.Group();
		const pole = new THREE.Mesh(
			new THREE.CylinderGeometry(0.05, 0.05, height, 8),
			new THREE.MeshStandardMaterial({ color: 0xf2e6cf }),
		);
		pole.position.y = height / 2;
		pole.castShadow = true;
		g.add(pole);
		const flag = new THREE.Mesh(
			new THREE.BoxGeometry(0.44, 0.28, 0.04),
			new THREE.MeshStandardMaterial({ color }),
		);
		flag.position.set(0.22, height - 0.18, 0);
		flag.castShadow = true;
		g.add(flag);
		g.position.set(x, 0, z);
		this.group.add(g);
		this.items.push({ type: 'flag', position: [x, z] });
		return g;
	}

	// --- Collision queries (used by the Planner) ---------------------------
	// Only walls block. Hoops and tunnels are pass-through (the bot drives
	// under/through them); flags are decorative.
	//
	// `pad` adds bot-radius padding so the planner routes around walls with
	// a margin instead of grazing them.

	// Snap a point to the nearest unblocked location (or return it
	// unchanged if already free). Used by shift-click waypoint
	// placement so a flag that lands inside a tree visually moves to
	// the nearest open spot instead of becoming an unreachable goal.
	// Spiral search at `step` resolution out to `maxRadius`; gives up
	// and returns the original point if nothing nearby is free.
	nearestFree(x, z, pad = 0.4, step = 0.25, maxRadius = 4) {
		if (!this.isBlocked(x, z, pad)) return { x, z };
		// Spiral by rings of increasing radius. At each ring sample N
		// directions; first free one wins.
		for (let r = step; r <= maxRadius; r += step) {
			const samples = Math.max(8, Math.ceil((2 * Math.PI * r) / step));
			for (let i = 0; i < samples; i++) {
				const a = (i / samples) * 2 * Math.PI;
				const sx = x + Math.cos(a) * r;
				const sz = z + Math.sin(a) * r;
				if (!this.isBlocked(sx, sz, pad)) return { x: sx, z: sz };
			}
		}
		return { x, z };   // give up
	}

	isBlocked(x, z, pad = 0.2) {
		for (const it of this.items) {
			if (it.type !== 'wall') continue;
			// AABB test in the wall's rotated local frame.
			const dx = x - it.position[0];
			const dz = z - it.position[1];
			const cs = Math.cos(-it.yaw);
			const sn = Math.sin(-it.yaw);
			const lx = dx * cs - dz * sn;
			const lz = dx * sn + dz * cs;
			if (Math.abs(lx) < it.length    / 2 + pad &&
			    Math.abs(lz) < it.thickness / 2 + pad) return true;
		}
		return false;
	}

	// Cast a ray from `origin` in `direction` (unit vector in x/z), return
	// the distance to the nearest wall hit, or `maxRange` if nothing hit.
	// Slab method against each wall's rotated AABB — exact, allocation-free.
	castRay(origin, direction, maxRange = 8) {
		let minDist = maxRange;
		for (const it of this.items) {
			if (it.type !== 'wall') continue;
			// Transform ray into the wall's local (yaw-cancelled) frame.
			const dx = origin.x - it.position[0];
			const dz = origin.z - it.position[1];
			const cs = Math.cos(-it.yaw), sn = Math.sin(-it.yaw);
			const lox = dx           * cs - dz           * sn;
			const loz = dx           * sn + dz           * cs;
			const ldx = direction.x  * cs - direction.z  * sn;
			const ldz = direction.x  * sn + direction.z  * cs;

			const hx = it.length    / 2;
			const hz = it.thickness / 2;

			// Slab intersection — tolerate ldx/ldz of 0 via infinities.
			const inv_x = ldx !== 0 ? 1 / ldx : Infinity;
			const inv_z = ldz !== 0 ? 1 / ldz : Infinity;
			const t1x = (-hx - lox) * inv_x, t2x = (hx - lox) * inv_x;
			const t1z = (-hz - loz) * inv_z, t2z = (hz - loz) * inv_z;
			const tmin = Math.max(Math.min(t1x, t2x), Math.min(t1z, t2z));
			const tmax = Math.min(Math.max(t1x, t2x), Math.max(t1z, t2z));
			if (tmax < 0 || tmin > tmax) continue;   // miss

			// First positive hit; if origin is inside the wall (tmin<0), that's
			// degenerate — treat as zero distance.
			const t = tmin >= 0 ? tmin : 0;
			if (t < minDist) minDist = t;
		}
		return minDist;
	}

	// Line-of-sight test by sub-sampling the segment. Used for path
	// smoothing — if a→b is unblocked, we can skip intermediate waypoints.
	hasLineOfSight(a, b, pad = 0.2) {
		const dist  = Math.hypot(b.x - a.x, b.z - a.z);
		const steps = Math.max(2, Math.ceil(dist / 0.05));
		for (let i = 1; i < steps; i++) {
			const t = i / steps;
			if (this.isBlocked(a.x + (b.x - a.x) * t, a.z + (b.z - a.z) * t, pad)) return false;
		}
		return true;
	}

	// --- Arc helper ---------------------------------------------------------
	// Approximates a circular arc with N straight wall segments. Each segment
	// is a chord between adjacent samples on the arc, oriented along the
	// chord (so the wall is tangent-ish to the circle). Used by loadTrack().
	_addArc({ cx, cz, radius, theta0, theta1, segments, thickness = 0.15, color = 0xf2e6cf }) {
		for (let i = 0; i < segments; i++) {
			const t0 = theta0 + (theta1 - theta0) * (i)     / segments;
			const t1 = theta0 + (theta1 - theta0) * (i + 1) / segments;
			const x0 = cx + radius * Math.cos(t0);
			const z0 = cz + radius * Math.sin(t0);
			const x1 = cx + radius * Math.cos(t1);
			const z1 = cz + radius * Math.sin(t1);
			const mx = (x0 + x1) / 2;
			const mz = (z0 + z1) / 2;
			const dx = x1 - x0;
			const dz = z1 - z0;
			// Tiny chord-length overshoot so adjacent segments overlap and
			// don't leave a hairline gap that lidar rays can sneak through.
			const length = Math.hypot(dx, dz) * 1.05;
			// three.js rotation.y rotates local +x to world (cos y, -sin y),
			// so to align the wall's long axis with the chord (dx, dz) we
			// need yaw = atan2(-dz, dx), not atan2(dz, dx).
			const yaw = Math.atan2(-dz, dx);
			this.addWall({ x: mx, z: mz, length, thickness, yaw, color });
		}
	}

	// --- Race track ---------------------------------------------------------
	// Stadium-oval corridor with a section of pillars on the far straight.
	// Built for the reactive lidar pilot: the bot follows the corridor walls
	// like a road, then has to weave through the pillar field on the back
	// straight. Bot spawns at (0, 0) heading +x, driving CCW.
	//
	// Geometry (top-down):
	//
	//                ╭───────────────────────╮      ← outer top    z = +6.75
	//                │  ▪    ▪    ▪    ▪    │      ← pillar field
	//                │ ╭───────────────────╮ │      ← inner top    z = +4.25
	//                │ │                   │ │
	//                │ │      (inner)      │ │      corridor width 2.5 m
	//                │ │                   │ │
	//                │ ╰───────────────────╯ │      ← inner bottom z = +1.25
	//          start →                       ←      ← bot spawn at (0,0)
	//                ╰───────────────────────╯      ← outer bottom z = -1.25
	//                ↑                       ↑
	//              x = -10                 x = +10
	//
	// Curves are approximated with chord segments via _addArc. Outer radius
	// 4 m, inner radius 1.5 m → 2.5 m corridor on the curves too.
	loadTrack() {
		this.clear();
		const wt = 0.15;     // wall thickness

		// Straights — outer.
		this.addWall({ x: 0, z: -1.25, length: 12.0, thickness: wt, yaw: 0 });
		this.addWall({ x: 0, z:  6.75, length: 12.0, thickness: wt, yaw: 0 });
		// Straights — inner.
		this.addWall({ x: 0, z:  1.25, length: 10.0, thickness: wt, yaw: 0 });
		this.addWall({ x: 0, z:  4.25, length: 10.0, thickness: wt, yaw: 0 });

		// Right end-cap. Outer arc center (6, 2.75) r=4; inner arc center (5, 2.75) r=1.5.
		this._addArc({ cx: 6, cz: 2.75, radius: 4,   theta0: -Math.PI / 2, theta1:  Math.PI / 2, segments: 14 });
		this._addArc({ cx: 5, cz: 2.75, radius: 1.5, theta0: -Math.PI / 2, theta1:  Math.PI / 2, segments:  8 });

		// Left end-cap. Mirror of the right.
		this._addArc({ cx: -6, cz: 2.75, radius: 4,   theta0:  Math.PI / 2, theta1:  3 * Math.PI / 2, segments: 14 });
		this._addArc({ cx: -5, cz: 2.75, radius: 1.5, theta0:  Math.PI / 2, theta1:  3 * Math.PI / 2, segments:  8 });

		// Pillar field — four short fat walls in the top straight, alternating
		// sides of the corridor centerline (z=5.5) so the bot has to weave.
		// Small enough (0.4 × 0.4) that 2.5 m corridor still has plenty of
		// passage on either side.
		const pillar = (x, z) => this.addWall({
			x, z, length: 0.4, thickness: 0.4, yaw: 0, color: 0xcc3322,
		});
		pillar(-3, 4.9);
		pillar(-1, 6.1);
		pillar( 1, 4.9);
		pillar( 3, 6.1);

		// Start/finish flags at the bot's spawn.
		this.addFlag({ x: 0, z: -0.6, color: 0x44ff66 });
		this.addFlag({ x: 0, z:  0.6, color: 0x44ff66 });
	}

	// --- A demo course ------------------------------------------------------
	// Serpentine maze. Bot starts at (0, 0); finish at the far end. Four
	// vertical walls alternately block the top and bottom half-corridor,
	// forcing a snake path that direct nav can't navigate but A* solves
	// trivially. Boundary walls top and bottom prevent going around.
	//
	// Wall layout (top-down view, x→ horizontal, z↑ vertical):
	//
	//     z=+2.5  ─────────────────────────────────────────────────  (boundary)
	//                   │              │              │
	//     z=+0.3        │     ┌────────┴──┐    ┌──────┴──┐
	//     z= 0     S→   │     │           │    │         │   ←G
	//     z=-0.3   ┌────┴──┐  │           │    │         │
	//              │       │  │           │    │         │
	//     z=-2.5  ─┴───────┴──┴───────────┴────┴─────────┴───────  (boundary)
	//             x=0  ½  3   ½    5.5   ½   8    ½   10.5    12
	//
	// The four maze walls have extents (after yaw=π/2 rotation):
	//     block-bottom (z=-0.85, length 2.3) → z = -2.0 to +0.3
	//     block-top    (z=+0.85, length 2.3) → z = -0.3 to +2.0
	// Bot must alternate above-then-below to pass each one.

	loadDemoCourse() {
		this.clear();

		const wall_t = 0.15;

		// Boundary walls — bot can't bypass the maze.
		this.addWall({ x: 6, z:  2.5, length: 13.0, thickness: wall_t, yaw: 0 });
		this.addWall({ x: 6, z: -2.5, length: 13.0, thickness: wall_t, yaw: 0 });

		// Serpentine maze walls.
		this.addWall({ x:  3.0, z: -0.85, length: 2.3, thickness: wall_t, yaw: Math.PI / 2 });
		this.addWall({ x:  5.5, z:  0.85, length: 2.3, thickness: wall_t, yaw: Math.PI / 2 });
		this.addWall({ x:  8.0, z: -0.85, length: 2.3, thickness: wall_t, yaw: Math.PI / 2 });
		this.addWall({ x: 10.5, z:  0.85, length: 2.3, thickness: wall_t, yaw: Math.PI / 2 });

		// Start markers.
		this.addFlag({ x: 0.6, z:  0.6, color: 0x44ff66 });
		this.addFlag({ x: 0.6, z: -0.6, color: 0x44ff66 });

		// Decorative hoop just before the finish (pass-through).
		this.addRing({ x: 11.7, z: 0, radius: 0.85, color: 0xcc3322 });

		// Finish line.
		this.addFlag({ x: 12.5, z:  0.7, color: 0xff4466 });
		this.addFlag({ x: 12.5, z: -0.7, color: 0xff4466 });
	}
}
