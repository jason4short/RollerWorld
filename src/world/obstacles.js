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
