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

	// --- A demo course ------------------------------------------------------
	loadDemoCourse() {
		this.clear();
		// Kickoff flags either side of starting line.
		this.addFlag({ x:  0.6, z:  0.6, color: 0x44ff66 });
		this.addFlag({ x:  0.6, z: -0.6, color: 0x44ff66 });

		// A wall to drive around.
		this.addWall({ x: 2.5, z:  0.7, length: 1.2 });

		// Ring gate to drive through, slightly off center.
		this.addRing({ x: 4.0, z: -0.3, radius: 0.9 });

		// Mid-course tunnel.
		this.addTunnel({ x: 6.0, z:  0.0, length: 1.6, radius: 1.0 });

		// Slalom flags.
		this.addFlag({ x:  8.0, z:  0.5, color: 0xffcc44 });
		this.addFlag({ x:  9.0, z: -0.5, color: 0xffcc44 });
		this.addFlag({ x: 10.0, z:  0.5, color: 0xffcc44 });

		// A second wall.
		this.addWall({ x: 11.5, z: 0, length: 0.8, yaw: Math.PI / 4 });

		// Finish gate — two flags + a ring.
		this.addFlag({ x: 13.0, z:  0.7, color: 0xff4466 });
		this.addFlag({ x: 13.0, z: -0.7, color: 0xff4466 });
		this.addRing({ x: 13.0, z:  0.0, radius: 1.0, color: 0xff4466 });
	}
}
