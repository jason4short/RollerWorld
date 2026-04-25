import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { Obstacles }     from '../world/obstacles.js';

// 3D bot view using three.js. Same draw() interface as the 2D WorldRenderer:
//   draw(state, params, navTargetX)
// Plus a worldXFromClick(event) helper for placing nav targets by clicking
// the canvas (raycasts onto the ground plane).

export class WorldRenderer3D {
	constructor(canvas) {
		this.canvas = canvas;

		this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
		this.renderer.setPixelRatio(window.devicePixelRatio);
		this.renderer.setSize(canvas.width, canvas.height, false);
		this.renderer.setClearColor(0xee8855, 1);    // warm orange sky
		this.renderer.shadowMap.enabled = true;
		this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;

		this.scene = new THREE.Scene();
		this.scene.background = new THREE.Color(0xee8855);
		// No fog — Bruno-Simon look has crisp depth without atmospheric falloff.

		const aspect = canvas.width / canvas.height;
		this.camera = new THREE.PerspectiveCamera(35, aspect, 0.05, 200);
		this.camera.position.set(-5, 5, 7);          // higher isometric-ish angle

		// Lighting — bright warm ambient + a single sun with soft shadows.
		this.scene.add(new THREE.AmbientLight(0xffeedd, 0.9));
		const sun = new THREE.DirectionalLight(0xfff8ee, 1.0);
		sun.position.set(6, 12, 4);
		sun.castShadow = true;
		sun.shadow.mapSize.set(2048, 2048);
		sun.shadow.camera.near = 1;
		sun.shadow.camera.far  = 40;
		sun.shadow.camera.left = -15; sun.shadow.camera.right = 15;
		sun.shadow.camera.top  =  15; sun.shadow.camera.bottom = -15;
		sun.shadow.bias = -0.0005;
		this.scene.add(sun);
		this.sun = sun;

		// Ground — solid warm orange, no grid.
		const groundMat = new THREE.MeshStandardMaterial({
			color: 0xdd6633, roughness: 1.0, metalness: 0,
		});
		this.ground = new THREE.Mesh(new THREE.PlaneGeometry(400, 400), groundMat);
		this.ground.rotation.x = -Math.PI / 2;
		this.ground.receiveShadow = true;
		this.scene.add(this.ground);

		// Scattered cream "path" tiles around the starting area — visual
		// breadcrumbs in the spirit of the Bruno-Simon site.
		this._buildPathTiles();

		// Orbit camera — drag to rotate, wheel to zoom. Target is updated each
		// frame to follow the bot's position, so the user's view orientation
		// is preserved while the bot drives.
		this.controls = new OrbitControls(this.camera, canvas);
		this.controls.enableDamping = true;
		this.controls.dampingFactor = 0.1;
		this.controls.minDistance = 1.0;
		this.controls.maxDistance = 40;
		this.controls.maxPolarAngle = Math.PI * 0.49;   // don't go below ground
		this.controls.target.set(0, 0.4, 0);

		// Bot — group containing wheels and a body subgroup that pivots for pitch.
		this.bot = new THREE.Group();
		this.scene.add(this.bot);
		this._buildBot();

		// Nav target flag (current/head — bobbing + spinning).
		this.target = this._buildTargetFlag();
		this.target.visible = false;
		this.scene.add(this.target);

		// Pool of static flags for upcoming waypoints in the queue. Built
		// lazily on demand and reused; meshes past the queue length are
		// hidden rather than destroyed.
		this.queueFlags = [];

		// Obstacle course
		this.obstacles = new Obstacles();
		this.obstacles.loadDemoCourse();
		this.scene.add(this.obstacles.group);
	}

	// Convert a canvas-relative click to world-frame ground (y=0) coords.
	// Returns { x, z } or null if the click misses the ground.
	screenToGround(clientX, clientY) {
		const rect = this.canvas.getBoundingClientRect();
		const ndc = new THREE.Vector2(
			((clientX - rect.left) / rect.width)  *  2 - 1,
			((clientY - rect.top)  / rect.height) * -2 + 1,
		);
		const ray = new THREE.Raycaster();
		ray.setFromCamera(ndc, this.camera);
		const plane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
		const hit = new THREE.Vector3();
		if (!ray.ray.intersectPlane(plane, hit)) return null;
		return { x: hit.x, z: hit.z };
	}

	_buildBot() {
		// Blocky tires — dark, low-poly cylinders.
		const wheelMat = new THREE.MeshStandardMaterial({ color: 0x2a1f17, roughness: 1 });
		const wheelGeom = new THREE.CylinderGeometry(1, 1, 0.08, 16);
		wheelGeom.rotateX(Math.PI / 2);

		this.wheelL = new THREE.Mesh(wheelGeom, wheelMat);
		this.wheelR = new THREE.Mesh(wheelGeom.clone(), wheelMat.clone());
		this.wheelL.castShadow = true;
		this.wheelR.castShadow = true;
		this.bot.add(this.wheelL, this.wheelR);

		// Cream hub on each wheel for visible rotation.
		const hubMat = new THREE.MeshStandardMaterial({ color: 0xf2e6cf });
		for (const wheel of [this.wheelL, this.wheelR]) {
			const hub = new THREE.Mesh(
				new THREE.CylinderGeometry(0.35, 0.35, 0.12, 12),
				hubMat,
			);
			hub.rotation.x = Math.PI / 2;
			wheel.add(hub);
			// Single dark spoke so rotation is unambiguous.
			const spoke = new THREE.Mesh(
				new THREE.BoxGeometry(1.2, 0.06, 0.02),
				new THREE.MeshStandardMaterial({ color: 0x1a1a1a }),
			);
			wheel.add(spoke);
		}

		// Body — pivots about Z (pitch). Children positioned for L-dependent geometry.
		this.body = new THREE.Group();
		this.bot.add(this.body);

		// Main chassis — a single red box. Bright accent against the orange world.
		this.stack = new THREE.Mesh(
			new THREE.BoxGeometry(0.36, 0.22, 0.34),
			new THREE.MeshStandardMaterial({ color: 0xcc3322, roughness: 0.6 }),
		);
		this.stack.castShadow = true;
		this.body.add(this.stack);

		// Cab block on top — slightly smaller, cream colored.
		this.deck = new THREE.Mesh(
			new THREE.BoxGeometry(0.22, 0.16, 0.28),
			new THREE.MeshStandardMaterial({ color: 0xf2e6cf, roughness: 0.7 }),
		);
		this.deck.castShadow = true;
		this.body.add(this.deck);

		// Hidden CoM marker (kept for code reuse, but invisible in this style).
		this.com = new THREE.Mesh(
			new THREE.SphereGeometry(0.02, 8, 8),
			new THREE.MeshBasicMaterial({ visible: false }),
		);
		this.body.add(this.com);

		// Empty rails array — kept so draw() doesn't blow up; no visible rails.
		this.rails = [];
	}

	_buildPathTiles() {
		// Cream tiles scattered along the path — purely decorative, evokes the
		// Bruno-Simon "stepping-stones" feel.
		const mat = new THREE.MeshStandardMaterial({ color: 0xf2e6cf, roughness: 1 });
		const positions = [
			[ 1.0,  0.0], [ 1.4,  0.3], [ 1.8, -0.2],
			[ 2.4,  0.1], [ 3.0, -0.3], [ 3.6,  0.2],
			[ 4.5,  0.0], [ 5.5, -0.1], [ 6.5,  0.2],
			[ 7.5, -0.3], [ 8.5,  0.4], [ 9.5, -0.2],
			[10.5,  0.0], [11.5,  0.3], [12.5, -0.2],
		];
		for (const [x, z] of positions) {
			const w = 0.45 + Math.random() * 0.15;
			const d = 0.45 + Math.random() * 0.15;
			const tile = new THREE.Mesh(new THREE.BoxGeometry(w, 0.04, d), mat);
			tile.position.set(x, 0.02, z);
			tile.rotation.y = (Math.random() - 0.5) * 0.3;
			tile.receiveShadow = true;
			this.scene.add(tile);
		}
	}

	_buildTargetFlag() {
		const g = new THREE.Group();
		const pole = new THREE.Mesh(
			new THREE.CylinderGeometry(0.04, 0.04, 1.2, 8),
			new THREE.MeshStandardMaterial({ color: 0xf2e6cf }),
		);
		pole.position.y = 0.6;
		pole.castShadow = true;
		g.add(pole);
		const flag = new THREE.Mesh(
			new THREE.BoxGeometry(0.44, 0.28, 0.04),
			new THREE.MeshStandardMaterial({ color: 0x44ff66, emissive: 0x114422 }),
		);
		flag.position.set(0.22, 1.04, 0);
		flag.castShadow = true;
		g.add(flag);
		return g;
	}

	_buildQueueFlag() {
		// Same shape as the nav target flag but a calmer color and no
		// emissive — these are the "next up" flags, not the active one.
		const g = new THREE.Group();
		const pole = new THREE.Mesh(
			new THREE.CylinderGeometry(0.04, 0.04, 1.2, 8),
			new THREE.MeshStandardMaterial({ color: 0xf2e6cf }),
		);
		pole.position.y = 0.6;
		pole.castShadow = true;
		g.add(pole);
		const flag = new THREE.Mesh(
			new THREE.BoxGeometry(0.44, 0.28, 0.04),
			new THREE.MeshStandardMaterial({ color: 0xffcc44 }),
		);
		flag.position.set(0.22, 1.04, 0);
		flag.castShadow = true;
		g.add(flag);
		return g;
	}

	draw(state, params, navTarget = null, queueRest = []) {
		const { L, R } = params;

		// Wheels — scale for current radius, position at axle height.
		this.wheelL.scale.set(R, R, 1);
		this.wheelR.scale.set(R, R, 1);
		this.wheelL.position.set(0, R, -0.16);
		this.wheelR.position.set(0, R, 0.16);

		// Wheel rotation about its own (now-Z) spin axis.
		// Forward (+x) motion → CW from +Z view → negative rotation about Z.
		this.wheelL.rotation.z = -state.x / R;
		this.wheelR.rotation.z = -state.x / R;

		// Body pivots at axle height. Update L-dependent positions/scales.
		this.body.position.set(0, R, 0);
		// +θ in our convention = bob toward +x. In three.js, that's a negative
		// rotation about Z (positive Z-rot would take bob toward -x).
		this.body.rotation.z = -state.th;

		// Stack/cab positions scale loosely with L so taller bots look taller.
		// Chassis at half-L, cab on top of it, no rails in the blocky style.
		this.stack.position.y = L * 0.55;
		this.deck.position.y  = L * 0.55 + 0.18;
		this.com.position.y   = L;

		// Bot world position + heading
		this.bot.position.set(state.x, 0, state.z);
		this.bot.rotation.y = state.psi;

		// Nav target flag (2D position on the ground). Bob + spin so it
		// reads as "go here" not just another course marker.
		if (navTarget !== null) {
			this.target.visible = true;
			const t = performance.now() * 0.001;
			this.target.position.set(navTarget.x, 0.05 + 0.05 * Math.sin(t * 3), navTarget.z ?? 0);
			this.target.rotation.y = t * 0.8;
		} else {
			this.target.visible = false;
		}

		// Upcoming waypoints — grow the pool as needed, hide the unused ones.
		while (this.queueFlags.length < queueRest.length) {
			const f = this._buildQueueFlag();
			this.scene.add(f);
			this.queueFlags.push(f);
		}
		for (let i = 0; i < this.queueFlags.length; i++) {
			const f = this.queueFlags[i];
			if (i < queueRest.length) {
				f.visible = true;
				f.position.set(queueRest[i].x, 0, queueRest[i].z);
			} else {
				f.visible = false;
			}
		}

		// Camera follows the bot. We move the OrbitControls target to the
		// bot's position each frame, and shift the camera by the same delta
		// so the user's chosen viewing angle/distance is preserved while
		// the bot drives.
		// Focus a bit higher above the ground for the wider zoom-out view.
		const focusY = Math.max(0.6, L * 0.8);
		const newTarget = new THREE.Vector3(state.x, focusY, state.z);
		const delta = newTarget.clone().sub(this.controls.target);
		this.camera.position.add(delta);
		this.controls.target.copy(newTarget);
		this.controls.update();

		this.renderer.render(this.scene, this.camera);
	}

	// Convert a canvas click to a world (x, z) point on the ground plane.
	worldPointFromClick(e) {
		const rect = this.canvas.getBoundingClientRect();
		const ndc = new THREE.Vector2(
			((e.clientX - rect.left) / rect.width)  * 2 - 1,
			-((e.clientY - rect.top) / rect.height) * 2 + 1,
		);
		const raycaster = new THREE.Raycaster();
		raycaster.setFromCamera(ndc, this.camera);
		const hits = raycaster.intersectObject(this.ground);
		if (!hits.length) return null;
		return { x: hits[0].point.x, z: hits[0].point.z };
	}
}
