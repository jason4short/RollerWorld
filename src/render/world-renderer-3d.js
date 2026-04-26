import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { Obstacles }     from '../world/obstacles.js';
import { RoadNetwork }   from '../world/road-network.js';
import { RoadCanvas }    from '../world/road-canvas.js';
import { heightAt }      from '../world/terrain.js';

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
		// Atmospheric fog — distance hazes into the warm sky color, giving
		// the park a sense of scale and softening the far horizon. Tuned
		// so the bot's immediate surroundings (~30 m) are crisp and the
		// far edges of the park dissolve into the sky.
		this.scene.fog = new THREE.Fog(0xee8855, 40, 180);

		const aspect = canvas.width / canvas.height;
		// Far plane bumped to 400 — the park is ~110 m end-to-end, the
		// orbit camera should be able to pull back enough to see all of it.
		this.camera = new THREE.PerspectiveCamera(35, aspect, 0.05, 400);
		this.camera.position.set(-12, 8, 14);          // higher isometric-ish angle

		// Lighting — bright warm ambient + a single sun with soft shadows.
		this.scene.add(new THREE.AmbientLight(0xffeedd, 0.9));
		const sun = new THREE.DirectionalLight(0xfff8ee, 1.0);
		// Sun positioned high and to the side; shadow camera bounds expanded
		// so the whole park (~110 m × 70 m playable) receives shadows.
		sun.position.set(40, 80, 30);
		sun.castShadow = true;
		sun.shadow.mapSize.set(2048, 2048);
		sun.shadow.camera.near = 1;
		sun.shadow.camera.far  = 250;
		sun.shadow.camera.left = -80; sun.shadow.camera.right = 80;
		sun.shadow.camera.top  =  80; sun.shadow.camera.bottom = -80;
		sun.shadow.bias = -0.0005;
		this.scene.add(sun);
		this.sun = sun;

		// Ground — heightfield mesh built from the procedural `heightAt`
		// function. PlaneGeometry is subdivided into a 200×200 grid of
		// vertices over a 250 m × 250 m patch; each vertex gets its z
		// (which becomes y after the rotation) displaced by heightAt(x, z).
		// Same heightAt drives the road texture coords, the bot's render
		// position, and the slope-force disturbance — single source of
		// truth for anything height-related.
		const GROUND_SIZE = 250;
		const GROUND_SEGS = 200;
		const groundGeom = new THREE.PlaneGeometry(GROUND_SIZE, GROUND_SIZE, GROUND_SEGS, GROUND_SEGS);
		const gp = groundGeom.attributes.position;
		for (let i = 0; i < gp.count; i++) {
			const x = gp.getX(i);
			const y = gp.getY(i);
			gp.setZ(i, heightAt(x, -y));
		}
		groundGeom.computeVertexNormals();
		// Ground texture is the road-network canvas (built below). The
		// painted asphalt-on-grass IS the road — no separate road meshes.
		// The same canvas is the data source for the color sensor: bot
		// drives on what its sensor sees.
		this.GROUND_SIZE = GROUND_SIZE;   // expose for canvas mapping
		const groundMat = new THREE.MeshStandardMaterial({
			roughness: 1.0, metalness: 0,
			flatShading: true,    // chunky low-poly facets, not smoothed
		});
		this.ground = new THREE.Mesh(groundGeom, groundMat);
		this.ground.rotation.x = -Math.PI / 2;
		this.ground.receiveShadow = true;
		this.scene.add(this.ground);

		// Water — a single flat plane at "sea level" that the terrain
		// pokes up through. Where the heightfield dips below this y, the
		// water is what we see; where it's above, the ground wins via the
		// depth buffer. Sea level is chosen just below the deepest road
		// segments so most of the network stays dry, while the very
		// bottoms of valleys form ponds and streams.
		this.SEA_LEVEL = -3.5;
		const waterMat = new THREE.MeshStandardMaterial({
			color: 0x3a8eaa, roughness: 0.25, metalness: 0.1,
			transparent: true, opacity: 0.92,
		});
		this.water = new THREE.Mesh(new THREE.PlaneGeometry(GROUND_SIZE, GROUND_SIZE), waterMat);
		this.water.rotation.x = -Math.PI / 2;
		this.water.position.y = this.SEA_LEVEL;
		this.scene.add(this.water);

		// (Stepping-stones removed — the race-track world has its own walls
		// as visual landmarks; tiles ended up overlapping them awkwardly.)

		// Orbit camera — drag to rotate, wheel to zoom. Target is updated each
		// frame to follow the bot's position, so the user's view orientation
		// is preserved while the bot drives.
		this.controls = new OrbitControls(this.camera, canvas);
		this.controls.enableDamping = true;
		this.controls.dampingFactor = 0.1;
		this.controls.minDistance = 1.0;
		this.controls.maxDistance = 200;
		this.controls.maxPolarAngle = Math.PI * 0.49;   // don't go below ground
		this.controls.target.set(0, 0.4, 0);

		// Auto-recenter: after a beat of no interaction, smoothly slew the
		// camera around to a third-person behind-the-bot pose. Distance and
		// height are captured from the user's last manual pose so spinning
		// in close gives a close auto-pose, spinning out gives a wider one.
		this.autoFollowEnabled   = true;     // app toggles via setAutoFollow()
		this.userInteracting     = false;
		this.lastInteractionTime = 0;        // 0 ⇒ auto-follow on first load
		this.followDistance      = null;     // populated on first interaction-end
		this.followHeight        = null;
		this.controls.addEventListener('start', () => { this.userInteracting = true; });
		this.controls.addEventListener('end',   () => {
			this.userInteracting = false;
			this.lastInteractionTime = performance.now();
			const off = this.camera.position.clone().sub(this.controls.target);
			this.followDistance = Math.hypot(off.x, off.z);
			this.followHeight   = off.y;
		});

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

		// Planner path polyline — set per-frame from the latest planner output.
		this.pathLine = new THREE.Line(
			new THREE.BufferGeometry(),
			new THREE.LineBasicMaterial({ color: 0x66ddff, transparent: true, opacity: 0.85 }),
		);
		this.pathLine.frustumCulled = false;
		this.pathLine.visible = false;
		this.scene.add(this.pathLine);

		// Bot trail — a fading polyline of where the bot has actually driven.
		// Useful pedagogy: overlay actual trajectory against the planned path
		// (the cyan pathLine above) to see how well the bot tracked.
		// Vertex colors fade from chassis red at the head to ground orange at
		// the tail, which reads like a tire-track scuff against the ground.
		this.trailMax       = 300;        // max samples kept
		this.trailMinStep   = 0.04;       // meters; skip samples closer than this
		this.trailPoints    = [];         // ring of {x, z}
		this.trailLine = new THREE.Line(
			new THREE.BufferGeometry(),
			new THREE.LineBasicMaterial({ vertexColors: true }),
		);
		this.trailLine.frustumCulled = false;
		this.trailLine.visible = false;
		this.scene.add(this.trailLine);

		// Occupancy-grid overlay — a CanvasTexture mapped onto a horizontal
		// plane just above the ground. Sized to match the grid's world extent
		// at construction time via setOccupancyGrid().
		this.gridCanvas  = document.createElement('canvas');
		this.gridTexture = new THREE.CanvasTexture(this.gridCanvas);
		this.gridTexture.magFilter = THREE.NearestFilter;
		this.gridTexture.minFilter = THREE.NearestFilter;
		this.gridMesh = null;   // built on first setOccupancyGrid call

		// Lidar rays — N short line segments from the bot to each scan hit.
		// Vertex colors so the Safety governor can highlight clipping rays.
		this.lidarLines = new THREE.LineSegments(
			new THREE.BufferGeometry(),
			new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.7 }),
		);
		this.lidarLines.frustumCulled = false;
		this.lidarLines.visible = false;
		this.scene.add(this.lidarLines);

		// Obstacles — currently empty in the park world. The barrier-wall
		// hook for blocked road edges will populate it. The Track / DemoCourse
		// builders are still available on Obstacles if a lesson needs them.
		this.obstacles = new Obstacles();
		this.scene.add(this.obstacles.group);

		// Park road network — eleven nodes, fourteen curved edges, three
		// forks, two dead-end spurs. The renderer builds road meshes from
		// the centerline samples; later phases will paint a ground texture
		// from the same data and feed a color sensor on the bot.
		this.roadNetwork = new RoadNetwork().loadPark();

		// RoadCanvas paints the network into a 2D canvas. Used both as
		// the ground texture (visible asphalt-on-grass) and as the data
		// source for the bot's color sensor — single source of truth.
		this.roadCanvas = new RoadCanvas(this.roadNetwork, {
			worldSize: GROUND_SIZE, pxPerMeter: 8,
		});
		const roadTexture = new THREE.CanvasTexture(this.roadCanvas.canvas);
		roadTexture.flipY     = false;
		roadTexture.colorSpace = THREE.SRGBColorSpace;
		roadTexture.minFilter = THREE.LinearMipmapLinearFilter;
		roadTexture.magFilter = THREE.LinearFilter;
		roadTexture.anisotropy = this.renderer.capabilities.getMaxAnisotropy();
		groundMat.map = roadTexture;
		groundMat.needsUpdate = true;

		// Tree clusters scattered across the park, avoiding the roads.
		this._buildTrees();

		// Boulders + cream stepping-stones — visual rhythm props.
		this._buildProps();
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
		// Hit the actual heightfield mesh so a shift-click on a hilltop
		// resolves to the (x, z) under the cursor, not the (x, z) where
		// the ray would have crossed y=0 several meters past the hill.
		const hits = ray.intersectObject(this.ground);
		if (hits.length) return { x: hits[0].point.x, z: hits[0].point.z };
		// Fallback to the flat plane for clicks that miss the mesh
		// (e.g., clicks on the sky outside the ground extent).
		const plane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
		const hit = new THREE.Vector3();
		if (!ray.ray.intersectPlane(plane, hit)) return null;
		return { x: hit.x, z: hit.z };
	}


	// Tree clusters scattered across the park. Each tree is a low-poly
	// cone with a small trunk, randomly drawn from a 5-color autumn
	// palette. Avoids the road network — any candidate position closer
	// than `roadAvoid` to a centerline sample is rejected. Uses a fixed
	// PRNG seed so the layout is deterministic between page reloads (no
	// jarring re-shuffles when the user resets).
	_buildTrees() {
		this.treeGroup = new THREE.Group();
		this.scene.add(this.treeGroup);

		const palette = [
			0x3e6b32,   // deep forest green
			0x6b8e3f,   // grass green
			0x4a7a5e,   // dusty teal
			0xc4623a,   // burnt orange
			0xa84436,   // dusty red
		];
		const TRUNK_COLOR = 0x4a3326;

		// Pre-build a flat list of all road centerline samples for the
		// distance check. Sample at coarse density — 32 samples per edge
		// is plenty for tree-avoidance purposes.
		const roadSamples = [];
		for (let ei = 0; ei < this.roadNetwork.edges.length; ei++) {
			const samples = this.roadNetwork.sampleEdge(ei, 32);
			for (const s of samples) roadSamples.push(s);
		}
		const roadAvoid = this.roadNetwork.width / 2 + 1.4;   // m
		const roadAvoidSq = roadAvoid * roadAvoid;

		// Deterministic 32-bit linear-congruential PRNG (Numerical Recipes
		// constants). A fixed seed gives the same forest every reload.
		let seed = 0x13579bdf;
		const rand = () => {
			seed = (seed * 1664525 + 1013904223) >>> 0;
			return seed / 0x100000000;
		};

		// Park playable footprint plus margin so trees fade out around
		// the edges instead of stopping at a hard boundary.
		const X_MIN = -55, X_MAX =  70;
		const Z_MIN = -35, Z_MAX =  45;

		const TARGET_TREES = 220;
		let placed = 0, attempts = 0;
		while (placed < TARGET_TREES && attempts < TARGET_TREES * 8) {
			attempts++;
			const x = X_MIN + rand() * (X_MAX - X_MIN);
			const z = Z_MIN + rand() * (Z_MAX - Z_MIN);

			// Reject if too close to any road sample.
			let tooClose = false;
			for (const s of roadSamples) {
				const dx = x - s.x, dz = z - s.z;
				if (dx * dx + dz * dz < roadAvoidSq) { tooClose = true; break; }
			}
			if (tooClose) continue;

			// Reject if underwater — trees would float on the surface
			// otherwise. Skip a small margin above sea level too so we
			// don't get trees with their feet in the water.
			const y = heightAt(x, z);
			if (y < this.SEA_LEVEL + 0.4) continue;

			// Vary tree size a little so the forest has rhythm. Slightly
			// taller, thinner trees on hills (where height > 0) for a hint
			// of pine-on-mountain feel.
			const baseHeight = 1.6 + rand() * 1.8 + Math.max(0, y) * 0.3;
			const baseRadius = 0.6 + rand() * 0.5;
			const color = palette[(rand() * palette.length) | 0];

			const tree = new THREE.Group();
			const trunk = new THREE.Mesh(
				new THREE.CylinderGeometry(0.12, 0.16, 0.5, 6),
				new THREE.MeshStandardMaterial({ color: TRUNK_COLOR, roughness: 1, flatShading: true }),
			);
			trunk.position.y = 0.25;
			trunk.castShadow = true;
			tree.add(trunk);

			const foliage = new THREE.Mesh(
				new THREE.ConeGeometry(baseRadius, baseHeight, 7),
				new THREE.MeshStandardMaterial({ color, roughness: 1, flatShading: true }),
			);
			foliage.position.y = 0.5 + baseHeight / 2 - 0.1;
			foliage.rotation.y = rand() * Math.PI * 2;
			foliage.castShadow = true;
			tree.add(foliage);

			tree.position.set(x, y, z);
			this.treeGroup.add(tree);
			placed++;
		}
	}

	// Public hook for re-painting the road after the network changes
	// (e.g. a caller flipped an edge's `blocked` flag). Repaints the
	// canvas and flags the texture for re-upload — the sensor's pixel
	// buffer also updates because RoadCanvas.draw() refreshes it.
	rebuildRoad() {
		this.roadCanvas.draw();
		if (this.ground.material.map) this.ground.material.map.needsUpdate = true;
	}

	_buildBot() {
		// Wheels: solid black, oversized (visual scale ~1.5× the physics R) to
		// give the kawaii proportions in the design — wheels about as tall as
		// the body, with a single cream spoke for visible rotation.
		this.WHEEL_VISUAL_SCALE = 1.5;
		const wheelMat 		= new THREE.MeshStandardMaterial({ color: 0x000000, roughness: 0.9 });
		const wheelGeom 	= new THREE.CylinderGeometry(1, 1, 0.05, 24);
		wheelGeom.rotateX(Math.PI / 2);

		this.wheelL 			= new THREE.Mesh(wheelGeom, wheelMat);
		this.wheelR 			= new THREE.Mesh(wheelGeom.clone(), wheelMat.clone());
		this.wheelL.castShadow 	= true;
		this.wheelR.castShadow 	= true;
		this.bot.add(this.wheelL, this.wheelR);

		// Cream pill spoke on each wheel — pinned to the OUTSIDE face so the
		// pills face away from the body (left wheel's pill on -Z side, right
		// wheel's pill on +Z side). Wheel cylinder thickness is 0.05, so the
		// outer face sits at local z = ±0.025; nudge a hair past to avoid
		// z-fighting with the wheel face.
		const spokeGeom	 = new THREE.BoxGeometry(0.18, 0.6, 0.01);
		const spokeMat	 = new THREE.MeshStandardMaterial({ color: 0xf2e6cf });
		const outer_z    = 0.026;

		const spokeL = new THREE.Mesh(spokeGeom, spokeMat);
		spokeL.position.set(0, 0.5, -outer_z);
		this.wheelL.add(spokeL);

		const spokeR = new THREE.Mesh(spokeGeom, spokeMat);
		spokeR.position.set(0, 0.5,  outer_z);
		this.wheelR.add(spokeR);

		// Body group — pivots about Z (pitch).
		this.body = new THREE.Group();
		this.bot.add(this.body);

		// Maroon chassis — thin front-to-back (X), wide between the wheels (Z),
		// taller than either. Matches the kawaii proportions in the design.
		this.stack = new THREE.Mesh(
			new THREE.BoxGeometry(0.16, 0.34, 0.36),
			new THREE.MeshStandardMaterial({ color: 0xCC3322, roughness: 0.8 }),
		);
		this.stack.castShadow = true;
		this.body.add(this.stack);

		// Darker base strip across the bottom of the chassis (front-view detail).
		this.skirt = new THREE.Mesh(
			new THREE.BoxGeometry(0.17, 0.05, 0.365),
			new THREE.MeshStandardMaterial({ color: 0x4a1208, roughness: 0.9 }),
		);
		this.skirt.castShadow = true;
		this.body.add(this.skirt);

		// Cream head cap — slightly wider than the chassis, eyes face +X (forward).
		this.deck = new THREE.Group();
		const headW_x = 0.10;   // depth front-to-back
		const headH_y = 0.10;
		const headW_z = 0.30;   // width between the wheels
		const head = new THREE.Mesh(
			new THREE.BoxGeometry(headW_x, headH_y, headW_z),
			new THREE.MeshStandardMaterial({ color: 0xf2e6cf, roughness: 0.7 }),
		);
		head.castShadow = true;
		this.deck.add(head);

		// Two black dot eyes on the front face (+X).
		const eyeMat  = new THREE.MeshStandardMaterial({ color: 0x000000 });
		const eyeGeom = new THREE.SphereGeometry(0.022, 12, 12);
		const eyeX    = headW_x / 2 + 0.001;   // just outside the front face
		for (const dz of [-0.10, 0.10]) {
			const eye = new THREE.Mesh(eyeGeom, eyeMat);
			eye.position.set(eyeX, 0.005, dz);
			this.deck.add(eye);
		}
		this.body.add(this.deck);

		// Hidden CoM marker (kept for code reuse, invisible in this style).
		this.com = new THREE.Mesh(
			new THREE.SphereGeometry(0.02, 8, 8),
			new THREE.MeshBasicMaterial({ visible: false }),
		);
		this.body.add(this.com);

		this.rails = [];
	}

	// Scattered boulders + cream stepping-stones. Boulders are gray
	// non-uniformly-scaled boxes spread anywhere on land (off-road, above
	// water). Stones are smaller cream blocks placed near road centerlines
	// — they sit as decorative path-side details, like trail markers.
	// Same deterministic PRNG as the trees so the scenery is stable.
	_buildProps() {
		this.propGroup = new THREE.Group();
		this.scene.add(this.propGroup);

		// Distinct PRNG so adding/removing a tree doesn't shuffle the rocks.
		let seed = 0xa5b3c7d9;
		const rand = () => {
			seed = (seed * 1664525 + 1013904223) >>> 0;
			return seed / 0x100000000;
		};

		// Pre-build road sample list (same density as the tree avoidance).
		const roadSamples = [];
		for (let ei = 0; ei < this.roadNetwork.edges.length; ei++) {
			const samples = this.roadNetwork.sampleEdge(ei, 32);
			for (const s of samples) roadSamples.push(s);
		}

		const X_MIN = -55, X_MAX =  70;
		const Z_MIN = -35, Z_MAX =  45;

		// --- Boulders -------------------------------------------------------
		const boulderMat = new THREE.MeshStandardMaterial({
			color: 0x6b6358, roughness: 1.0, flatShading: true,
		});
		const boulderShade = new THREE.MeshStandardMaterial({
			color: 0x8a8275, roughness: 1.0, flatShading: true,    // a lighter highlight color
		});
		const ROAD_AVOID = (this.roadNetwork.width / 2 + 1.0);
		const ROAD_AVOID_SQ = ROAD_AVOID * ROAD_AVOID;

		const TARGET_BOULDERS = 60;
		let placed = 0, attempts = 0;
		while (placed < TARGET_BOULDERS && attempts < TARGET_BOULDERS * 8) {
			attempts++;
			const x = X_MIN + rand() * (X_MAX - X_MIN);
			const z = Z_MIN + rand() * (Z_MAX - Z_MIN);
			const y = heightAt(x, z);
			if (y < this.SEA_LEVEL + 0.2) continue;

			let tooClose = false;
			for (const s of roadSamples) {
				const dx = x - s.x, dz = z - s.z;
				if (dx * dx + dz * dz < ROAD_AVOID_SQ) { tooClose = true; break; }
			}
			if (tooClose) continue;

			// Pair of overlapping boxes for a chunky non-cube silhouette.
			const big = new THREE.Mesh(
				new THREE.BoxGeometry(
					0.8 + rand() * 1.4,
					0.5 + rand() * 0.9,
					0.7 + rand() * 1.2,
				),
				rand() < 0.5 ? boulderMat : boulderShade,
			);
			big.castShadow = true;
			big.receiveShadow = true;
			big.rotation.y = rand() * Math.PI * 2;
			big.position.set(x, y + 0.25, z);
			this.propGroup.add(big);
			placed++;
		}

		// --- Cream stepping-stones along the roads ---------------------------
		// Walk along each edge and drop a small stone every few meters at
		// random offsets to either side of the road. Calmer than full
		// pavement detailing, but adds rhythm.
		const stoneMat = new THREE.MeshStandardMaterial({
			color: 0xf2e6cf, roughness: 1.0, flatShading: true,
		});
		const STONE_SPACING = 6.0;       // m along centerline
		const STONE_OFFSET  = this.roadNetwork.width / 2 + 0.7;   // m perpendicular

		for (let ei = 0; ei < this.roadNetwork.edges.length; ei++) {
			if (this.roadNetwork.edges[ei].blocked) continue;
			const samples = this.roadNetwork.sampleEdge(ei, 80);
			// Walk the centerline accumulating arclength; drop a stone each
			// time we pass a multiple of STONE_SPACING.
			let acc = 0;
			let nextDrop = STONE_SPACING * (0.3 + rand() * 0.7);
			for (let i = 1; i < samples.length; i++) {
				const dx = samples[i].x - samples[i - 1].x;
				const dz = samples[i].z - samples[i - 1].z;
				const seg = Math.hypot(dx, dz);
				acc += seg;
				if (acc >= nextDrop) {
					nextDrop = acc + STONE_SPACING * (0.7 + rand() * 0.6);
					const tx = samples[i].tangent_x, tz = samples[i].tangent_z;
					const nx =  tz, nz = -tx;
					const side = rand() < 0.5 ? +1 : -1;
					const off  = STONE_OFFSET + rand() * 0.4;
					const sx = samples[i].x + nx * off * side;
					const sz = samples[i].z + nz * off * side;
					const sy = heightAt(sx, sz);
					if (sy < this.SEA_LEVEL + 0.1) continue;
					const stone = new THREE.Mesh(
						new THREE.BoxGeometry(
							0.45 + rand() * 0.25,
							0.18 + rand() * 0.10,
							0.45 + rand() * 0.25,
						),
						stoneMat,
					);
					stone.position.set(sx, sy + 0.08, sz);
					stone.rotation.y = rand() * Math.PI * 2;
					stone.receiveShadow = true;
					this.propGroup.add(stone);
				}
			}
		}
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

	// Toggle the auto-recenter behavior. When false, the camera always
	// pans-with-bot and never rotates around to behind it.
	setAutoFollow(enabled) {
		this.autoFollowEnabled = !!enabled;
	}

	// Repaint the occupancy-grid overlay from the grid's current log-odds.
	// Pass null to hide.
	setOccupancyGrid(grid) {
		if (!grid) {
			if (this.gridMesh) this.gridMesh.visible = false;
			return;
		}
		// Build the mesh once, sized to the grid's world extent.
		if (!this.gridMesh) {
			this.gridCanvas.width  = grid.cols;
			this.gridCanvas.height = grid.rows;
			const w = grid.cols * grid.cellSize;
			const h = grid.rows * grid.cellSize;
			this.gridMesh = new THREE.Mesh(
				new THREE.PlaneGeometry(w, h),
				new THREE.MeshBasicMaterial({
					map: this.gridTexture, transparent: true, depthWrite: false,
				}),
			);
			this.gridMesh.rotation.x = -Math.PI / 2;
			this.gridMesh.position.set(grid.originX + w / 2, 0.025, grid.originZ + h / 2);
			this.scene.add(this.gridMesh);
		}
		// Repaint the canvas from grid.lo. Dark = occupied, pale = free,
		// fully transparent = unknown.
		const ctx = this.gridCanvas.getContext('2d');
		const img = ctx.getImageData(0, 0, grid.cols, grid.rows);
		const d = img.data;
		for (let j = 0; j < grid.rows; j++) {
			for (let i = 0; i < grid.cols; i++) {
				const lo = grid.lo[j * grid.cols + i];
				const k = (j * grid.cols + i) * 4;
				if (lo > 0.5) {
					d[k+0] = 30;  d[k+1] = 30;  d[k+2] = 30;  d[k+3] = 235;
				} else if (lo < -0.5) {
					d[k+0] = 230; d[k+1] = 230; d[k+2] = 250; d[k+3] = 70;
				} else {
					d[k+3] = 0;
				}
			}
		}
		ctx.putImageData(img, 0, 0);
		this.gridTexture.needsUpdate = true;
		this.gridMesh.visible = true;
	}

	// rays: array of { hit_x, hit_z } from Lidar.scan(). Drawn as line
	// segments from the bot up to each hit. Bot origin is the line start.
	// `highlight` is an optional same-length boolean array; highlighted
	// rays are drawn red (safety clipping), unhighlighted yellow.
	// Pass null/empty rays to hide.
	setLidar(rays, botPos, highlight = null, hover = 0.18) {
		if (!rays || rays.length === 0) {
			this.lidarLines.visible = false;
			return;
		}
		const N = rays.length;
		const verts  = new Float32Array(N * 6);   // 2 verts × 3 floats per ray
		const colors = new Float32Array(N * 6);   // 2 verts × 3 floats per ray
		// Lift each endpoint to its terrain height + hover so rays run along
		// the ground instead of floating at a fixed altitude.
		const yBot = heightAt(botPos.x, botPos.z) + hover;
		for (let i = 0; i < N; i++) {
			const r = rays[i];
			const j = i * 6;
			verts[j + 0] = botPos.x;
			verts[j + 1] = yBot;
			verts[j + 2] = botPos.z;
			verts[j + 3] = r.hit_x;
			verts[j + 4] = heightAt(r.hit_x, r.hit_z) + hover;
			verts[j + 5] = r.hit_z;
			// Color: yellow normally, red when this ray triggered safety clipping.
			const hot = highlight && highlight[i];
			const cR = hot ? 1.00 : 1.00;
			const cG = hot ? 0.30 : 0.93;
			const cB = hot ? 0.30 : 0.53;
			colors[j + 0] = cR; colors[j + 1] = cG; colors[j + 2] = cB;
			colors[j + 3] = cR; colors[j + 4] = cG; colors[j + 5] = cB;
		}
		this.lidarLines.geometry.setAttribute('position', new THREE.BufferAttribute(verts, 3));
		this.lidarLines.geometry.setAttribute('color',    new THREE.BufferAttribute(colors, 3));
		this.lidarLines.geometry.computeBoundingSphere();
		this.lidarLines.visible = true;
	}

	// Wipe the bot trail — call when the sim is reset so the breadcrumbs
	// don't bridge across a teleport from end-of-run back to origin.
	clearTrail() {
		this.trailPoints.length = 0;
		this.trailLine.visible  = false;
	}

	// Append the bot's current ground position to the trail and rebuild the
	// line geometry. Internal — called from draw().
	_updateTrail(x, z) {
		const pts = this.trailPoints;
		const last = pts.length ? pts[pts.length - 1] : null;
		if (!last || Math.hypot(x - last.x, z - last.z) >= this.trailMinStep) {
			pts.push({ x, z });
			if (pts.length > this.trailMax) pts.shift();
		}
		if (pts.length < 2) {
			this.trailLine.visible = false;
			return;
		}
		// Build positions + per-vertex colors. Head is index N-1 (newest);
		// fade toward the ground color at index 0 so the tail dissolves into
		// the ground rather than ending in a hard line.
		const N = pts.length;
		const verts  = new Float32Array(N * 3);
		const colors = new Float32Array(N * 3);
		// chassis red (head) → ground orange (tail)
		const headR = 0.80, headG = 0.20, headB = 0.13;
		const tailR = 0.87, tailG = 0.40, tailB = 0.20;
		for (let i = 0; i < N; i++) {
			verts[i * 3 + 0] = pts[i].x;
			verts[i * 3 + 1] = 0.03;     // just above ground, below path tiles
			verts[i * 3 + 2] = pts[i].z;
			const t = i / (N - 1);       // 0 at tail, 1 at head
			colors[i * 3 + 0] = tailR + (headR - tailR) * t;
			colors[i * 3 + 1] = tailG + (headG - tailG) * t;
			colors[i * 3 + 2] = tailB + (headB - tailB) * t;
		}
		const geom = this.trailLine.geometry;
		geom.setAttribute('position', new THREE.BufferAttribute(verts, 3));
		geom.setAttribute('color',    new THREE.BufferAttribute(colors, 3));
		geom.computeBoundingSphere();
		this.trailLine.visible = true;
	}

	// path: array of {x, z} — Planner's current path. Drawn as a thin
	// cyan polyline a hair above ground so it's visible against grass.
	setPath(path) {
		if (!path || path.length < 2) {
			this.pathLine.visible = false;
			return;
		}
		// Lift each vertex to the terrain so the polyline rides hills
		// instead of floating at a fixed altitude. Small hover above
		// ground keeps it visible against the road texture.
		const verts = new Float32Array(path.length * 3);
		for (let i = 0; i < path.length; i++) {
			verts[i * 3 + 0] = path[i].x;
			verts[i * 3 + 1] = heightAt(path[i].x, path[i].z) + 0.06;
			verts[i * 3 + 2] = path[i].z;
		}
		this.pathLine.geometry.setAttribute('position', new THREE.BufferAttribute(verts, 3));
		this.pathLine.geometry.computeBoundingSphere();
		this.pathLine.visible = true;
	}

	draw(state, params, navTarget = null, queueRest = []) {
		const { L, R } = params;

		// Wheels — visually larger than the physics R for the kawaii look,
		// but rolling rotation still uses true R so the sim stays consistent.
		const Rv = R * this.WHEEL_VISUAL_SCALE;
		this.wheelL.scale.set(Rv, Rv, 1);
		this.wheelR.scale.set(Rv, Rv, 1);
		this.wheelL.position.set(0, Rv, -0.24);
		this.wheelR.position.set(0, Rv,  0.24);

		// Wheel rotation comes straight from the sim — pendulum integrates
		// per-wheel angle from the differential-drive kinematics, so this is
		// the actual angular position of each wheel, not an estimate.
		this.wheelL.rotation.z = -state.wheel_left_angle;
		this.wheelR.rotation.z = -state.wheel_right_angle;

		// Body pivots at axle height. Update L-dependent positions/scales.
		this.body.position.set(0, Rv, 0);
		// +pitch in our convention = bob toward +x. In three.js, that's a
		// negative rotation about Z (positive Z-rot would take bob toward -x).
		this.body.rotation.z = -state.pitch;

		// Body proportions: chassis sits with its base at axle height; the
		// head cap sits on top of the chassis. Skirt is a thin dark strip at
		// the very bottom of the chassis.
		const chassisH = 0.34;
		const headH    = 0.14;
		const skirtH   = 0.05;
		this.stack.position.y = chassisH / 2;
		this.skirt.position.y = skirtH / 2;
		this.deck.position.y  = chassisH + headH / 2;
		this.com.position.y   = L;

		// Bot world position + heading. Y rides on the terrain — the bot's
		// visual base sits at heightAt(x, z); body still pivots around
		// gravity-vertical, not terrain-normal. (Per the "balance bots
		// don't care about slopes" principle: hills are a force
		// disturbance, not a control problem.)
		this.bot.position.set(state.x, heightAt(state.x, state.z), state.z);
		this.bot.rotation.y = state.heading;

		// Trail of where the bot has actually driven — disabled for now,
		// re-enable by uncommenting if you want the breadcrumb viz back.
		// this._updateTrail(state.x, state.z);

		// Nav target flag (2D position on the ground). Bob + spin so it
		// reads as "go here" not just another course marker.
		if (navTarget !== null) {
			this.target.visible = true;
			const t = performance.now() * 0.001;
			const ty = heightAt(navTarget.x, navTarget.z ?? 0);
			this.target.position.set(navTarget.x, ty + 0.05 + 0.05 * Math.sin(t * 3), navTarget.z ?? 0);
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
				f.position.set(queueRest[i].x, heightAt(queueRest[i].x, queueRest[i].z), queueRest[i].z);
			} else {
				f.visible = false;
			}
		}

		// Camera follows the bot. Two modes:
		//
		//   user-orbit (recent drag): camera pans with the bot, view angle
		//     preserved. The user's chosen pose stays put.
		//
		//   auto-follow (idle > 2.5 s): camera smoothly slews around to a
		//     third-person view directly behind the bot, at the user's last
		//     captured distance + height. LPF coefficient ~0.025 → ~1.5 s
		//     time constant. Fast enough to feel responsive, slow enough to
		//     not lurch.
		//
		// The bot's forward in world coords is (cos(h), -sin(h)) for (x, z),
		// matching the plant's heading convention. "Behind" the bot is the
		// negation: (-cos(h), +sin(h)).
		// focusY tracks the terrain height under the bot so the camera frames
		// the bot at the right altitude in valleys and over hills.
		const groundY = heightAt(state.x, state.z);
		const focusY  = groundY + Math.max(0.6, L * 0.8);
		const newTarget = new THREE.Vector3(state.x, focusY, state.z);
		const idleSec = (performance.now() - this.lastInteractionTime) / 1000;
		const autoFollow = this.autoFollowEnabled && !this.userInteracting && idleSec > 2.5;

		if (autoFollow) {
			// Default chase-cam distance is wide enough to see a chunk of
			// the park around the bot — the world is ~110 m × 70 m, so a
			// 6 m chase view feels claustrophobic. User-set values stick.
			const D = this.followDistance ?? 12;
			const H = this.followHeight   ?? 6;
			const desX = state.x - D * Math.cos(state.heading);
			const desZ = state.z + D * Math.sin(state.heading);
			const desY = focusY + H;
			const alpha = 0.025;
			this.camera.position.x += (desX - this.camera.position.x) * alpha;
			this.camera.position.y += (desY - this.camera.position.y) * alpha;
			this.camera.position.z += (desZ - this.camera.position.z) * alpha;
		} else {
			// Translate camera with the bot so the user's orbit is preserved.
			const delta = newTarget.clone().sub(this.controls.target);
			this.camera.position.add(delta);
		}
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
