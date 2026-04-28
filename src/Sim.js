// Sim — the physical world. Everything the bot's body is or senses, and
// nothing else. No controllers, no planner, no UI, no scheduling.
//
// Owns:
//   pendulum     — 2D pendulum-on-wheels rigid body
//   motor        — PWM → force model
//   Sensors      — IMU + encoder noise/bias on top of pendulum truth
//   lidar        — 24-ray fan against the world's obstacle list
//   roadSensor   — color "camera" against the world's road texture
//   occupancyGrid — accumulated lidar map (the bot's "memory" of walls)
//   imuBiasInjected — disturbance bias added to every IMU sample
//
// API mirrors what a real robot would expose: sample sensors, push an
// actuator command (force + yaw torque), advance time. Disturbance
// methods (pushChassis / yawKick / shoveTip / setImuBias) let the
// outside world inject impulses without poking at pendulum.state.

import { Pendulum }			from './physics/pendulum.js';
import { Motor }			from './physics/motor.js';
import { Sensors }			from './physics/sensors.js';
import { Lidar }			from './physics/lidar.js';
import { RoadSensor }  		from './physics/road-sensor.js';
import { OccupancyGrid }	from './world/occupancy_grid.js';

export class Sim {
	constructor({ params, motorCfg }) {
		this.DT       = 1 / 500;     // physics integration step (s) — 500 Hz
		this.pendulum = new Pendulum(params);
		this.motor    = new Motor(motorCfg);
		this.sensors  = new Sensors();
		this.measured = null;        // most recent sample (held between sensor ticks)

		// Lidar — 24-ray fan, 5 m range. Road sensor uses the same ray
		// count for shape-compatibility with RoadController; range 8 m
		// is enough to plan a corner without distant fork branches
		// dominating the steering vote.
		this.lidar      = new Lidar({ rays: 24, maxRange: 5 });
		this.roadSensor = new RoadSensor({ rays: 24, maxRange: 8 });

		// Map sized to cover the park footprint with margin; cellSize
		// 0.25 m matches the planner's grid resolution.
		this.occupancyGrid = new OccupancyGrid({
			originX: -12, originZ: -6, width: 26, height: 14, cellSize: 0.25,
		});

		this.simElapsedTime  = 0;    // sim-time elapsed since last reset (s)
		this.imuBiasInjected = 0;    // additive bias on sensors.pitch (the bot DOESN'T know)

		// World refs — set externally once the renderer exists, since the
		// scene's obstacle list and road canvas live there.
		this.obstacles  = null;
		this.roadCanvas = null;
	}

	setWorld({ obstacles, roadCanvas }) {
		this.obstacles  = obstacles;
		this.roadCanvas = roadCanvas;
	}

	// Apply per-frame UI changes — pendulum mass/length etc., motor cfg.
	applyConfig({ params, motorCfg }) {
		this.pendulum.params = params;
		Object.assign(this.motor, motorCfg);
	}

	// Reset to the standard spawn pose. Caller passes initial pitch (rad)
	// from the UI; everything else is a fixed park location chosen to
	// give a reasonable first drive direction.
	reset({ params, initialPitch }) {
		this.pendulum.params = params;
		// Spawn at the park's west-entrance node, heading toward the
		// south-fork (node A → node B). Heading from world (dx, dz):
		//   forward = (cos h, -sin h), so h = atan2(-dz, dx).
		// Coordinates match the road-network's loadPark() scale (S=5).
		const SPAWN_X = -45, SPAWN_Z = -5;
		const SPAWN_HEADING = Math.atan2(-(-15 - -5), (-15 - -45));   // ≈ 0.32 rad
		this.pendulum.setState({
			x: SPAWN_X, z: SPAWN_Z, vel_cart: 0,
			pitch: initialPitch, pitch_rate: 0,
			heading: SPAWN_HEADING, yaw_rate: 0,
		});
		this.sensors.reset();
		this.motor.reset();
		this.measured       = null;
		this.simElapsedTime = 0;
	}

	// Take one sensor snapshot. Loop calls this only when the sensor's
	// period has elapsed; the cached `measured` is held by ZOH between
	// ticks. Scan flags ask which exteroceptors are wanted on this tick:
	//   scanLidar     — lidar ray fan (planner & viz)
	//   scanRoad      — color sensor (road pilot)
	//   integrateMap  — fold the lidar return into occupancyGrid (lidar_astar)
	sampleSensors(params, sensorCfg, { scanLidar, scanRoad, integrateMap }) {
		this.measured = this.sensors.sample(this.pendulum.state, params, sensorCfg, this.simElapsedTime);

		// Inject IMU bias if the disturbance panel turned it on. The
		// controller sees a tilted "upright" — its auto-trim should
		// eventually absorb a fixed bias; a step bias exposes the time
		// constant.
		if (this.imuBiasInjected !== 0) this.measured.pitch += this.imuBiasInjected;

		if (scanRoad) {
			this.measured.road = this.roadSensor.scan(this.pendulum.state, this.roadCanvas);
		}
		if (scanLidar) {
			this.measured.lidar = this.lidar.scan(this.pendulum.state, this.obstacles);
			if (integrateMap) {
				this.occupancyGrid.integrateScan(
					{ x: this.pendulum.state.x, z: this.pendulum.state.z ?? 0 },
					this.measured.lidar,
					this.lidar.maxRange,
				);
			}
		}
		return this.measured;
	}

	// Apply force (N, body-forward) and yaw torque (N·m), advance one DT.
	integrate(force, yawTorque) {
		this.pendulum.step(force, yawTorque, this.DT);
		this.simElapsedTime += this.DT;
	}

	hasFallen() { return Math.abs(this.pendulum.state.pitch) > Math.PI / 2; }

	// --- Disturbance API — formal entry points so callers don't reach
	//     into pendulum.state directly. -----------------------------------

	// Chassis impulse (N·s) along body-forward. Returns the resulting Δv
	// for logging.
	pushChassis(impulse) {
		const total_mass = this.pendulum.params.M + this.pendulum.params.m;
		const dv = impulse / total_mass;
		this.pendulum.state.vel_cart += dv;
		return dv;
	}

	// Yaw impulse (N·m·s). Returns Δω for logging.
	yawKick(impulse) {
		const dw = impulse / this.pendulum.params.I_yaw;
		this.pendulum.state.yaw_rate += dw;
		return dw;
	}

	// Tip-direction shove — adds directly to pitch_rate (rad/s).
	shoveTip(rate) { this.pendulum.state.pitch_rate += rate; }

	setImuBias(rad) { this.imuBiasInjected = rad; }
}
