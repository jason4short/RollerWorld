// Pilot — the "what should we do" layer. Reads input (joystick, arrow
// keys, road sensor, planned route) and produces a Command for the
// Robot. Owns waypoint state + the planner because at this scale the
// pilot and the mission are the same person — a human picks goals and
// flies between them.
//
// Modes (set via the UI):
//   raw   — arrow-key tilt, direct lean control
//   fbw   — joystick maps to fwd/yaw rate (fly-by-wire)
//   auto  — chase a waypoint queue; planner builds the route
//   road  — color sensor → virtual joystick (lane following)
//
// No physics, no controllers. Pilot reads the most recent sensor
// snapshot, optionally consults exteroceptors (lidar, road), and
// returns a Command bag the Robot will translate into force/torque.
//
// Telemetry from inside the pilot dispatch (vel_desired_last, err_last,
// nav.target_x, etc.) is exposed on this.nav and friends — Loop reads
// them when assembling the history row.

import { NavController }	from './controllers/nav.js';
import { RoadController }	from './controllers/road.js';
import { ReactiveNav }		from './controllers/reactive-nav.js';
import { Planner }			from './controllers/stack/planner.js';

export class Pilot {
	constructor({ ui, joystick, lidarMaxRange }) {
		this.ui            = ui;
		this.joystick      = joystick;
		this.lidarMaxRange = lidarMaxRange;

		// Three "virtual stick" producers — each turns one kind of input
		// into a (fwd, yaw) stick or a tilt+yaw_rate pair. Pilot picks
		// one per frame based on the active mode.
		this.nav			= new NavController();    // waypoint / FBW pilot → tilt + yaw_rate
		this.road			= new RoadController();   // color sensor → virtual stick (Road pilot)
		this.reactiveNav	= new ReactiveNav();      // goal-biased FTG: lidar + waypoint → virtual stick

		this.planner		= new Planner();          // 'direct' | 'astar' | 'lidar_astar' | 'reactive'
		this.waypoints		= [];

		// Replan throttling — sim-time of last replan and the running
		// simElapsedTime, so the hysteresis test doesn't drift across
		// pause/resume.
		this._lastReplanT = 0;

		// --- Keyboard pilot state --------------------------------------
		// Pilot directly sets a tilt target — hold ↑/↓ to lean, bot
		// accelerates while leaned. Release → tilt = 0, bot returns
		// upright and coasts to a stop via friction.
		this.keyTilt        = 0;
		this.keyTiltMax     = 10 * (Math.PI / 180);   // hold-key tilt limit (rad) — 10°
		this.keyYawRate     = 0;
		this.keyYawRateMax  = 6.0;                    // arrow-key yaw rate (rad/s) — ~340°/s
		this.keyYawHeading  = 0;                      // integrated arrow-key yaw → heading target
	}

	reset() {
		this.waypoints.length   = 0;
		this.keyYawRate       	= 0;
		this.keyYawHeading 		= 0;
		this.nav.reset?.();
	}

	// --- Sensor-scan flags. Loop asks these before each sensor sample
	//     so Sim only fires the lidar / road sensor when something
	//     actually wants the data. -----------------------------------
	wantsLidar()       { return this.planner.mode === 'lidar_astar' || this.planner.mode === 'reactive'; }
	wantsRoad()        { return this.ui.readPilotMode() === 'road'; }
	shouldIntegrateMap() { return this.planner.mode === 'lidar_astar'; }


	// --- Keyboard hooks (App.wireKeys writes here) ------------------
	setKeyTilt(t)    { this.keyTilt = t; }
	setKeyYawRate(r) { this.keyYawRate = r; }


	// --- Waypoint / target hooks (App.wireUI writes here) -----------
	clearWaypoints() { this.waypoints.length = 0; }

	setNavTarget(x, z) {
		this.nav.target_x = x;
		this.nav.target_z = z;
	}

	addWaypoints(list) {
		for (const wp of list) this.waypoints.push(wp);
		const head = this.waypoints[0];
		if (head) this.setNavTarget(head.x, head.z);
	}

	// Replan in lidar_astar mode — but ONLY when the current path is
	// invalidated by newly-mapped walls (LOS broken on some segment),
	// or it's been a long time since the last refresh. Replanning every
	// second made the bot jitter: A* would find a slightly different
	// path each tick and the immediate target kept jumping. The path
	// stays stable until a wall actually cuts it.
	maybeReplan(startState, occupancyGrid, simElapsedTime) {
		if (this.planner.mode !== 'lidar_astar' || !this.planner.goal) return;
		const startPos = { x: startState.x, z: startState.z ?? 0 };

		// Validity check uses pad=0 — literally "is a wall ON this
		// segment now?" The planner already inflated by 0.6 when
		// proposing the path, so the path is guaranteed clear. We only
		// invalidate when something LITERALLY appears on the planned line.
		//
		// Plus a hysteresis: even when blocked, replans happen at most
		// once per 1.5s.
		let pathBlocked = false;
		let prev = startPos;
		for (const wp of this.waypoints) {
			if (!occupancyGrid.hasLineOfSight(prev, wp, 0)) { pathBlocked = true; break; }
			prev = wp;
		}
		const sinceLast = simElapsedTime - this._lastReplanT;
		const stale     = sinceLast > 10;     // 10 s refresh
		const canReplan = sinceLast > 1.5;    // hysteresis: min 1.5 s between
		const noPath    = this.waypoints.length === 0;

		if (!((pathBlocked && canReplan) || stale || noPath)) return;
		this._lastReplanT = simElapsedTime;
		const path = this.planner.plan(
			startPos, this.planner.goal,
			{ obstacles: occupancyGrid, res: 0.25, pad: 0.6 },
		);
		if (path.length > 0) {
			this.waypoints.length = 0;
			this.waypoints.push(...path);
			this.setNavTarget(path[0].x, path[0].z);
		}
		const head   = path[0] ?? { x: NaN, z: NaN };
		const next   = path[1] ?? { x: NaN, z: NaN };
		const reason = pathBlocked ? 'blocked' : stale ? 'stale' : 'first';
		this.ui.log(simElapsedTime,
			`replan (${reason}): n=${path.length} ` +
			`head=(${head.x.toFixed(2)},${head.z.toFixed(2)}) ` +
			`next=(${next.x.toFixed(2)},${next.z.toFixed(2)})`);
	}

	// Build a Command from current pilot inputs + sensors. Called once
	// per RAF requestAnimationFrame (pilot decisions are human-perceptible, no need for inner-
	// loop rate). Modes:
	//   fbw   — joystick → cascadeCommand or tilt+yaw_rate
	//   auto  — waypoint nav (with reactive variant on lidar)
	//   road  — color sensor → virtual stick
	//   raw   — arrow-key tilt, integrated yaw heading
	command(measured, isCascade, deltaTime, simElapsedTime) {
		// Fall back to a synthetic state if Sim hasn't sampled yet
		// (first frame after reset). Pilot uses .x/.z/.heading for nav
		// and reactive paths.
		const senseOrTruth = measured ?? null;
		const pilotMode = this.ui.readPilotMode();
		
		let tiltSetpoint = 0, yawRateSetpoint = 0;
		let cascadeCommand = null;

		if (pilotMode === 'fbw' && this.joystick) {
			const s = this.joystick.value();
			// Screen-up = forward, screen-right = turn right (negative
			// yaw_rate, matching ArrowRight's sign convention). Yaw
			// scaled down — full stick is too aggressive on a balance
			// bot, easier to drive with a softer turning rate.
			const FBW_YAW_SCALE = 0.25;
			const stick = { fwd: s.y, yaw: -s.x * FBW_YAW_SCALE };

			if (isCascade) {
				cascadeCommand = { mode: 'fbw', stick };
			} else {
				const out = this.nav.updateFBW(senseOrTruth, stick, this.ui.readNavGains(), deltaTime);
				tiltSetpoint    = out.tilt;
				yawRateSetpoint = out.yaw_rate;
			}
			
		} else if (pilotMode === 'auto') {
			// Goal-biased reactive nav — when planner is in 'reactive'
			// mode, Auto pilot has a waypoint, but obstacle avoidance is
			// FTG-on-lidar instead of A*. Plugs into the cascade through
			// the FBW path as a virtual stick.
			if (this.planner.mode === 'reactive') {
				const stick = this.reactiveNav.update(
					measured?.lidar,
					senseOrTruth,
					{ x: this.nav.target_x, z: this.nav.target_z },
					this.lidarMaxRange,
				);
				
				if (isCascade) {
					cascadeCommand = { mode: 'fbw', stick };
					
				} else {
					const out = this.nav.updateFBW(senseOrTruth, stick, this.ui.readNavGains(), deltaTime);
					tiltSetpoint    = out.tilt;
					yawRateSetpoint = out.yaw_rate;
				}
			} else if (isCascade) {
				cascadeCommand = { mode: 'auto' };
				
			} else {
				const out = this.nav.update(senseOrTruth, this.ui.readNavGains(), deltaTime);
				tiltSetpoint    = out.tilt;
				yawRateSetpoint = out.yaw_rate;
			}
			this.advanceWaypointIfArrived(measured, simElapsedTime);

		} else if (pilotMode === 'road') {
			// Reactive road pilot — turn the latest color-sensor scan
			// into a virtual FBW stick. Goes through the cascade's FBW
			// path so velocity and yaw-rate tracking are handled by the
			// existing Nav layer.
			const stick = this.road.update(
				measured?.road,
				senseOrTruth?.heading ?? 0,
				this.lidarMaxRange,   // road sensor uses same range parameter for shape compat
			);
			if (isCascade) {
				cascadeCommand = { mode: 'fbw', stick };
			} else {
				const out = this.nav.updateFBW(senseOrTruth, stick, this.ui.readNavGains(), deltaTime);
				tiltSetpoint    = out.tilt;
				yawRateSetpoint = out.yaw_rate;
			}
		} else {
			// Raw arrow-key tilt. Integrate the arrow-key yaw rate into
			// a virtual heading reference so the Attitude layer (cascade)
			// or yaw torque loop (legacy) gets a meaningful target while
			// arrows are held.
			this.keyYawHeading += this.keyYawRate * deltaTime;
			while (this.keyYawHeading >  Math.PI) this.keyYawHeading -= 2 * Math.PI;
			while (this.keyYawHeading < -Math.PI) this.keyYawHeading += 2 * Math.PI;

			if (isCascade) {
				// Bypass Mixer, drive Attitude's pitch_target directly.
				// heading_rate_ff carries the instantaneous rate so the
				// bot rotates smoothly between integrated-target updates
				// instead of stepping.
				cascadeCommand = {
					mode:            'tilt',
					pitch_target:    this.keyTilt,
					yaw_target:      this.keyYawHeading,
					heading_rate_ff: this.keyYawRate,
				};
			} else {
				tiltSetpoint    = this.keyTilt;
				yawRateSetpoint = this.keyYawRate;
			}
		}

		return {
			tiltSetpoint,
			yawRateSetpoint,
			cascadeCommand,
			navTarget:      { x: this.nav.target_x, z: this.nav.target_z },
			navVelDesired:  this.nav.vel_desired_last ?? 0,
		};
	}

	// Auto mode: when the bot is within the arrival radius of the head
	// waypoint, drop it and advance. Empty queue → kick back to FBW so
	// the pilot has control again.
	advanceWaypointIfArrived(measured, simElapsedTime) {
		const head = this.waypoints[0];
		if (!head) return;
		const sx = measured?.x ?? 0;
		const sz = measured?.z ?? 0;
		const dx = head.x - sx;
		const dz = head.z - sz;
		const arrival = this.ui.readNavGains().yaw_disable_radius || 0.2;
		if (Math.hypot(dx, dz) > arrival) return;

		this.waypoints.shift();
		this.ui.log(simElapsedTime, `WP reached  · queue=${this.waypoints.length}`);
		const next = this.waypoints[0];
		
		if (next) {
			this.setNavTarget(next.x, next.z);
			document.getElementById('navTargetX').value = next.x.toFixed(2);
			const tz = document.getElementById('navTargetZ');
			if (tz) tz.value = next.z.toFixed(2);
		} else {
			document.getElementById('pilotMode').value = 'fbw';
			document.getElementById('navEnabled').checked = false;
			this.ui.log(simElapsedTime, 'route done → FBW');
		}
	}
}
