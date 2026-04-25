import { Pendulum }							from './physics/pendulum.js';
import { Motor }								 from './physics/motor.js';
import { Sensors }							 from './physics/sensors.js';
import { Lidar }								 from './physics/lidar.js';
import { PIDController }				 from './controllers/pid.js';
import { ArduBalanceController } from './controllers/ardubalance.js';
import { NavController }				 from './controllers/nav.js';
import { YawController }				 from './controllers/yaw.js';
import { NNController }					from './controllers/nn.js';
import { ControllerStack }				from './controllers/stack/index.js';
import { Planner }							from './controllers/stack/planner.js';
import { MLP }									 from './nn/mlp.js';
import { NNTrainer }						 from './nn/trainer.js';
import { WorldRenderer3D }			 from './render/world-renderer-3d.js';
import { Plotter, PLOT_SIGNALS } from './render/plotter.js';
import { UI }										from './ui.js';
import { Recorder }							from './recorder.js';
import { PRESETS }							 from './presets.js';
import { MotorCalibrator }			 from './calibration.js';
import { Joystick }							from './ui/joystick.js';

// App — owns the simulation loop and wires everything together.
//
// Responsibilities:
//	 - Read UI state each frame
//	 - Schedule sensor sampling, outer attitude loop, and inner motor loop
//		 at their configured rates against the 500 Hz physics integrator
//	 - Dispatch pilot commands (arrow keys) to the active controller
//	 - Drive the renderer and plotter

export class App {
	constructor() {
		this.DT = 1 / 500;
		this.ui			 = new UI();
		this.plant		= new Pendulum(this.ui.readParams());
		this.motor		= new Motor(this.ui.readMotor());
		this.sensors	= new Sensors();
		this.measured = null;		 // most recent sensor sample, held between sensor ticks
		this.lastForce = 0;			 // held between inner-loop ticks (ZOH)

		this.controllers = {
			pid: new PIDController(),
			ardubalance: new ArduBalanceController(),
			nn: new NNController(),
		};

		this.nnTrainer 			= new NNTrainer();
		this.controllerType 	= this.ui.readController();
		this.nav 				= new NavController();
		this.yawController 		= new YawController();
		this.stack 				= new ControllerStack();
		this._lastStackOut		= null;
		this.planner			= new Planner();
		this.lidar				= new Lidar({ rays: 24, maxRange: 5 });
		this.lidarEnabled		= true;
		this.currentTab			= 'control';   // 'control' | 'sim'

		// Disturbance state — biases / impulses injected by the Disturbances
		// panel. Impulses are applied as instantaneous state kicks; bias is
		// added to the sensor pitch reading every tick while enabled.
		this.imuBiasInjected = 0;
		this.calibrator 		= new MotorCalibrator({ dt: this.DT });
		const joyEl 			= document.getElementById('joystick');
		this.joystick 			= joyEl ? new Joystick(joyEl) : null;
		this.recorder 			= new Recorder();
		this.renderer 			= new WorldRenderer3D(document.getElementById('world'));
		this.plotter			= new Plotter(document.getElementById('plot'));
		// Per-layer inset plots inside each cascade panel — let the visitor
		// see each layer's input/output rolling alongside its gain panel.
		// Short window (3 s) so transients are vivid without scrolling.
		this.panelPlots = {
			mixer:  new Plotter(document.getElementById('plotMixer'),    3),
			pitch:  new Plotter(document.getElementById('plotAttPitch'), 3),
			yaw:    new Plotter(document.getElementById('plotAttYaw'),   3),
			wheels: new Plotter(document.getElementById('plotWheels'),   3),
		};

		this.running 	= false;
		this.tSim		= 0;
		this.history 	= [];
		this.acc		= 0;
		this.lastT	 	= 0;

		// Time until each sub-loop is due to run (s).
		this.dueSensor 	= 0;
		this.dueOuter	= 0;
		this.dueInner	= 0;

		// Keyboard pilot command: target tilt angle (rad) that the active
		// controller drives toward. Left/right arrows hold to lean.
		// Pilot directly sets a target tilt — hold ↑/↓ to lean, bot accelerates
		// while leaned. Release → tilt = 0, bot returns to upright and coasts
		// to a stop via friction. Simple and stable.
		this.pilotTilt			= 0;
		this.pilotTiltMax 		= 10 * (Math.PI / 180);
		this.pilotYawRate		= 0;
		this.pilotYawRateMax 	= 6.0; // rad/s (~340°/s — match real bot)
		// Cascade tilt-mode integrates pilotYawRate into a heading reference
		// so arrow-key turns produce a real heading_target the Attitude layer
		// can track. Same trick as FBW's heading integration, but here it
		// lives on App since arrow keys are an app-level pilot input.
		this.pilotYawHeadingRef = 0;
		this.lastTauYaw			= 0;

		// Waypoint queue. Shift+click appends; Auto mode chases head, on
		// arrival pops to the next, exits to FBW when empty.
		this.waypoints = [];

		this.wireUI();
		this.wireKeys();
		this.syncControllerPanels();
		this.populatePlotMenus();
		this.reset();
		this.drawLUT();
		requestAnimationFrame(ts => this.tick(ts));
	}

	currentController() { return this.controllers[this.controllerType]; }

	// Auto mode: when the bot is within the arrival radius of the current
	// waypoint, drop it from the queue and advance. Empty queue → kick
	// back to FBW so the pilot has control again.
	_advanceWaypointIfArrived() {
		const head = this.waypoints[0];
		if (!head) return;
		const sx = (this.measured || this.plant.state).x;
		const sz = (this.measured || this.plant.state).z ?? 0;
		const dx = head.x - sx;
		const dz = head.z - sz;
		const arrival = this.ui.readNavGains().yaw_disable_radius || 0.2;
		if (Math.hypot(dx, dz) > arrival) return;

		this.waypoints.shift();
		this.ui.log(this.tSim, `WP reached  · queue=${this.waypoints.length}`);
		const next = this.waypoints[0];
		if (next) {
			this.nav.target_x = next.x;
			this.nav.target_z = next.z;
			document.getElementById('navTargetX').value = next.x.toFixed(2);
			const tz = document.getElementById('navTargetZ');
			if (tz) tz.value = next.z.toFixed(2);
		} else {
			document.getElementById('pilotMode').value = 'fbw';
			document.getElementById('navEnabled').checked = false;
			this.ui.log(this.tSim, 'route done → FBW');
		}
	}

	currentGains() {
		if (this.controllerType === 'ardubalance') return this.ui.readArduGains();
		if (this.controllerType === 'cascade')     return null;   // cascade reads its own gains via readCascadeGains
		return this.ui.readGains();
	}

	reset() {
		const { th0 } = this.ui.readInit();
		this.plant.params = this.ui.readParams();
		this.plant.setState({
			x: 0, z: 0, vel_cart: 0,
			pitch: th0 * Math.PI / 180, pitch_rate: 0,
			heading: 0, yaw_rate: 0,
		});
		this.lastTauYaw = 0;
		this.pilotYawRate = 0;
		this.pilotYawHeadingRef = 0;
		this.waypoints.length = 0;
		for (const c of Object.values(this.controllers)) c.reset();
		this.stack.reset();
		this.sensors.reset();
		this.measured		= null;
		this.lastForce		= 0;
		this.dueSensor		= 0;
		this.dueOuter		= 0;
		this.dueInner		= 0;
		this.tSim 			= 0;
		this.history.length = 0;
		this.render();
	}

	render() {
		const showFlag = this.ui.navEnabled() || this.ui.readPilotMode() === 'auto';
		const navTarget = showFlag
			? { x: this.nav.target_x, z: this.nav.target_z }
			: null;
		// Upcoming WPs (excluding the head, which is rendered as the bobbing
		// target). Empty when queue is one or zero deep.
		const queueRest = this.waypoints.length > 1 ? this.waypoints.slice(1) : [];
		// Draw the path the planner laid down: bot → each remaining waypoint.
		// Shrinks naturally as waypoints are consumed.
		this.renderer.setPath(this.waypoints.length
			? [{ x: this.plant.state.x, z: this.plant.state.z ?? 0 }, ...this.waypoints]
			: []);

		// Lidar — read the latest scan from sensors (computed at sensorHz)
		// and hand to renderer for visualization. When the Safety governor
		// is clipping (scale < 1), highlight the rays inside its forward
		// arc in red so the visitor can SEE which rays are braking the bot.
		const lidarRays = (this.lidarEnabled && this.measured?.lidar) ? this.measured.lidar : null;
		let highlight = null;
		if (lidarRays && this.controllerType === 'cascade'
		    && this.stack.safety.lastScale < 0.999) {
			const fwdArc = Math.PI / 4;
			const heading = this.plant.state.heading;
			highlight = lidarRays.map(r => {
				let a = r.angle - heading;
				while (a >  Math.PI) a -= 2 * Math.PI;
				while (a < -Math.PI) a += 2 * Math.PI;
				return Math.abs(a) <= fwdArc;
			});
		}
		this.renderer.setLidar(lidarRays, this.plant.state, highlight);

		// Periodic safety log so the numbers behind the brake are visible.
		// Throttled to ~2 Hz when actively clipping; silent otherwise.
		if (this.controllerType === 'cascade' && this.stack.safety.lastScale < 0.999) {
			const now = performance.now();
			if (!this._lastSafetyLogT || now - this._lastSafetyLogT > 500) {
				this._lastSafetyLogT = now;
				this.ui.log(this.tSim,
					`safety: scale=${this.stack.safety.lastScale.toFixed(2)} ` +
					`minDist=${this.stack.safety.lastMinDist.toFixed(2)}m`);
			}
		}

		this.renderer.draw(this.plant.state, this.plant.params, navTarget, queueRest);
		const fRef = this.controllerType === 'ardubalance'
			? this.motor.Km
			: this.controllerType === 'cascade'
				? this.ui.num('att_force_max')
				: this.ui.readGains().Fmax;
		const pwmRef = this.motor.PWM_max || 2000;
		const series = this.buildPlotSeries(fRef, pwmRef);
		this.plotter.draw(this.history, series);
		this.drawPanelPlots(pwmRef);
		this._syncJoystickVisibility();
	}

	// Show the overlay joystick only when the pilot is actually using it
	// (FBW or Raw tilt). Auto mode hides it — nav drives, the joystick
	// would just clutter the world view.
	_syncJoystickVisibility() {
		const joy = document.getElementById('joystick');
		if (!joy) return;
		const mode = this.ui.readPilotMode();
		joy.style.display = (mode === 'auto') ? 'none' : '';

		// Sync the canvas-overlay pilot buttons too. Angle/FBW reflect
		// the current mode; Auto indicator lights up only in auto.
		const setActive = (id, on) => {
			const el = document.getElementById(id);
			if (el) el.classList.toggle('active', on);
		};
		setActive('btnPilotAngle', mode === 'raw');
		setActive('btnPilotFbw',   mode === 'fbw');
		setActive('btnPilotAuto',  mode === 'auto');
	}


	// Inset plots inside each cascade panel. Each shows the few signals
	// that layer produces or consumes, scaled so a healthy controller
	// fills roughly half the canvas height. When cascade isn't active,
	// the panels are hidden anyway, so we can skip drawing.
	drawPanelPlots(pwmRef) {
		if (this.controllerType !== 'cascade') return;
		const tiltLim   = Math.PI / 6;
		const forceLim  = this.ui.num('att_force_max') || 60;
		const torqueLim = this.ui.num('att_torque_max') || 2;
		const SIGNALS = {
			mixer: [
				{ key: 'vel_cart',     color: '#6cf', scale: 1 / 3,      label: 'vel' },
				{ key: 'vel_desired',  color: '#fc6', scale: 1 / 3,      label: 'target' },
				{ key: 'pitch_target', color: '#f6c', scale: 1 / tiltLim, label: 'tilt' },
			],
			pitch: [
				{ key: 'pitch',        color: '#6cf', scale: 1 / tiltLim, label: 'pitch' },
				{ key: 'pitch_target', color: '#fc6', scale: 1 / tiltLim, label: 'target' },
				{ key: 'force_fwd',    color: '#6f9', scale: 1 / forceLim, label: 'force' },
			],
			yaw: [
				{ key: 'torque_yaw',   color: '#f6c', scale: 1 / torqueLim, label: 'τ_yaw' },
			],
			wheels: [
				{ key: 'pwm_left',     color: '#6cf', scale: 1 / pwmRef, label: 'L' },
				{ key: 'pwm_right',    color: '#fc6', scale: 1 / pwmRef, label: 'R' },
			],
		};
		for (const [name, plotter] of Object.entries(this.panelPlots)) {
			plotter.draw(this.history, SIGNALS[name]);
		}
	}

	populatePlotMenus() {
		const keys = Object.keys(PLOT_SIGNALS);
		const defaults = ['pitch', 'vel_cart', 'vel_desired'];	 // sensible for nav debugging
		for (let i = 0; i < 3; i++) {
			const sel = document.getElementById(`plot${i + 1}`);
			sel.innerHTML = '<option value="none">(none)</option>' +
				keys.map(k => `<option value="${k}">${PLOT_SIGNALS[k].label}</option>`).join('');
			sel.value = defaults[i] || 'none';
		}
	}

	buildPlotSeries(fRef, pwmRef) {
		const colors = ['#ec6', '#6cf', '#2a6'];
		const slots = ['plot1', 'plot2', 'plot3'];
		const out = [];

		for (let i = 0; i < slots.length; i++) {
			const key = document.getElementById(slots[i]).value;
			if (!key || key === 'none') continue;
			const info = PLOT_SIGNALS[key];
			if (!info) continue;
			let scale = info.scale;
			if (key === 'F')			scale = 1 / (fRef	|| 1);
			if (key === 'pwm')			scale = 1 / (pwmRef || 1);
			if (key === 'pwm_nn')		scale = 1 / (pwmRef || 1);
			if (key === 'pwm_residual') scale = 1 / (pwmRef || 1);
			if (key === 'pwm_left')		scale = 1 / (pwmRef || 1);
			if (key === 'pwm_right')	scale = 1 / (pwmRef || 1);
			out.push({ key, color: colors[i], scale, label: info.label });
		}
		return out;
	}

	tick(ts) {
		if (this.running) {
			if (!this.lastT) this.lastT = ts;
			this.acc += Math.min(0.05, (ts - this.lastT) / 1000);

			this.lastT 			= ts;
			const params		= this.ui.readParams();
			const gains			= this.currentGains();
			const sensorCfg 	= this.ui.readSensors();
			const rates			= this.ui.readRates();
			this.plant.params 	= params;
			
			Object.assign(this.motor, this.ui.readMotor());

			const controller = this.currentController();
			const isCascade  = this.controllerType === 'cascade';

			// Nav outer-outer loop: if enabled, position error drives the tilt
			// setpoint. Otherwise the pilot (arrow keys) drives it directly.
			const navOn = this.ui.navEnabled();

			// Nav runs once per animation frame (~60 Hz). Approximate its dt from
			// wall-clock elapsed so its internal LPF is rate-correct regardless
			// of browser frame pacing.
			const navDt = Math.max(0.001, Math.min(0.1, (ts - (this._lastNavTs || ts)) / 1000));
			this._lastNavTs = ts;


			// Input priority: FBW pilot (joystick) > nav waypoint > raw arrow tilt.
			// Two parallel routings:
			//   Cascade controller       — pilot input → cascadeCommand, the stack handles the rest.
			//   Legacy (ArduBalance/PID) — pilot input → tiltSetpoint + yawRateSetpoint.
			const pilotMode = this.ui.readPilotMode();
			let tiltSetpoint = 0, yawRateSetpoint = 0;
			let cascadeCommand = null;

			if (pilotMode === 'fbw' && this.joystick) {
				const s		= this.joystick.value();
				// Screen-up = forward, screen-right = turn right (negative
				// yaw_rate, matching ArrowRight's sign convention).
				// Yaw scaled down — full stick is too aggressive on a balance
				// bot, easier to drive with a softer turning rate.
				const FBW_YAW_SCALE = 0.25;
				const stick	= { fwd: s.y, yaw: -s.x * FBW_YAW_SCALE };
				if (isCascade) {
					cascadeCommand = { mode: 'fbw', stick };
				} else {
					const out	= this.nav.updateFbw(this.measured || this.plant.state,
						stick, this.ui.readNavGains(), navDt);
					tiltSetpoint	 = out.tilt;
					yawRateSetpoint  = out.yaw_rate;
				}

			} else if (pilotMode === 'auto') {
				if (isCascade) {
					cascadeCommand = { mode: 'auto' };
				} else {
					const out = this.nav.update(this.measured || this.plant.state, this.ui.readNavGains(), navDt);
					tiltSetpoint	 = out.tilt;
					yawRateSetpoint  = out.yaw_rate;
				}
				this._advanceWaypointIfArrived();

			} else {
				// Integrate the arrow-key yaw rate into a virtual heading
				// reference so the Attitude layer (cascade) or yaw torque
				// loop (legacy) gets a meaningful target while arrows are held.
				this.pilotYawHeadingRef += this.pilotYawRate * navDt;
				while (this.pilotYawHeadingRef >  Math.PI) this.pilotYawHeadingRef -= 2 * Math.PI;
				while (this.pilotYawHeadingRef < -Math.PI) this.pilotYawHeadingRef += 2 * Math.PI;

				if (isCascade) {
					// Arrow-key debug: bypass Mixer, drive Attitude's pitch_target
					// directly. Yaw target is the integrated arrow-key rate;
					// heading_rate_ff carries the instantaneous rate so the bot
					// rotates smoothly between browser frames instead of stepping.
					cascadeCommand = {
						mode:            'tilt',
						pitch_target:    this.pilotTilt,
						yaw_target:      this.pilotYawHeadingRef,
						heading_rate_ff: this.pilotYawRate,
					};
				} else {
					tiltSetpoint	 = this.pilotTilt;
					yawRateSetpoint  = this.pilotYawRate;
				}
			}

			if (isCascade) {
				// Single source of truth for the waypoint target — the legacy
				// NavController owns `target_x/z`; the stack mirrors it.
				this.stack.nav.target_x = this.nav.target_x;
				this.stack.nav.target_z = this.nav.target_z;
			} else if (controller instanceof ArduBalanceController) {
				controller.target_angle = tiltSetpoint;
			} else if (controller instanceof NNController) {
				// NN swallows the velocity-tracking step; it takes vel_cart_target
				// (post-slew) directly from nav rather than a tilt setpoint.
				controller.vel_cart_target = this.nav.vel_desired_last ?? 0;
			}

			const dtSensor = 1 / Math.max(1, rates.sensorHz);
			const dtOuter	= 1 / Math.max(1, rates.outerHz);
			const dtInner	= 1 / Math.max(1, rates.innerHz);

			if (isCascade) {
				// Map the existing rate UI onto cascade layers. Nav is fixed at
				// 60 Hz (it represents the "human-perceptible" decision rate).
				this.stack.setRates({
					nav:      60,
					mixer:    rates.outerHz,
					attitude: rates.outerHz,
					wheels:   rates.innerHz,
				});
			}

			while (this.acc >= this.DT) {
				// --- Sensor sample (runs at sensorHz) ---
				this.dueSensor -= this.DT;
				if (this.dueSensor <= 0 || this.measured === null) {
					this.measured = this.sensors.sample(this.plant.state, params, sensorCfg, this.tSim);
					// Inject IMU bias if the disturbance panel turned it on. The
					// controller sees a tilted "upright" — its auto-trim should
					// eventually absorb a real fixed bias; a step bias exposes
					// the time constant.
					if (this.imuBiasInjected !== 0) this.measured.pitch += this.imuBiasInjected;
					// Lidar is a sensor too — runs at sensorHz alongside the IMU
					// and encoder. The Safety governor in the cascade reads it
					// from `measured.lidar`; the renderer reads it for viz.
					if (this.lidarEnabled) {
						this.measured.lidar = this.lidar.scan(this.plant.state, this.renderer.obstacles);
					}
					this.dueSensor += dtSensor;
				}

				// --- Outer attitude loop (legacy controllers only — cascade
				// is single-rate at innerHz; its layered structure already
				// separates the timescales conceptually).
				if (!isCascade) {
					this.dueOuter -= this.DT;
					if (this.dueOuter <= 0) {
						// PID biases its error by the tilt setpoint; ArduBalance uses target_angle set above.
						const measOuter = controller instanceof PIDController
							? { ...this.measured, pitch: this.measured.pitch - tiltSetpoint }
							: this.measured;
						controller.updateVelocity(measOuter, gains, dtOuter);
						this.dueOuter += dtOuter;
					}
				}

				// --- Inner motor loop (runs at innerHz) — produces PWM/force, held by ZOH ---
				this.dueInner -= this.DT;
				if (this.dueInner <= 0) {
					if (isCascade) {
						const cgains = this.ui.readCascadeGains();
						const out = this.stack.update(this.measured, cascadeCommand, cgains, this.motor, dtInner);
						this.lastForce  = out.wheelOut.force_fwd_actual;
						this.lastTauYaw = out.wheelOut.torque_yaw_actual;
						this._lastStackOut = out;

						// Record cascade-layer tuples while running. Captures both
						// Mixer and Attitude-pitch streams so a single recording
						// session feeds either layer NN's recorded-mode trainer.
						if (this.recorder.recording) {
							this.recorder.recordCascadeMixer({
								vel_lpf:      this.stack.mixer.vel_lpf,
								vel_target:   this.stack.mixer.vel_target,
								pitch_target: this.stack.mixer.pitch_target,
							});
							this.recorder.recordCascadePitch({
								pitch:        this.measured.pitch,
								pitch_rate:   this.measured.pitch_rate,
								pitch_target: this.stack.mixer.pitch_target,
								force_fwd:    this.stack.attitude.lastForceFwd,
							});
							this.recorder.recordCascadeYaw({
								heading_err: this.stack.attitude.lastHeadingErr,
								yaw_rate:    this.measured.yaw_rate,
								torque_yaw:  this.stack.attitude.lastTorqueYaw,
							});
							this.recorder.recordCascadeWheels({
								force_fwd:   this.stack.attOut.force_fwd,
								torque_yaw:  this.stack.attOut.torque_yaw,
								vel_cart:    this.measured.vel_cart,
								yaw_rate:    this.measured.yaw_rate,
								pwm_left:    out.wheelOut.pwm_left,
								pwm_right:   out.wheelOut.pwm_right,
							});
							// Nav recording only meaningful in auto mode.
							if (cascadeCommand?.mode === 'auto') {
								const dx = this.nav.target_x - this.measured.x;
								const dz = this.nav.target_z - (this.measured.z ?? 0);
								this.recorder.recordCascadeNav({
									dx, dz,
									heading:         this.measured.heading,
									vel_cart:        this.measured.vel_cart,
									vel_target_body: this.stack.navOut.vel_target_body,
									heading_err:     this.stack.nav.heading_err,
								});
							}
						}
					} else {
						const measInner = controller instanceof PIDController
							? { ...this.measured, pitch: this.measured.pitch - tiltSetpoint }
							: this.measured;
						this.lastForce = controller.produceForce(measInner, gains, dtInner, this.motor);

						// Yaw runs at the inner-loop rate too. Pilot sets the target rate;
						// YawController applies P feedback to drive measured rate to target.
						this.yawController.target_yaw_rate = yawRateSetpoint;
						this.lastTauYaw = this.yawController.update(this.measured, this.ui.readYawGains());

						// "Shadow" forward pass of the NN on the same sensor state —
						// regardless of which controller is driving. Lets us plot what
						// the NN would say alongside what the active controller said.
						if (this.controllers.nn.mlp && controller !== this.controllers.nn) {
							this.controllers.nn.vel_cart_target = this.nav.vel_desired_last ?? 0;
							this.controllers.nn.produceForce(measInner, gains, dtInner, this.motor);
						}

						// Record the (sensor → PWM) pair while ArduBalance is running.
						// Ignore PID — we're distilling the cascaded controller specifically.
						if (this.recorder.recording && controller instanceof ArduBalanceController) {
							this.recorder.record({
								pitch:           this.measured.pitch,
								pitch_rate:      this.measured.pitch_rate,
								vel_cart:        this.measured.vel_cart,
								vel_cart_target: this.nav.vel_desired_last ?? 0,
								pwm:             controller.lastPWM ?? 0,
							});
						}
					}
					this.dueInner += dtInner;
				}

				// Physics advances every DT with the last computed force held.
				this.plant.step(this.lastForce, this.lastTauYaw, this.DT);
				this.tSim += this.DT;
				// CoM computation for plotting (true state, not sensor-filtered).
				const cs = Math.cos(this.plant.state.pitch);
				const sn = Math.sin(this.plant.state.pitch);
				const x_CoM_true = this.plant.state.x + params.L * sn;
				const v_CoM_true = this.plant.state.vel_cart + params.L * cs * this.plant.state.pitch_rate;

				// "Motor PWM" = whatever the active controller just commanded.
				// Cascade has two motors — report the larger-magnitude one for a
				// scalar trace; per-wheel PWMs are also exposed below.
				const stackOut = this._lastStackOut?.wheelOut;
				const cascadePwm = stackOut
					? (Math.abs(stackOut.pwm_left) > Math.abs(stackOut.pwm_right) ? stackOut.pwm_left : stackOut.pwm_right)
					: 0;
				const activePwm = isCascade
					? cascadePwm
					: (this.controllers[this.controllerType]?.lastPWM ?? 0);

				this.history.push({
					t:				this.tSim,
					pitch:			this.plant.state.pitch,
					pitch_rate:		this.plant.state.pitch_rate,
					x:				this.plant.state.x,
					vel_cart:		this.plant.state.vel_cart,
					x_CoM:			x_CoM_true,
					v_CoM:			v_CoM_true,
					F:		 		this.lastForce,
					pwm:			activePwm,
					// NN shadow is always the NN's output, regardless of who's driving.
					pwm_nn:			this.controllers.nn.lastPWM ?? 0,
					pwm_residual:	activePwm - (this.controllers.nn.lastPWM ?? 0),
					// vel_command only exists in ArduBalance — leave 0 otherwise.
					vel_command:	this.controllers.ardubalance.vel_command ?? 0,
					vel_desired: 	this.nav.vel_desired_last ?? this.stack.mixer.vel_target ?? 0,
					err_x:		 	this.nav.err_last ?? this.stack.nav.distance_err ?? 0,
					tilt_sp:	 	isCascade ? (this.stack.mixer.pitch_target ?? 0) : tiltSetpoint,
					// Cascade-specific traces (zero in legacy modes).
					pitch_target:	this.stack.mixer.pitch_target ?? 0,
					force_fwd:		this.stack.attitude.lastForceFwd ?? 0,
					torque_yaw:		this.stack.attitude.lastTorqueYaw ?? 0,
					pwm_left:		stackOut?.pwm_left  ?? 0,
					pwm_right:		stackOut?.pwm_right ?? 0,
				});
				
				if (this.history.length > 5000) this.history.shift();
				this.acc -= this.DT;

				if (Math.abs(this.plant.state.pitch) > Math.PI / 2) {
					this.running = false;
					this.ui.log(this.tSim, `fell at t=${this.tSim.toFixed(2)}s`);
					document.getElementById('btnRun').textContent = 'Start';
					break;
				}
			}
		} else {
			this.lastT = ts;
		}

		// Render every frame regardless of sim state — keeps OrbitControls
		// drag responsive while paused, lets the joystick still show, and
		// keeps the plotter/panel-plots up-to-date with their last data.
		this.render();

		requestAnimationFrame(ts => this.tick(ts));
	}

	// Sidebar visibility: a panel is shown iff its tab matches the active
	// tab AND its data-ctrl (if any) matches the active controller type.
	// Untagged panels are always visible (e.g., the Log).
	syncSidebar() {
		const tab  = this.currentTab ?? 'control';
		const ctrl = this.controllerType;
		for (const el of document.querySelectorAll('.panel')) {
			const tabOk     = !el.dataset.tab     || el.dataset.tab === tab;
			const ctrlOk    = !el.dataset.ctrl    || el.dataset.ctrl === ctrl;
			const ctrlNotOk = !el.dataset.ctrlNot || el.dataset.ctrlNot !== ctrl;
			el.style.display = (tabOk && ctrlOk && ctrlNotOk) ? '' : 'none';
		}
		for (const btn of document.querySelectorAll('#tabBar .tab-btn')) {
			btn.classList.toggle('active', btn.dataset.tab === tab);
		}
	}

	// Back-compat alias — older call sites.
	syncControllerPanels() { this.syncSidebar(); }

	wireUI() {
		document.getElementById('btnRun').onclick = e => {
			this.running = !this.running;
			e.target.textContent = this.running ? 'Pause' : 'Start';
		};

		document.getElementById('btnReset').onclick = () => {
			this.running = false;
			document.getElementById('btnRun').textContent = 'Start';
			this.reset();
		};

		// Sidebar tab switcher (Control / Sim).
		for (const btn of document.querySelectorAll('#tabBar .tab-btn')) {
			btn.onclick = () => {
				this.currentTab = btn.dataset.tab;
				this.syncSidebar();
			};
		}

		// Pilot mode buttons (canvas overlay). Angle and FBW are clickable;
		// Auto is a status indicator that lights up only while waypoints
		// are queued, and disengages automatically when the queue empties.
		const setPilotMode = mode => {
			const sel = document.getElementById('pilotMode');
			if (sel.value !== mode) {
				sel.value = mode;
				sel.dispatchEvent(new Event('change'));
			}
			if (mode !== 'auto') this.waypoints.length = 0;   // exit cleanly
		};
		document.getElementById('btnPilotAngle').onclick = () => setPilotMode('raw');
		document.getElementById('btnPilotFbw').onclick   = () => setPilotMode('fbw');

		document.getElementById('btnPush').onclick = () => {
			const shove_rate = this.ui.num('shoveOmega');
			this.plant.state.pitch_rate += shove_rate;
			this.ui.log(this.tSim, `shove: +${shove_rate.toFixed(1)} rad/s tip`);
		};

		// Camera-follow toggle. ON (default): camera auto-recenters behind
		// the bot after idle. OFF: camera holds whatever pose you orbit to,
		// pans with the bot, never rotates around.
		const btnCamFollow = document.getElementById('btnCamFollow');
		btnCamFollow.onclick = () => {
			const enabled = !btnCamFollow.classList.contains('active');
			btnCamFollow.classList.toggle('active', enabled);
			this.renderer.setAutoFollow(enabled);
			this.ui.log(this.tSim, `camera follow: ${enabled ? 'on' : 'off'}`);
		};

		const btnLidar = document.getElementById('btnLidar');
		btnLidar.onclick = () => {
			this.lidarEnabled = !btnLidar.classList.contains('active');
			btnLidar.classList.toggle('active', this.lidarEnabled);
			this.ui.log(this.tSim, `lidar: ${this.lidarEnabled ? 'on' : 'off'}`);
		};

		// Disturbances — instantaneous state kicks, plus a toggleable IMU bias.
		// Force impulse → Δv = J / (M+m). Yaw impulse → Δω = J / I_yaw.
		const pushChassis = sign => {
			const J = sign * this.ui.num('disturbForceImpulse');
			const m_total = this.plant.params.M + this.plant.params.m;
			this.plant.state.vel_cart += J / m_total;
			this.ui.log(this.tSim, `chassis push: ${J.toFixed(1)} N·s → Δv=${(J / m_total).toFixed(2)} m/s`);
		};
		const yawKick = sign => {
			const J = sign * this.ui.num('disturbTauImpulse');
			this.plant.state.yaw_rate += J / this.plant.params.I_yaw;
			this.ui.log(this.tSim, `yaw kick: ${J.toFixed(2)} N·m·s`);
		};
		document.getElementById('btnDisturbForward').onclick = () => pushChassis(+1);
		document.getElementById('btnDisturbBack').onclick    = () => pushChassis(-1);
		document.getElementById('btnDisturbYawL').onclick    = () => yawKick(+1);
		document.getElementById('btnDisturbYawR').onclick    = () => yawKick(-1);

		const biasBtn = document.getElementById('btnDisturbBiasToggle');
		biasBtn.onclick = () => {
			if (this.imuBiasInjected !== 0) {
				this.imuBiasInjected = 0;
				biasBtn.textContent = 'Bias OFF';
				this.ui.log(this.tSim, 'IMU bias cleared');
			} else {
				this.imuBiasInjected = this.ui.num('disturbImuBias');
				biasBtn.textContent = `Bias ON (+${(this.imuBiasInjected * 180 / Math.PI).toFixed(1)}°)`;
				this.ui.log(this.tSim, `IMU bias on: +${(this.imuBiasInjected * 180 / Math.PI).toFixed(1)}°`);
			}
		};

		// Recorder UI
		const recBtn	 = document.getElementById('btnRecord');
		const recStats = document.getElementById('recStats');
		const refreshRecStats = () => {
			const ardu  = this.recorder.count('ardubalance');
			const mixer = this.recorder.count('cascade_mixer');
			const pitch = this.recorder.count('cascade_pitch');
			const parts = [];
			if (ardu)  parts.push(`${ardu} ardu`);
			if (mixer) parts.push(`${mixer} mixer`);
			if (pitch) parts.push(`${pitch} pitch`);
			const detail = parts.length ? `  (${parts.join(', ')})` : '';
			recStats.textContent = `${this.recorder.size()} samples${detail}` +
				(this.recorder.recording ? '  [recording…]' : '');
		};
		recBtn.onclick = () => {
			if (this.recorder.recording) {
				this.recorder.stop();
				recBtn.textContent = 'Record';
				this.ui.log(this.tSim, `recording stopped: ${this.recorder.size()} samples`);
			} else {
				this.recorder.start();
				recBtn.textContent = 'Stop';
				this.ui.log(this.tSim, 'recording started');
			}
			refreshRecStats();
		};
		document.getElementById('btnClearRec').onclick = () => {
			this.recorder.clear();
			this.ui.log(this.tSim, 'recording cleared');
			refreshRecStats();
		};
		document.getElementById('btnSaveRec').onclick = () => {
			if (this.recorder.size() === 0) return this.ui.log(this.tSim, 'no samples to save');
			this.recorder.download();
			this.ui.log(this.tSim, `saved ${this.recorder.size()} samples`);
		};
		// Update the stats line periodically while recording.
		setInterval(refreshRecStats, 250);

		document.getElementById('btnTrainNN').onclick = () => this.trainNN();
		document.getElementById('btnTrainPitchNN').onclick = () => this.trainAttitudePitchNN();
		document.getElementById('pitch_mode').onchange = e => {
			const mode = e.target.value;
			const mlp = this.attitudePitchMlp ?? null;
			if (mode === 'nn' && !mlp) {
				this.ui.log(this.tSim, 'no trained pitch NN yet — staying on rule');
				e.target.value = 'rule';
				return;
			}
			this.stack.attitude.setPitchMode(mode, mlp);
			this.ui.log(this.tSim, `pitch: ${mode}`);
		};

		document.getElementById('btnTrainMixerNN').onclick = () => this.trainMixerNN();
		document.getElementById('mixer_mode').onchange = e => {
			const mode = e.target.value;
			const mlp = this.mixerMlp ?? null;
			if (mode === 'nn' && !mlp) {
				this.ui.log(this.tSim, 'no trained mixer NN yet — staying on rule');
				e.target.value = 'rule';
				return;
			}
			this.stack.mixer.setMode(mode, mlp);
			this.ui.log(this.tSim, `mixer: ${mode}`);
		};

		document.getElementById('btnTrainYawNN').onclick = () => this.trainAttitudeYawNN();
		document.getElementById('yaw_mode').onchange = e => {
			const mode = e.target.value;
			const mlp = this.attitudeYawMlp ?? null;
			if (mode === 'nn' && !mlp) {
				this.ui.log(this.tSim, 'no trained yaw NN yet — staying on rule');
				e.target.value = 'rule';
				return;
			}
			this.stack.attitude.setYawMode(mode, mlp);
			this.ui.log(this.tSim, `yaw: ${mode}`);
		};

		document.getElementById('btnTrainWheelsNN').onclick = () => this.trainWheelsNN();
		document.getElementById('wheels_mode').onchange = e => {
			const mode = e.target.value;
			const mlp = this.wheelsMlp ?? null;
			if (mode === 'nn' && !mlp) {
				this.ui.log(this.tSim, 'no trained wheels NN yet — staying on rule');
				e.target.value = 'rule';
				return;
			}
			this.stack.wheels.setMode(mode, mlp);
			this.ui.log(this.tSim, `wheels: ${mode}`);
		};

		document.getElementById('btnTrainNavNN').onclick = () => this.trainNavNN();

		// Planner mode (Direct vs A*) — applies on the next shift+click.
		document.getElementById('plannerMode').onchange = e => {
			this.planner.setMode(e.target.value);
			this.ui.log(this.tSim, `planner: ${e.target.value}`);
		};
		document.getElementById('nav_mode_nn').onchange = e => {
			const mode = e.target.value;
			const mlp = this.navMlp ?? null;
			if (mode === 'nn' && !mlp) {
				this.ui.log(this.tSim, 'no trained nav NN yet — staying on rule');
				e.target.value = 'rule';
				return;
			}
			this.stack.nav.setAutoMode(mode, mlp);
			this.ui.log(this.tSim, `nav (auto): ${mode}`);
		};

		document.getElementById('btnCalibrate').onclick = () => this.calibrateMotor();
		document.getElementById('btnClearLUT').onclick = () => {
			this.controllers.ardubalance.pwmTable.clear();
			this.ui.log(this.tSim, 'PWM LUT cleared → linear fallback');
			this.drawLUT();
		};

		document.getElementById('ctrlType').onchange = e => {
			this.controllerType = e.target.value;
			this.syncControllerPanels();
			this.ui.log(this.tSim, `controller: ${this.controllerType}`);
			this.reset();
		};

		// Nav mode dropdown — show the matching gain block, push mode to controller.
		const navModeEl = document.getElementById('navMode');
		const syncNavMode = () => {
			this.nav.mode = navModeEl.value;
			for (const el of document.querySelectorAll('[data-navmode]')) {
				el.style.display = el.dataset.navmode === navModeEl.value ? '' : 'none';
			}
		};
		navModeEl.addEventListener('change', syncNavMode);
		syncNavMode();

		// Nav target — number inputs and the controller share state.
		const navTxInput = document.getElementById('navTargetX');
		const navTzInput = document.getElementById('navTargetZ');
		const syncTargetFromInputs = () => {
			this.nav.target_x = +navTxInput.value;
			this.nav.target_z = +(navTzInput?.value ?? 0);
		};
		navTxInput.addEventListener('input', syncTargetFromInputs);
		if (navTzInput) navTzInput.addEventListener('input', syncTargetFromInputs);
		syncTargetFromInputs();

		// Shift+click on the world canvas drops a waypoint flag and sets the
		// nav target. Plain clicks pass through to OrbitControls so the
		// camera still drags freely.
		const worldEl = document.getElementById('world');
		worldEl.addEventListener('click', e => {
			if (!e.shiftKey) return;
			const hit = this.renderer.screenToGround(e.clientX, e.clientY);
			if (!hit) return;

			// Run the planner. In direct mode the path is just [hit]; in A*
			// mode the planner routes around walls. Either way, the result
			// becomes the waypoint queue, so the existing arrival-and-advance
			// logic handles multi-waypoint paths without changes.
			//
			// Plan FROM the end of the existing queue (or the bot if empty).
			// Otherwise multi-click extends the path back through the bot's
			// current position, making the bot backtrack between clicks.
			const start = this.waypoints.length > 0
				? this.waypoints[this.waypoints.length - 1]
				: (this.measured ?? this.plant.state);
			const path  = this.planner.plan(
				{ x: start.x, z: start.z ?? 0 }, hit,
				{ obstacles: this.renderer.obstacles, res: 0.25, pad: 0.25 },
			);
			// First segment of A* output is the start point itself; drop it
			// when extending so we don't queue a redundant "go to where I
			// already am" hop.
			const segment = (this.waypoints.length > 0 && path.length > 1) ? path.slice(1) : path;
			this.waypoints.push(...segment);

			const head = this.waypoints[0];
			this.nav.target_x = head.x;
			this.nav.target_z = head.z;
			navTxInput.value = head.x.toFixed(2);
			if (navTzInput) navTzInput.value = head.z.toFixed(2);
			document.getElementById('navEnabled').checked = true;
			document.getElementById('pilotMode').value = 'auto';
			this.ui.log(this.tSim,
				`WP via ${this.planner.mode}: (${hit.x.toFixed(2)}, ${hit.z.toFixed(2)})  · path=${path.length}, queue=${this.waypoints.length}`);
		});

		for (const btn of document.querySelectorAll('[data-preset]')) {
			btn.onclick = () => {
				const p = PRESETS[btn.dataset.preset];
				this.ui.writeAll(p);
				this.ui.log(this.tSim, `preset: ${btn.dataset.preset}`);
				this.running = false;
				document.getElementById('btnRun').textContent = 'Start';
				this.reset();
			};
		}

		document.getElementById('btnSaveTuning').onclick = () => this.saveTuning();
		document.getElementById('btnExportTuning').onclick = () => this.exportTuning();
		document.getElementById('btnImportTuning').onclick = () => {
			document.getElementById('importFile').click();
		};
		document.getElementById('importFile').onchange = e => {
			const file = e.target.files[0];
			if (file) this.importTuning(file);
			e.target.value = '';
		};
		this.renderSavedTunings();
	}

	wireKeys() {
		const typing = () => {
			const a = document.activeElement;
			return a && (a.tagName === 'INPUT' || a.tagName === 'SELECT' || a.tagName === 'TEXTAREA');
		};

		const pressed = new Set();
		const updatePilot = () => {
			// Up/Down → forward/back drive (pitch lean toward direction of travel).

			let tilt = 0;
			if (pressed.has('ArrowUp'))		tilt += this.pilotTiltMax;
			if (pressed.has('ArrowDown'))	tilt -= this.pilotTiltMax;
			this.pilotTilt = tilt;

			// Left/Right → yaw rate command.
			let rate = 0;
			if (pressed.has('ArrowLeft'))	rate += this.pilotYawRateMax;
			if (pressed.has('ArrowRight')) 	rate -= this.pilotYawRateMax;
			this.pilotYawRate = rate;
		};

		window.addEventListener('keydown', e => {
			if (typing()) return;
			if (e.repeat) return;
			switch (e.key) {
				case 'ArrowLeft':
				case 'ArrowRight':
				case 'ArrowUp':
				case 'ArrowDown':
					pressed.add(e.key); updatePilot(); e.preventDefault(); break;
				case ' ':
					this.running = !this.running;
					document.getElementById('btnRun').textContent = this.running ? 'Pause' : 'Start';
					e.preventDefault(); break;
			}
		});

		window.addEventListener('keyup', e => {
			if (pressed.delete(e.key)) updatePilot();
		});

		// Release held keys on focus loss.
		window.addEventListener('blur', () => { pressed.clear(); updatePilot(); });
	}

	calibrateMotor() {
		const params = this.ui.readParams();
		const motor	= new Motor(this.ui.readMotor());
		const PWM_max = motor.PWM_max;
		// Sample a good spread of PWM values, denser near zero where the
		// deadband nonlinearity lives.
		const steps = [0, 60, 100, 140, 200, 280, 400, 600, 900, 1300, 1700, PWM_max];
		const results = this.calibrator.run({
			pwmSteps: steps, settleSec: 1.2, sampleSec: 0.4, params, motor,
		});
		this.controllers.ardubalance.pwmTable.setFromCalibration(
			results.map(r => ({ pwm: r.pwm, speed: r.speed })),
		);
		this.ui.log(this.tSim,
			`calibrated: ${results.length} points, top=${results.at(-1).speed.toFixed(2)} m/s @ PWM ${PWM_max}`);
		for (const r of results) {
			this.ui.log(this.tSim, `	PWM=${r.pwm.toString().padStart(4)} → ${r.speed.toFixed(3)} m/s`);
		}
		this.drawLUT();
	}

	drawLUT() {
		const cv = document.getElementById('lutPreview');
		if (!cv) return;
		const ctx = cv.getContext('2d');
		const W = cv.width, H = cv.height;
		ctx.clearRect(0, 0, W, H);
		ctx.fillStyle = '#0a0a0a'; ctx.fillRect(0, 0, W, H);
		ctx.strokeStyle = '#222';
		ctx.beginPath(); ctx.moveTo(0, H); ctx.lineTo(W, H); ctx.stroke();

		const table = this.controllers.ardubalance.pwmTable;
		const maxSpeed = 4; // m/s axis
		const PWM_max = this.motor.PWM_max || 2000;

		// Linear fallback reference (dashed)
		ctx.strokeStyle = '#333'; ctx.setLineDash([3, 3]); ctx.beginPath();
		for (let px = 0; px <= W; px += 4) {
			const s = (px / W) * maxSpeed;
			const pwm = (+document.getElementById('ff_per_mps').value) * s;
			const py = H - (Math.min(pwm, PWM_max) / PWM_max) * H;
			if (px === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
		}
		ctx.stroke(); ctx.setLineDash([]);

		// Calibrated table (solid)
		if (table.isCalibrated) {
			ctx.strokeStyle = '#6cf'; ctx.lineWidth = 1.5;
			ctx.beginPath();
			for (const p of table.points) {
				const px = (p.speed / maxSpeed) * W;
				const py = H - (Math.min(p.pwm, PWM_max) / PWM_max) * H;
				if (p === table.points[0]) ctx.moveTo(px, py); else ctx.lineTo(px, py);
			}
			ctx.stroke();
			ctx.fillStyle = '#6cf';
			for (const p of table.points) {
				const px = (p.speed / maxSpeed) * W;
				const py = H - (Math.min(p.pwm, PWM_max) / PWM_max) * H;
				ctx.beginPath(); ctx.arc(px, py, 2, 0, Math.PI * 2); ctx.fill();
			}
		}
		ctx.fillStyle = '#666'; ctx.font = '10px ui-monospace';
		ctx.fillText('PWM', 2, 10);
		ctx.fillText(`${maxSpeed} m/s`, W - 44, H - 2);
	}

	// ---- NN training -------------------------------------------------------
	async trainNN() {
		const mode		= document.getElementById('nnMode').value;
		const hidden	= +document.getElementById('nnHidden').value;
		const epochs	= +document.getElementById('nnEpochs').value;
		const samples 	= +document.getElementById('nnSamples').value;
		const lr			= +document.getElementById('nnLR').value;
		const nnStats = document.getElementById('nnStats');

		const arduCount = this.recorder.count('ardubalance');
		if (mode === 'recorded' && arduCount < 50) {
			this.ui.log(this.tSim, `need more recorded ArduBalance samples (have ${arduCount}, want ≥50)`);
			return;
		}

		const mlp = new MLP(5, hidden, 1);
		const gains = this.ui.readArduGains();
		const navGains = this.ui.readNavGains();
		const dtInner = 1 / Math.max(1, this.ui.readRates().innerHz);
		const srcDesc = mode === 'random'
			? `${samples} random samples/epoch`
			: `${this.recorder.data.length} recorded samples`;
		nnStats.textContent = `training: 0/${epochs}, ${srcDesc}, ${mlp.paramCount()} params`;
		this.ui.log(this.tSim, `training NN (${mode}, ${hidden} hidden, ${mlp.paramCount()} params)`);

		const lossHistory = [];
		const t0 = performance.now();
		await this.nnTrainer.train({
			mlp,
			mode,
			data: this.recorder.data,
			gains,
			navGains,
			motor: this.motor,
			dt: dtInner,
			epochs,
			samplesPerEpoch: samples,
			lr,
			momentum: 0.9,
			onProgress: (e, loss) => {
				lossHistory.push(loss);
				nnStats.textContent = `epoch ${e + 1}/${epochs}	loss=${loss.toExponential(3)}`;
				this.drawLossPlot(lossHistory);
			},
		});
		const dt = (performance.now() - t0) / 1000;

		this.controllers.nn.mlp = mlp;
		const finalLoss = lossHistory.at(-1);
		nnStats.textContent = `done: loss=${finalLoss.toExponential(3)}	(${dt.toFixed(1)}s)`;
		this.ui.log(this.tSim, `NN trained: final loss=${finalLoss.toExponential(3)} in ${dt.toFixed(1)}s`);
	}

	// Train a small MLP to imitate the rule-based mixer (vel → tilt). Two
	// inputs, one output — converges quickly and shows the cleanest
	// example of distillation in the project.
	async trainMixerNN() {
		const hidden  = +document.getElementById('nnHidden').value;
		const epochs  = Math.max(50, +document.getElementById('nnEpochs').value);
		const samples = +document.getElementById('nnSamples').value;
		const lr      = +document.getElementById('nnLR').value;
		const stats   = document.getElementById('mixerNNStats');

		const mode = document.getElementById('nnMode').value;
		const recordedCount = this.recorder.count('cascade_mixer');
		if (mode === 'recorded' && recordedCount < 50) {
			this.ui.log(this.tSim, `need more recorded cascade_mixer samples (have ${recordedCount}, want ≥50)`);
			return;
		}

		const mlp = new MLP(2, hidden, 1);
		const mixerGains = this.ui.readCascadeGains().mixer;
		const srcDesc = mode === 'random' ? `${samples} random/epoch` : `${recordedCount} recorded`;
		stats.textContent = `training (${mode}): 0/${epochs}, ${srcDesc}, ${mlp.paramCount()} params`;
		this.ui.log(this.tSim, `training mixer NN (${mode}, ${hidden} hidden, ${mlp.paramCount()} params)`);

		const lossHistory = [];
		const t0 = performance.now();
		await this.nnTrainer.trainMixer({
			mlp, mixerGains,
			mode, data: this.recorder.data,
			epochs, samplesPerEpoch: samples, lr, momentum: 0.9,
			onProgress: (e, loss) => {
				lossHistory.push(loss);
				stats.textContent = `epoch ${e + 1}/${epochs}	loss=${loss.toExponential(3)}`;
				this.drawLossPlot(lossHistory);
			},
		});
		const dt = (performance.now() - t0) / 1000;

		this.mixerMlp = mlp;
		this.stack.mixer.setMode('nn', mlp);
		document.getElementById('mixer_mode').value = 'nn';
		const finalLoss = lossHistory.at(-1);
		stats.textContent = `done: loss=${finalLoss.toExponential(3)}	(${dt.toFixed(1)}s) — using NN`;
		this.ui.log(this.tSim, `mixer NN trained: final loss=${finalLoss.toExponential(3)} in ${dt.toFixed(1)}s, switched to NN`);
	}

	async trainNavNN() {
		const hidden  = +document.getElementById('nnHidden').value;
		const epochs  = Math.max(50, +document.getElementById('nnEpochs').value);
		const samples = +document.getElementById('nnSamples').value;
		const lr      = +document.getElementById('nnLR').value;
		const stats   = document.getElementById('navNNStats');

		const mode = document.getElementById('nnMode').value;
		const recordedCount = this.recorder.count('cascade_nav');
		if (mode === 'recorded' && recordedCount < 50) {
			this.ui.log(this.tSim, `need more recorded cascade_nav samples (have ${recordedCount}, want ≥50)`);
			return;
		}

		const mlp = new MLP(4, hidden, 2);
		const navGains = this.ui.readCascadeGains().nav;
		const srcDesc = mode === 'random' ? `${samples} random/epoch` : `${recordedCount} recorded`;
		stats.textContent = `training (${mode}): 0/${epochs}, ${srcDesc}, ${mlp.paramCount()} params`;
		this.ui.log(this.tSim, `training nav NN (${mode}, ${hidden} hidden, ${mlp.paramCount()} params)`);

		const lossHistory = [];
		const t0 = performance.now();
		await this.nnTrainer.trainNav({
			mlp, navGains,
			mode, data: this.recorder.data,
			epochs, samplesPerEpoch: samples, lr, momentum: 0.9,
			onProgress: (e, loss) => {
				lossHistory.push(loss);
				stats.textContent = `epoch ${e + 1}/${epochs}	loss=${loss.toExponential(3)}`;
				this.drawLossPlot(lossHistory);
			},
		});
		const dt = (performance.now() - t0) / 1000;

		this.navMlp = mlp;
		this.stack.nav.setAutoMode('nn', mlp);
		document.getElementById('nav_mode_nn').value = 'nn';
		const finalLoss = lossHistory.at(-1);
		stats.textContent = `done: loss=${finalLoss.toExponential(3)}	(${dt.toFixed(1)}s) — using NN`;
		this.ui.log(this.tSim, `nav NN trained: final loss=${finalLoss.toExponential(3)} in ${dt.toFixed(1)}s, switched to NN`);
	}

	async trainWheelsNN() {
		const hidden  = +document.getElementById('nnHidden').value;
		const epochs  = Math.max(50, +document.getElementById('nnEpochs').value);
		const samples = +document.getElementById('nnSamples').value;
		const lr      = +document.getElementById('nnLR').value;
		const stats   = document.getElementById('wheelsNNStats');

		const mode = document.getElementById('nnMode').value;
		const recordedCount = this.recorder.count('cascade_wheels');
		if (mode === 'recorded' && recordedCount < 50) {
			this.ui.log(this.tSim, `need more recorded cascade_wheels samples (have ${recordedCount}, want ≥50)`);
			return;
		}

		const mlp = new MLP(4, hidden, 2);
		const wheelGains = this.ui.readCascadeGains().wheels;
		const srcDesc = mode === 'random' ? `${samples} random/epoch` : `${recordedCount} recorded`;
		stats.textContent = `training (${mode}): 0/${epochs}, ${srcDesc}, ${mlp.paramCount()} params`;
		this.ui.log(this.tSim, `training wheels NN (${mode}, ${hidden} hidden, ${mlp.paramCount()} params)`);

		const lossHistory = [];
		const t0 = performance.now();
		await this.nnTrainer.trainWheels({
			mlp, wheelGains, motor: this.motor,
			mode, data: this.recorder.data,
			epochs, samplesPerEpoch: samples, lr, momentum: 0.9,
			onProgress: (e, loss) => {
				lossHistory.push(loss);
				stats.textContent = `epoch ${e + 1}/${epochs}	loss=${loss.toExponential(3)}`;
				this.drawLossPlot(lossHistory);
			},
		});
		const dt = (performance.now() - t0) / 1000;

		this.wheelsMlp = mlp;
		this.stack.wheels.setMode('nn', mlp);
		document.getElementById('wheels_mode').value = 'nn';
		const finalLoss = lossHistory.at(-1);
		stats.textContent = `done: loss=${finalLoss.toExponential(3)}	(${dt.toFixed(1)}s) — using NN (FF only)`;
		this.ui.log(this.tSim, `wheels NN trained: final loss=${finalLoss.toExponential(3)} in ${dt.toFixed(1)}s, switched to NN`);
	}

	async trainAttitudeYawNN() {
		const hidden  = +document.getElementById('nnHidden').value;
		const epochs  = Math.max(50, +document.getElementById('nnEpochs').value);
		const samples = +document.getElementById('nnSamples').value;
		const lr      = +document.getElementById('nnLR').value;
		const stats   = document.getElementById('yawNNStats');

		const mode = document.getElementById('nnMode').value;
		const recordedCount = this.recorder.count('cascade_yaw');
		if (mode === 'recorded' && recordedCount < 50) {
			this.ui.log(this.tSim, `need more recorded cascade_yaw samples (have ${recordedCount}, want ≥50)`);
			return;
		}

		const mlp = new MLP(2, hidden, 1);
		const attGains = this.ui.readCascadeGains().attitude;
		const srcDesc = mode === 'random' ? `${samples} random/epoch` : `${recordedCount} recorded`;
		stats.textContent = `training (${mode}): 0/${epochs}, ${srcDesc}, ${mlp.paramCount()} params`;
		this.ui.log(this.tSim, `training yaw NN (${mode}, ${hidden} hidden, ${mlp.paramCount()} params)`);

		const lossHistory = [];
		const t0 = performance.now();
		await this.nnTrainer.trainAttitudeYaw({
			mlp, attGains,
			mode, data: this.recorder.data,
			epochs, samplesPerEpoch: samples, lr, momentum: 0.9,
			onProgress: (e, loss) => {
				lossHistory.push(loss);
				stats.textContent = `epoch ${e + 1}/${epochs}	loss=${loss.toExponential(3)}`;
				this.drawLossPlot(lossHistory);
			},
		});
		const dt = (performance.now() - t0) / 1000;

		this.attitudeYawMlp = mlp;
		this.stack.attitude.setYawMode('nn', mlp);
		document.getElementById('yaw_mode').value = 'nn';
		const finalLoss = lossHistory.at(-1);
		stats.textContent = `done: loss=${finalLoss.toExponential(3)}	(${dt.toFixed(1)}s) — using NN`;
		this.ui.log(this.tSim, `yaw NN trained: final loss=${finalLoss.toExponential(3)} in ${dt.toFixed(1)}s, switched to NN`);
	}

	// Train a small MLP to imitate the rule-based pitch of Attitude.
	// Reuses the NN-training UI's hidden-units / epochs / samples / LR
	// fields so a student can vary network capacity and watch loss vs
	// fidelity. On success, the trained MLP is auto-installed and the
	// pitch select flips to 'nn'.
	async trainAttitudePitchNN() {
		const hidden  = +document.getElementById('nnHidden').value;
		const epochs  = Math.max(50, +document.getElementById('nnEpochs').value);
		const samples = +document.getElementById('nnSamples').value;
		const lr      = +document.getElementById('nnLR').value;
		const stats   = document.getElementById('pitchNNStats');

		const mode = document.getElementById('nnMode').value;
		const recordedCount = this.recorder.count('cascade_pitch');
		if (mode === 'recorded' && recordedCount < 50) {
			this.ui.log(this.tSim, `need more recorded cascade_pitch samples (have ${recordedCount}, want ≥50)`);
			return;
		}

		const mlp = new MLP(3, hidden, 1);
		const attGains = this.ui.readCascadeGains().attitude;
		const srcDesc = mode === 'random' ? `${samples} random/epoch` : `${recordedCount} recorded`;
		stats.textContent = `training (${mode}): 0/${epochs}, ${srcDesc}, ${mlp.paramCount()} params`;
		this.ui.log(this.tSim, `training pitch NN (${mode}, ${hidden} hidden, ${mlp.paramCount()} params)`);

		const lossHistory = [];
		const t0 = performance.now();
		await this.nnTrainer.trainAttitudePitch({
			mlp, attGains,
			mode, data: this.recorder.data,
			epochs, samplesPerEpoch: samples, lr, momentum: 0.9,
			onProgress: (e, loss) => {
				lossHistory.push(loss);
				stats.textContent = `epoch ${e + 1}/${epochs}	loss=${loss.toExponential(3)}`;
				this.drawLossPlot(lossHistory);
			},
		});
		const dt = (performance.now() - t0) / 1000;

		this.attitudePitchMlp = mlp;
		this.stack.attitude.setPitchMode('nn', mlp);
		document.getElementById('pitch_mode').value = 'nn';
		const finalLoss = lossHistory.at(-1);
		stats.textContent = `done: loss=${finalLoss.toExponential(3)}	(${dt.toFixed(1)}s) — using NN`;
		this.ui.log(this.tSim, `pitch NN trained: final loss=${finalLoss.toExponential(3)} in ${dt.toFixed(1)}s, switched to NN`);
	}

	drawLossPlot(losses) {
		const cv = document.getElementById('lossPlot');
		const ctx = cv.getContext('2d');
		const W = cv.width, H = cv.height;
		ctx.clearRect(0, 0, W, H);
		ctx.fillStyle = '#0a0a0a'; ctx.fillRect(0, 0, W, H);
		if (losses.length < 2) return;
		// Log scale on Y — loss usually spans orders of magnitude.
		const logs = losses.map(v => Math.log10(Math.max(v, 1e-12)));
		const mn = Math.min(...logs), mx = Math.max(...logs);
		const span = (mx - mn) || 1;
		ctx.strokeStyle = '#6cf'; ctx.lineWidth = 1.2;
		ctx.beginPath();
		for (let i = 0; i < logs.length; i++) {
			const x = (i / (losses.length - 1)) * W;
			const y = H - ((logs[i] - mn) / span) * (H - 4) - 2;
			if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
		}
		ctx.stroke();
		ctx.fillStyle = '#666'; ctx.font = '10px ui-monospace';
		ctx.fillText(`loss (log) · ${losses.length} epochs`, 4, 10);
	}

	// ---- Tuning persistence ------------------------------------------------
	// Stored in localStorage under a single JSON blob keyed by name.
	_loadStore() {
		try { return JSON.parse(localStorage.getItem('roller-tunings') || '{}'); }
		catch { return {}; }
	}
	_saveStore(s) { localStorage.setItem('roller-tunings', JSON.stringify(s)); }

	saveTuning() {
		const name = prompt('Name this tuning:');
		if (!name) return;
		const store = this._loadStore();
		store[name] = this.ui.readAll();
		this._saveStore(store);
		this.ui.log(this.tSim, `saved tuning "${name}"`);
		this.renderSavedTunings();
	}

	loadTuning(name) {
		const store = this._loadStore();
		const t = store[name];
		if (!t) return;
		this.ui.writeAll(t);
		this.ui.log(this.tSim, `loaded tuning "${name}"`);
		this.running = false;
		document.getElementById('btnRun').textContent = 'Start';
		this.reset();
	}

	deleteTuning(name) {
		const store = this._loadStore();
		delete store[name];
		this._saveStore(store);
		this.renderSavedTunings();
	}

	renderSavedTunings() {
		const store = this._loadStore();
		const host = document.getElementById('savedTunings');
		host.innerHTML = '';
		const names = Object.keys(store).sort();
		if (names.length === 0) {
			host.innerHTML = '<span style="color:#666">(none saved)</span>';
			return;
		}
		for (const name of names) {
			const row = document.createElement('div');
			row.style.cssText = 'display:flex; justify-content:space-between; padding:2px 0';
			const load = document.createElement('a');
			load.href = '#'; load.textContent = name;
			load.style.cssText = 'color:#8ab; text-decoration:none';
			load.onclick = e => { e.preventDefault(); this.loadTuning(name); };
			const del = document.createElement('a');
			del.href = '#'; del.textContent = '×';
			del.style.cssText = 'color:#a44; text-decoration:none; padding:0 6px';
			del.onclick = e => {
				e.preventDefault();
				if (confirm(`Delete tuning "${name}"?`)) this.deleteTuning(name);
			};
			row.appendChild(load); row.appendChild(del);
			host.appendChild(row);
		}
	}

	exportTuning() {
		const tuning = this.ui.readAll();
		const blob = new Blob([JSON.stringify(tuning, null, 2)], { type: 'application/json' });
		const url = URL.createObjectURL(blob);
		const a = document.createElement('a');
		a.href = url;
		a.download = `roller-tuning-${new Date().toISOString().slice(0, 19).replace(/[:T]/g, '-')}.json`;
		a.click();
		URL.revokeObjectURL(url);
		this.ui.log(this.tSim, 'exported tuning');
	}

	async importTuning(file) {
		try {
			const text = await file.text();
			const tuning = JSON.parse(text);
			this.ui.writeAll(tuning);
			this.ui.log(this.tSim, `imported "${file.name}"`);
			this.running = false;
			document.getElementById('btnRun').textContent = 'Start';
			this.reset();
		} catch (err) {
			this.ui.log(this.tSim, `import failed: ${err.message}`);
		}
	}

}
