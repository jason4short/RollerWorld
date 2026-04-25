import { Pendulum }							from './physics/pendulum.js';
import { Motor }								 from './physics/motor.js';
import { Sensors }							 from './physics/sensors.js';
import { PIDController }				 from './controllers/pid.js';
import { ArduBalanceController } from './controllers/ardubalance.js';
import { NavController }				 from './controllers/nav.js';
import { YawController }				 from './controllers/yaw.js';
import { NNController }					from './controllers/nn.js';
import { MLP }									 from './nn/mlp.js';
import { NNTrainer }						 from './nn/trainer.js';
import { WorldRenderer3D }			 from './render/world-renderer-3d.js';
import { Plotter, PLOT_SIGNALS } from './render/plotter.js';
import { UI }										from './ui.js';
import { ExperimentRunner }			from './experiment.js';
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
		this.calibrator 		= new MotorCalibrator({ dt: this.DT });
		const joyEl 			= document.getElementById('joystick');
		this.joystick 			= joyEl ? new Joystick(joyEl) : null;
		this.recorder 			= new Recorder();
		this.renderer 			= new WorldRenderer3D(document.getElementById('world'));
		this.plotter			= new Plotter(document.getElementById('plot'));
		this.experiment 		= new ExperimentRunner(this.DT);

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
		this.pilotYawRateMax 	= 1.5; // rad/s (~85°/s)
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
		return this.controllerType === 'ardubalance'
			? this.ui.readArduGains()
			: this.ui.readGains();
	}

	reset() {
		const { th0 } = this.ui.readInit();
		this.plant.params = this.ui.readParams();
		this.plant.setState({
			x: 0, z: 0, v: 0,
			pitch: th0 * Math.PI / 180, pitch_rate: 0,
			heading: 0, yaw_rate: 0,
		});
		this.lastTauYaw = 0;
		this.pilotYawRate = 0;
		this.waypoints.length = 0;
		for (const c of Object.values(this.controllers)) c.reset();
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
		this.renderer.draw(this.plant.state, this.plant.params, navTarget, queueRest);
		const fRef = this.controllerType === 'ardubalance'
			? this.motor.Km
			: this.ui.readGains().Fmax;
		const pwmRef = this.motor.PWM_max || 2000;
		const series = this.buildPlotSeries(fRef, pwmRef);
		this.plotter.draw(this.history, series);
		this.ui.setStats(this.tSim, this.plant.state, this.pilotTilt);
	}

	populatePlotMenus() {
		const keys = Object.keys(PLOT_SIGNALS);
		const defaults = ['pitch', 'v', 'v_desired'];	 // sensible for nav debugging
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

			// Nav outer-outer loop: if enabled, position error drives the tilt
			// setpoint. Otherwise the pilot (arrow keys) drives it directly.
			const navOn = this.ui.navEnabled();
			
			// Nav runs once per animation frame (~60 Hz). Approximate its dt from
			// wall-clock elapsed so its internal LPF is rate-correct regardless
			// of browser frame pacing.
			const navDt = Math.max(0.001, Math.min(0.1, (ts - (this._lastNavTs || ts)) / 1000));
			this._lastNavTs = ts;
			
			
			// Input priority: FBW pilot (joystick) > nav waypoint > raw arrow tilt.
			// FBW reuses nav.js's v_lpf/Kvel braking math but takes its v_desired
			// directly from the stick, so centering the stick brakes hard.
			const pilotMode = this.ui.readPilotMode();
			let tiltSetpoint, yawRateSetpoint;


			if (pilotMode === 'fbw' && this.joystick) {
				const s		= this.joystick.value();
				// Screen-up = forward, screen-right = turn right (negative
				// yaw_rate, matching ArrowRight's sign convention).
				const stick	= { fwd: s.y, yaw: -s.x };
				const out	= this.nav.updateFbw(this.measured || this.plant.state,
					stick, this.ui.readNavGains(), navDt);
				tiltSetpoint	 = out.tilt;
				yawRateSetpoint  = out.yaw_rate;

			} else if (pilotMode === 'auto') {
				const out = this.nav.update(this.measured || this.plant.state, this.ui.readNavGains(), navDt);
				tiltSetpoint	 = out.tilt;
				yawRateSetpoint  = out.yaw_rate;
				this._advanceWaypointIfArrived();

			} else {
				tiltSetpoint	 = this.pilotTilt;
				yawRateSetpoint  = this.pilotYawRate;
			}

			
			if (controller instanceof ArduBalanceController) {
				controller.target_angle = tiltSetpoint;
			} else if (controller instanceof NNController) {
				// NN swallows the velocity-tracking step; it takes vel_cart_target
				// (post-slew) directly from nav rather than a tilt setpoint.
				controller.vel_cart_target = this.nav.v_desired_last ?? 0;
			}

			const dtSensor = 1 / Math.max(1, rates.sensorHz);
			const dtOuter	= 1 / Math.max(1, rates.outerHz);
			const dtInner	= 1 / Math.max(1, rates.innerHz);

			while (this.acc >= this.DT) {
				// --- Sensor sample (runs at sensorHz) ---
				this.dueSensor -= this.DT;
				if (this.dueSensor <= 0 || this.measured === null) {
					this.measured = this.sensors.sample(this.plant.state, params, sensorCfg, this.tSim);
					this.dueSensor += dtSensor;
				}

				// --- Outer attitude loop (runs at outerHz) ---
				this.dueOuter -= this.DT;
				if (this.dueOuter <= 0) {
					// PID biases its error by the tilt setpoint; ArduBalance uses target_angle set above.
					const measOuter = controller instanceof PIDController
						? { ...this.measured, pitch: this.measured.pitch - tiltSetpoint }
						: this.measured;
					controller.updateVelocity(measOuter, gains, dtOuter);
					this.dueOuter += dtOuter;
				}

				// --- Inner motor loop (runs at innerHz) — produces PWM/force, held by ZOH ---
				this.dueInner -= this.DT;
				if (this.dueInner <= 0) {
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
						this.controllers.nn.vel_cart_target = this.nav.v_desired_last ?? 0;
						this.controllers.nn.produceForce(measInner, gains, dtInner, this.motor);
					}

					this.dueInner += dtInner;

					// Record the (sensor → PWM) pair while ArduBalance is running.
					// Ignore PID — we're distilling the cascaded controller specifically.
					if (this.recorder.recording && controller instanceof ArduBalanceController) {
						this.recorder.record({
							pitch:           this.measured.pitch,
							pitch_rate:      this.measured.pitch_rate,
							vel_cart:        this.measured.v,
							vel_cart_target: this.nav.v_desired_last ?? 0,
							pwm:             controller.lastPWM ?? 0,
						});
					}
				}

				// Physics advances every DT with the last computed force held.
				this.plant.step(this.lastForce, this.lastTauYaw, this.DT);
				this.tSim += this.DT;
				// CoM computation for plotting (true state, not sensor-filtered).
				const cs = Math.cos(this.plant.state.pitch);
				const sn = Math.sin(this.plant.state.pitch);
				const x_CoM_true = this.plant.state.x + params.L * sn;
				const v_CoM_true = this.plant.state.v + params.L * cs * this.plant.state.pitch_rate;

				this.history.push({
					t:				this.tSim,
					pitch:			this.plant.state.pitch,
					pitch_rate:		this.plant.state.pitch_rate,
					x:				this.plant.state.x,
					v:				this.plant.state.v,
					x_CoM:			x_CoM_true,
					v_CoM:			v_CoM_true,
					F:		 		this.lastForce,
					// "Motor PWM" = whatever the active controller just commanded.
					pwm:			this.controllers[this.controllerType].lastPWM ?? 0,
					// NN shadow is always the NN's output, regardless of who's driving.
					pwm_nn:			this.controllers.nn.lastPWM ?? 0,
					// Residual uses the active controller as the reference. When NN
					// is active this is 0; meaningful when ArduBalance is the driver.
					pwm_residual:	(this.controllers[this.controllerType].lastPWM ?? 0)
										- (this.controllers.nn.lastPWM ?? 0),
					// vel_command only exists in ArduBalance — leave 0 otherwise.
					vel_command:	this.controllers.ardubalance.vel_command ?? 0,
					v_desired: 		this.nav.v_desired_last ?? 0,
					err_x:		 	this.nav.err_last ?? 0,
					tilt_sp:	 	tiltSetpoint,
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
			this.render();
			
		} else {
			this.lastT = ts;
		}
		
		requestAnimationFrame(ts => this.tick(ts));
	}

	syncControllerPanels() {
		for (const el of document.querySelectorAll('[data-ctrl]')) {
			el.style.display = el.dataset.ctrl === this.controllerType ? '' : 'none';
		}
	}

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

		document.getElementById('btnPush').onclick = () => {
			const shove_rate = this.ui.num('shoveOmega');
			this.plant.state.pitch_rate += shove_rate;
			this.ui.log(this.tSim, `shove: +${shove_rate.toFixed(1)} rad/s tip`);
		};

		document.getElementById('btnExp').onclick = () => this.sweepKp();

		// Recorder UI
		const recBtn	 = document.getElementById('btnRecord');
		const recStats = document.getElementById('recStats');
		const refreshRecStats = () => {
			recStats.textContent = `${this.recorder.size()} samples` +
				(this.recorder.recording ? ' (recording…)' : '');
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
			this.waypoints.push({ x: hit.x, z: hit.z });
			// Head of the queue is the active target.
			const head = this.waypoints[0];
			this.nav.target_x = head.x;
			this.nav.target_z = head.z;
			navTxInput.value = head.x.toFixed(2);
			if (navTzInput) navTzInput.value = head.z.toFixed(2);
			document.getElementById('navEnabled').checked = true;
			document.getElementById('pilotMode').value = 'auto';
			this.ui.log(this.tSim,
				`WP +(${hit.x.toFixed(2)}, ${hit.z.toFixed(2)})  · queue=${this.waypoints.length}`);
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

		if (mode === 'recorded' && this.recorder.data.length < 50) {
			this.ui.log(this.tSim, `need more recorded samples (have ${this.recorder.data.length}, want ≥50)`);
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

	sweepKp() {
		this.ui.log(this.tSim, '--- sweep Kp from 10 to 260 ---');
		const params = this.ui.readParams();
		const gains	= this.ui.readGains();
		const init	 = this.ui.readInit();
		const values = [];
		for (let v = 10; v <= 260; v += 20) values.push(v);
		const results = this.experiment.sweep(
			'Kp', values, params, gains,
			{ th0: init.th0, noise: init.noise, duration: 10 },
			r => this.ui.log(this.tSim,
				`Kp=${r.Kp}	t=${r.survived.toFixed(2)}s	IAE=${r.iae.toFixed(3)}	${r.fell ? 'FELL' : 'ok'}`),
		);
		const best = results.filter(r => !r.fell).sort((a, b) => a.iae - b.iae)[0];
		if (best) this.ui.log(this.tSim, `best Kp=${best.Kp} (IAE=${best.iae.toFixed(3)})`);
	}
}
