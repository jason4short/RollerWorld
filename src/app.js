import { Sim }						from './Sim.js';
import { Rollerbot }				from './Rollerbot.js';
import { Pilot }					from './Pilot.js';
import { Loop }						from './Loop.js';
import { Training }					from './Training.js';
import { Motor }					from './physics/motor.js';
import { WorldRenderer3D }			from './render/world-renderer-3d.js';
import { Plotter, PLOT_SIGNALS } 	from './render/plotter.js';
import { UI }						from './UI.js';
import { Recorder }					from './Recorder.js';
import { PRESETS }					from './Presets.js';
import { MotorCalibrator }			from './Calibration.js';
import { Joystick }					from './Joystick.js';

// App — director. Owns the UI/DOM, the requestAnimationFrame RAF loop, the renderer, and the
// plotter. Each frame it reads wall-clock time, asks the Loop to drive
// the Sim/Robot/Pilot trio, then draws.
//
// Three peers, no nesting:
//   Sim    	— physical world (pendulum, motor, sensors, lidar, map)
//   Rollerbot  — controllers (legacy + cascade), produces force/torque
//   Pilot  	— input → command (joystick, keys, planner, waypoints)
//   Loop   	— multi-rate scheduler, the only thing that knows about timing
//
// App reaches into peers via short getter aliases so wireUI / render
// don't have to spell out the path each time.

export class App {
	constructor() {
		this.ui       = new UI();         // wraps the DOM controls (sliders, selects, log)
		this.recorder = new Recorder();   // captures (state, output) tuples for offline NN training
		this.history  = [];               // ring of recent state samples for the plotter (Loop pushes)

		// --- Tooling owned by App (UI-driven, not part of the sim) -----
		const joyEl     = document.getElementById('joystick');
		this.joystick   = joyEl ? new Joystick(joyEl) : null;
		this.renderer   = new WorldRenderer3D(document.getElementById('world'));
		this.plotter    = new Plotter(document.getElementById('plot'));

		// --- The three peers + the scheduler ---------------------------
		this.sim = new Sim({
			params:   this.ui.readParams(),
			motorCfg: this.ui.readMotor(),
		});
		this.sim.setWorld({
			obstacles:  this.renderer.obstacles,
			roadCanvas: this.renderer.roadCanvas,
		});
		this.robot = new Rollerbot({
			ui:             this.ui,
			recorder:       this.recorder,
			controllerType: this.ui.readController(),
		});
		this.pilot = new Pilot({
			ui:            this.ui,
			joystick:      this.joystick,
			lidarMaxRange: this.sim.lidar.maxRange,
		});
		this.loop = new Loop();

		// Training UI — distillation of rule-based controllers into MLPs.
		// Lives outside the sim/robot/pilot triad because it's a tool, not
		// a per-frame concern; runs only when the user clicks Train.
		this.training = new Training({
			ui:           this.ui,
			recorder:     this.recorder,
			robot:        this.robot,
			sim:          this.sim,
			drawLossPlot: losses => this.drawLossPlot(losses),
		});

		this.calibrator = new MotorCalibrator({ dt: this.sim.DT });

		// Per-layer inset plots inside each cascade panel — visitor sees each
		// layer's I/O rolling alongside its gain panel. Short 3 s window so
		// transients are vivid without scrolling.
		this.panelPlots = {
			mixer:  new Plotter(document.getElementById('plotMixer'),    3),
			pitch:  new Plotter(document.getElementById('plotAttPitch'), 3),
			yaw:    new Plotter(document.getElementById('plotAttYaw'),   3),
			wheels: new Plotter(document.getElementById('plotWheels'),   3),
		};

		// --- Visualization toggles -------------------------------------
		this.showLidarRays = true;    // draw ray segments from bot to first hit
		this.showMapGrid   = false;   // overlay accumulated occupancy grid (only meaningful in lidar_astar mode)
		this.currentTab    = 'control'; // sidebar tab — 'control' | 'sim'

		// --- Loop bookkeeping ------------------------------------------
		this.running       = false;   // sim ticking? toggled by the Start/Pause button
		this.lastTimeStamp = 0;       // previous frame's rAF timestamp (ms)

		this.wireUI();
		this.wireKeys();
		this.syncControllerPanels();
		this.populatePlotMenus();
		this.reset();
		this.drawLUT();
		requestAnimationFrame(ts => this.tick(ts));
	}

	// Convenience accessors — UI wiring and render reach through to the
	// peers many times; these keep call sites short and document where
	// each piece of state lives.
	get pendulum()        { return this.sim.pendulum; }
	get motor()           { return this.sim.motor; }
	get measured()        { return this.sim.measured; }
	get occupancyGrid()   { return this.sim.occupancyGrid; }
	get simElapsedTime()  { return this.sim.simElapsedTime; }
	get controllers()     { return this.robot.controllers; }
	get controllerType()  { return this.robot.controllerType; }
	set controllerType(v) { this.robot.controllerType = v; }
	get stack()           { return this.robot.stack; }
	get nav()             { return this.pilot.nav; }
	get road()            { return this.pilot.road; }
	get planner()         { return this.pilot.planner; }
	get waypoints()       { return this.pilot.waypoints; }

	currentController() { return this.robot.currentController(); }

	reset() {
		const { th0 } = this.ui.readInit();
		const params  = this.ui.readParams();
		this.sim.reset({ params, initialPitch: th0 * Math.PI / 180 });
		this.robot.reset();
		this.pilot.reset();
		this.loop.reset();
		this.history.length = 0;
		this.renderer.clearTrail?.();
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
			? [{ x: this.pendulum.state.x, z: this.pendulum.state.z ?? 0 }, ...this.waypoints]
			: []);

		// Sensor-ray viz — when the user has flipped the rays toggle on,
		// show whichever sensor is actively scanning. Road sensor rays in
		// road mode, lidar rays under the lidar_astar planner. The
		// renderer's setLidar() takes any { angle, dist, hit_x, hit_z }
		// list — the two sensors emit the same shape.
		let raysToShow = null;
		if (this.showLidarRays) {
			if (this.ui.readPilotMode() === 'road' && this.measured?.road) {
				raysToShow = this.measured.road;
			} else if ((this.planner.mode === 'lidar_astar' || this.planner.mode === 'reactive')
			           && this.measured?.lidar) {
				raysToShow = this.measured.lidar;
			}
		}
		this.renderer.setLidar(raysToShow, this.pendulum.state, null);
		this.renderer.setOccupancyGrid(
			(this.planner.mode === 'lidar_astar' && this.showMapGrid) ? this.occupancyGrid : null,
		);

		// Periodic safety log so the numbers behind the brake are visible.
		// Throttled to ~2 Hz when actively clipping; silent otherwise.
		if (this.controllerType === 'cascade' && this.stack.safety.lastScale < 0.999) {
			const now = performance.now();
			if (!this._lastSafetyLogT || now - this._lastSafetyLogT > 500) {
				this._lastSafetyLogT = now;
				this.ui.log(this.simElapsedTime,
					`safety: scale=${this.stack.safety.lastScale.toFixed(2)} ` +
					`minDist=${this.stack.safety.lastMinDist.toFixed(2)}m`);
			}
		}

		this.renderer.draw(this.pendulum.state, this.pendulum.params, navTarget, queueRest);
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
		joy.style.display = (mode === 'auto' || mode === 'road') ? 'none' : '';

		// Sync the canvas-overlay pilot buttons too. Angle/FBW reflect
		// the current mode; Auto indicator lights up only in auto.
		const setActive = (id, on) => {
			const el = document.getElementById(id);
			if (el) el.classList.toggle('active', on);
		};
		setActive('btnPilotAngle', mode === 'raw');
		setActive('btnPilotFbw',   mode === 'fbw');
		setActive('btnPilotAuto',  mode === 'auto');
		setActive('btnPilotRoad',  mode === 'road');
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

	// Director loop. Runs every animation frame regardless of pause —
	// rendering still happens so OrbitControls feels live and the
	// joystick stays responsive. When running, hand the wall-clock
	// delta to the Bot and let it advance the sim.
	tick(timestamp) {
		if (this.running) {
			if (!this.lastTimeStamp) this.lastTimeStamp = timestamp;
			const deltaTime = Math.min(0.05, (timestamp - this.lastTimeStamp) / 1000);
			this.lastTimeStamp = timestamp;

			const { justFell } = this.loop.advance(deltaTime, {
				sim:     this.sim,
				robot:   this.robot,
				pilot:   this.pilot,
				history: this.history,
				ui:      this.ui,
			});
			
			if (justFell) {
				this.running = false;
				document.getElementById('btnRun').textContent = 'Start';
			}
		} else {
			this.lastTimeStamp = timestamp;
		}

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

		// Distillation method tabs (Train / Record). Event delegation so
		// the binding is bulletproof regardless of when the tab buttons
		// land in the DOM.
		document.addEventListener('click', (e) => {
			const btn = e.target.closest('.distill-tab');
			if (!btn) return;
			const which = btn.dataset.distillTab;
			for (const b of document.querySelectorAll('.distill-tab')) {
				b.classList.toggle('active', b.dataset.distillTab === which);
			}
			for (const c of document.querySelectorAll('[data-distill-content]')) {
				c.style.display = c.dataset.distillContent === which ? '' : 'none';
			}
		});

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
		document.getElementById('btnPilotRoad').onclick  = () => setPilotMode('road');

		// Road controller's algorithm picker — only meaningful while in road
		// pilot mode, but the toggle is always live so the user can preflip
		// before engaging. Two states: 'wsum' (weighted-sum, Jason's) and
		// 'ftg' (Follow-the-Gap). Button label shows the current choice.
		const btnRoadAlgo = document.getElementById('btnRoadAlgo');
		const refreshAlgoLabel = () => {
			btnRoadAlgo.textContent = this.road.algorithm === 'ftg' ? 'FTG' : 'WSum';
		};
		refreshAlgoLabel();
		btnRoadAlgo.onclick = () => {
			this.road.setAlgorithm(this.road.algorithm === 'ftg' ? 'wsum' : 'ftg');
			refreshAlgoLabel();
			this.ui.log(this.simElapsedTime, `road algo: ${this.road.algorithm}`);
		};

		document.getElementById('btnPush').onclick = () => {
			const shove_rate = this.ui.num('shoveOmega');
			this.sim.shoveTip(shove_rate);
			this.ui.log(this.simElapsedTime, `shove: +${shove_rate.toFixed(1)} rad/s tip`);
		};

		// Camera-follow toggle. ON (default): camera auto-recenters behind
		// the bot after idle. OFF: camera holds whatever pose you orbit to,
		// pans with the bot, never rotates around.
		const btnCamFollow = document.getElementById('btnCamFollow');
		btnCamFollow.onclick = () => {
			const enabled = !btnCamFollow.classList.contains('active');
			btnCamFollow.classList.toggle('active', enabled);
			this.renderer.setAutoFollow(enabled);
			this.ui.log(this.simElapsedTime, `camera follow: ${enabled ? 'on' : 'off'}`);
		};

		const btnLidar = document.getElementById('btnLidar');
		btnLidar.classList.toggle('active', this.showLidarRays);
		btnLidar.onclick = () => {
			this.showLidarRays = !btnLidar.classList.contains('active');
			btnLidar.classList.toggle('active', this.showLidarRays);
			this.ui.log(this.simElapsedTime, `lidar rays: ${this.showLidarRays ? 'shown' : 'hidden'}`);
		};

		const btnMapGrid = document.getElementById('btnMapGrid');
		btnMapGrid.classList.toggle('active', this.showMapGrid);
		btnMapGrid.onclick = () => {
			this.showMapGrid = !btnMapGrid.classList.contains('active');
			btnMapGrid.classList.toggle('active', this.showMapGrid);
			this.ui.log(this.simElapsedTime, `map grid: ${this.showMapGrid ? 'shown' : 'hidden'}`);
		};

		// Disturbances — instantaneous state kicks via Sim's formal API.
		// Force impulse → Δv = J / (M+m). Yaw impulse → Δω = J / I_yaw.
		const pushChassis = sign => {
			const J  = sign * this.ui.num('disturbForceImpulse');
			const dv = this.sim.pushChassis(J);
			this.ui.log(this.simElapsedTime, `chassis push: ${J.toFixed(1)} N·s → Δv=${dv.toFixed(2)} m/s`);
		};
		const yawKick = sign => {
			const J = sign * this.ui.num('disturbTauImpulse');
			this.sim.yawKick(J);
			this.ui.log(this.simElapsedTime, `yaw kick: ${J.toFixed(2)} N·m·s`);
		};
		document.getElementById('btnDisturbForward').onclick = () => pushChassis(+1);
		document.getElementById('btnDisturbBack').onclick    = () => pushChassis(-1);
		document.getElementById('btnDisturbYawL').onclick    = () => yawKick(+1);
		document.getElementById('btnDisturbYawR').onclick    = () => yawKick(-1);

		const biasBtn = document.getElementById('btnDisturbBiasToggle');
		biasBtn.onclick = () => {
			if (this.sim.imuBiasInjected !== 0) {
				this.sim.setImuBias(0);
				biasBtn.textContent = 'Bias OFF';
				this.ui.log(this.simElapsedTime, 'IMU bias cleared');
			} else {
				const bias = this.ui.num('disturbImuBias');
				this.sim.setImuBias(bias);
				biasBtn.textContent = `Bias ON (+${(bias * 180 / Math.PI).toFixed(1)}°)`;
				this.ui.log(this.simElapsedTime, `IMU bias on: +${(bias * 180 / Math.PI).toFixed(1)}°`);
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
				this.ui.log(this.simElapsedTime, `recording stopped: ${this.recorder.size()} samples`);
			} else {
				this.recorder.start();
				recBtn.textContent = 'Stop';
				this.ui.log(this.simElapsedTime, 'recording started');
			}
			refreshRecStats();
		};
		document.getElementById('btnClearRec').onclick = () => {
			this.recorder.clear();
			this.ui.log(this.simElapsedTime, 'recording cleared');
			refreshRecStats();
		};
		document.getElementById('btnSaveRec').onclick = () => {
			if (this.recorder.size() === 0) return this.ui.log(this.simElapsedTime, 'no samples to save');
			this.recorder.download();
			this.ui.log(this.simElapsedTime, `saved ${this.recorder.size()} samples`);
		};
		// Update the stats line periodically while recording.
		setInterval(refreshRecStats, 250);

		document.getElementById('btnTrainNN').onclick       = () => this.training.trainNN();
		document.getElementById('btnTrainPitchNN').onclick  = () => this.training.trainAttitudePitchNN();
		document.getElementById('btnTrainPitchRNN').onclick = () => this.training.trainAttitudePitchRNN();
		document.getElementById('pitch_mode').onchange = e => {
			const mode = e.target.value;
			if (mode === 'nn') {
				const mlp = this.training.attitudePitchMlp;
				if (!mlp) {
					this.ui.log(this.simElapsedTime, 'no trained pitch NN yet — staying on rule');
					e.target.value = 'rule';
					return;
				}
				this.stack.attitude.setPitchMode('nn', mlp);
			} else if (mode === 'rnn') {
				const rnn = this.training.attitudePitchRnn;
				if (!rnn) {
					this.ui.log(this.simElapsedTime, 'no trained pitch RNN yet — staying on rule');
					e.target.value = 'rule';
					return;
				}
				this.stack.attitude.setPitchMode('rnn', rnn);
			} else {
				this.stack.attitude.setPitchMode('rule');
			}
			this.ui.log(this.simElapsedTime, `pitch: ${mode}`);
		};

		document.getElementById('btnTrainMixerNN').onclick = () => this.training.trainMixerNN();
		document.getElementById('mixer_mode').onchange = e => {
			const mode = e.target.value;
			const mlp = this.training.mixerMlp;
			if (mode === 'nn' && !mlp) {
				this.ui.log(this.simElapsedTime, 'no trained mixer NN yet — staying on rule');
				e.target.value = 'rule';
				return;
			}
			this.stack.mixer.setMode(mode, mlp);
			this.ui.log(this.simElapsedTime, `mixer: ${mode}`);
		};

		document.getElementById('btnTrainYawNN').onclick = () => this.training.trainAttitudeYawNN();
		document.getElementById('yaw_mode').onchange = e => {
			const mode = e.target.value;
			const mlp = this.training.attitudeYawMlp;
			if (mode === 'nn' && !mlp) {
				this.ui.log(this.simElapsedTime, 'no trained yaw NN yet — staying on rule');
				e.target.value = 'rule';
				return;
			}
			this.stack.attitude.setYawMode(mode, mlp);
			this.ui.log(this.simElapsedTime, `yaw: ${mode}`);
		};

		document.getElementById('btnTrainNavNN').onclick = () => this.training.trainNavNN();

		// Planner mode (Direct / A* ground truth / A* lidar map / Reactive).
		// Sync the planner instance to whatever the dropdown is showing
		// at boot — Firefox restores SELECT values across reloads, which
		// can leave the dropdown showing one thing and the planner
		// defaulting to another. After this, change events keep them
		// in sync.
		const plannerSel = document.getElementById('plannerMode');
		this.planner.setMode(plannerSel.value);
		plannerSel.onchange = e => {
			this.planner.setMode(e.target.value);
			this.ui.log(this.simElapsedTime, `planner: ${e.target.value}`);
		};
		document.getElementById('btnForgetMap').onclick = () => {
			this.occupancyGrid.clear();
			this.ui.log(this.simElapsedTime, 'occupancy map cleared');
		};
		document.getElementById('nav_mode_nn').onchange = e => {
			const mode = e.target.value;
			const mlp = this.training.navMlp;
			if (mode === 'nn' && !mlp) {
				this.ui.log(this.simElapsedTime, 'no trained nav NN yet — staying on rule');
				e.target.value = 'rule';
				return;
			}
			this.stack.nav.setAutoMode(mode, mlp);
			this.ui.log(this.simElapsedTime, `nav (auto): ${mode}`);
		};

		document.getElementById('btnCalibrate').onclick = () => this.calibrateMotor();
		document.getElementById('btnClearLUT').onclick = () => {
			this.controllers.ardubalance.pwmTable.clear();
			this.ui.log(this.simElapsedTime, 'PWM LUT cleared → linear fallback');
			this.drawLUT();
		};

		document.getElementById('ctrlType').onchange = e => {
			this.controllerType = e.target.value;
			this.syncControllerPanels();
			this.ui.log(this.simElapsedTime, `controller: ${this.controllerType}`);
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
			let hit = this.renderer.screenToGround(e.clientX, e.clientY);
			if (!hit) return;
			// If the click landed inside a tree/boulder, snap to the
			// nearest free spot so the flag is reachable. Without this,
			// the reactive nav can never approach the goal — its own
			// avoidance bubble keeps the bot out of the obstacle the
			// goal is buried inside.
			const snapped = this.renderer.obstacles.nearestFree(hit.x, hit.z, 1.2);
			if (snapped.x !== hit.x || snapped.z !== hit.z) {
				this.ui.log(this.simElapsedTime,
					`flag snapped to nearest free spot (Δ=${
						Math.hypot(snapped.x - hit.x, snapped.z - hit.z).toFixed(2)} m)`);
				hit = snapped;
			}

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
				: (this.measured ?? this.pendulum.state);
			// lidar_astar plans on the bot's accumulated map (bot's view);
			// astar/direct use ground-truth obstacles (cheating, but useful
			// as a teaching contrast).
			const obstacles = this.planner.mode === 'lidar_astar'
				? this.occupancyGrid : this.renderer.obstacles;
			const pad = this.planner.mode === 'lidar_astar' ? 0.6 : 0.25;
			const path  = this.planner.plan(
				{ x: start.x, z: start.z ?? 0 }, hit,
				{ obstacles, res: 0.25, pad },
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
			this.ui.log(this.simElapsedTime,
				`WP via ${this.planner.mode}: (${hit.x.toFixed(2)}, ${hit.z.toFixed(2)})  · path=${path.length}, queue=${this.waypoints.length}`);
		});

		for (const btn of document.querySelectorAll('[data-preset]')) {
			btn.onclick = () => {
				const p = PRESETS[btn.dataset.preset];
				this.ui.writeAll(p);
				this.ui.log(this.simElapsedTime, `preset: ${btn.dataset.preset}`);
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
			if (pressed.has('ArrowUp'))		tilt += this.pilot.keyTiltMax;
			if (pressed.has('ArrowDown'))	tilt -= this.pilot.keyTiltMax;
			this.pilot.keyTilt = tilt;

			// Left/Right → yaw rate command.
			let rate = 0;
			if (pressed.has('ArrowLeft'))	rate += this.pilot.keyYawRateMax;
			if (pressed.has('ArrowRight')) 	rate -= this.pilot.keyYawRateMax;
			this.pilot.keyYawRate = rate;
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
		this.ui.log(this.simElapsedTime,
			`calibrated: ${results.length} points, top=${results.at(-1).speed.toFixed(2)} m/s @ PWM ${PWM_max}`);
		for (const r of results) {
			this.ui.log(this.simElapsedTime, `	PWM=${r.pwm.toString().padStart(4)} → ${r.speed.toFixed(3)} m/s`);
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
		this.ui.log(this.simElapsedTime, `saved tuning "${name}"`);
		this.renderSavedTunings();
	}

	loadTuning(name) {
		const store = this._loadStore();
		const t = store[name];
		if (!t) return;
		this.ui.writeAll(t);
		this.ui.log(this.simElapsedTime, `loaded tuning "${name}"`);
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
		this.ui.log(this.simElapsedTime, 'exported tuning');
	}

	async importTuning(file) {
		try {
			const text = await file.text();
			const tuning = JSON.parse(text);
			this.ui.writeAll(tuning);
			this.ui.log(this.simElapsedTime, `imported "${file.name}"`);
			this.running = false;
			document.getElementById('btnRun').textContent = 'Start';
			this.reset();
		} catch (err) {
			this.ui.log(this.simElapsedTime, `import failed: ${err.message}`);
		}
	}

}
