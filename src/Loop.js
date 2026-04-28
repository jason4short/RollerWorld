// Loop — RAF dispatcher + sensor sampling + physics integration.
//
// Each advance() banks wall-clock time and drains it in DT (2 ms at
// 500 Hz) physics steps. Within a step:
//
//   sensorHz — Sim samples IMU/encoder (+ optional lidar/road)
//   bot.tick — Rollerbot accumulates dt and fires outer/inner controller
//              updates on its own schedule (was Loop's job; CP6 moved
//              the timing into the bot)
//   DT       — Sim integrates the held force/torque
//   per-step — pushHistory snapshots state into the plotter ring
//
// Pilot.command runs once per RAF (not per physics step) — pilot
// decisions are human-perceptible, no need for inner-loop rate.
//
// What this teaches: change rates.innerHz from 400 to 50 in the UI and
// the bot tips. Not because the controller is wrong, but because the
// actuator can't keep up with the dynamics — same lesson as on real
// hardware.

export class Loop {
	constructor() {
		this.accumulator = 0;
		this.dueSensor   = 0;
	}

	reset() {
		this.accumulator = 0;
		this.dueSensor   = 0;
	}

	// One animation frame. Returns { justFell } so the caller can flip
	// pause UI when the bot tips over.
	advance(deltaTime, { sim, robot, pilot, history, ui }) {
		const params    = ui.readParams();
		const sensorCfg = ui.readSensors();
		const rates     = ui.readRates();
		const motorCfg  = ui.readMotor();

		sim.applyConfig({ params, motorCfg });
		this.accumulator += deltaTime;

		// Pilot work runs at RAF rate. It's ok for pilot to peek at the
		// last sensor sample even if it's slightly stale — the planner
		// and dispatch don't need 500 Hz freshness.
		// On the first frame after a reset, the sensor sample hasn't run
		// yet — fall back to truth state so Pilot's nav controllers don't
		// trip on a null read. The fallback is harmless once Sim has
		// sampled because measured replaces it.
		const sensors = sim.measured ?? sim.pendulum.state;
		
		pilot.maybeReplan(sensors, sim.occupancyGrid, sim.simElapsedTime);
		
		const command = pilot.command(sensors,
			Math.max(0.001, Math.min(0.1, deltaTime || 0.016)),
			sim.simElapsedTime);
			
		robot.applyCommand(command);
		robot.applyRates(rates);

		const dtSensor = 1 / Math.max(1, rates.sensorHz);

		let justFell = false;

		while (this.accumulator >= sim.DT) {
			// --- Sensor sample (sensorHz) ---
			this.dueSensor -= sim.DT;
			if (this.dueSensor <= 0 || sim.measured === null) {
				sim.sampleSensors(params, sensorCfg, {
					scanLidar:    pilot.wantsLidar(),
					scanRoad:     pilot.wantsRoad(),
					integrateMap: pilot.shouldIntegrateMap(),
				});
				this.dueSensor += dtSensor;
			}

			// --- Rollerbot fires its own outer/inner cadences. ---
			robot.tick(sim.measured, sim.DT, sim.motor);

			// Physics integrates every DT with the last computed force
			// held by ZOH between inner-loop firings.
			sim.integrate(robot.lastForce, robot.lastYawTorque);

			pushHistory(history, sim, robot, pilot, command, params);

			this.accumulator -= sim.DT;

			if (sim.hasFallen()) { justFell = true; break; }
		}

		return { justFell };
	}
}

// Single source of truth for the plotter / panel-plot signal schema.
// Reaches across all three peers — that's the lesson, the plotter is a
// system-level view, not any one module's internal trace.
export function pushHistory(history, sim, robot, pilot, command, params) {
	const state = sim.pendulum.state;
	// CoM computation for plotting (true state, not sensor-filtered).
	const cs = Math.cos(state.pitch);
	const sn = Math.sin(state.pitch);
	const x_CoM_true = state.x + params.L * sn;
	const v_CoM_true = state.vel_cart + params.L * cs * state.pitch_rate;

	// "Motor PWM" = whatever the motor actually drove. Larger-magnitude
	// wheel reported as the scalar trace; per-wheel PWMs are exposed below.
	const isCascade = robot.isCascade();
	const activePwm = Math.abs(robot.lastPwmLeft) > Math.abs(robot.lastPwmRight)
		? robot.lastPwmLeft
		: robot.lastPwmRight;

	history.push({
		t:				sim.simElapsedTime,
		pitch:			state.pitch,
		pitch_rate:		state.pitch_rate,
		x:				state.x,
		vel_cart:		state.vel_cart,
		x_CoM:			x_CoM_true,
		v_CoM:			v_CoM_true,
		F:				robot.lastForce,
		pwm:			activePwm,
		// NN shadow is always the NN's output, regardless of who's driving.
		pwm_nn:			robot.controllers.nn.lastPWM ?? 0,
		pwm_residual:	activePwm - (robot.controllers.nn.lastPWM ?? 0),
		// vel_command only exists in ArduBalance — leave 0 otherwise.
		vel_command:	robot.controllers.ardubalance.vel_command ?? 0,
		vel_desired:	pilot.nav.vel_desired_last ?? robot.stack.mixer.vel_target ?? 0,
		err_x:			pilot.nav.err_last ?? robot.stack.nav.distance_err ?? 0,
		tilt_sp:		command.intent === 'attitude'
							? command.tilt
							: (robot.stack.mixer.pitch_target ?? 0),
		// Cascade-specific traces (zero in legacy modes).
		pitch_target:	robot.stack.mixer.pitch_target ?? 0,
		force_fwd:		robot.stack.attitude.lastForceFwd ?? 0,
		torque_yaw:		robot.stack.attitude.lastTorqueYaw ?? 0,
		pwm_left:		robot.lastPwmLeft,
		pwm_right:		robot.lastPwmRight,
	});

	if (history.length > 5000) history.shift();
}
