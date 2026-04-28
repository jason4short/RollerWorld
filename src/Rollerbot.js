// Rollerbot — the brain. Holds four controllers behind a uniform
// interface and dispatches per-frame commands to the active one.
//
//   pitch-hold   — single-loop PD-on-pitch (no actuator model)
//   ardubalance  — the firmware Roller is modeled on (whole-stack reference)
//   nn           — single MLP trained to imitate ArduBalance
//   cascade      — modern layered: Nav → Mixer → Attitude → Wheels
//
// All four expose:
//   c.setAttitude(tilt, yawRate)
//   c.slowLoop(sensors, gains, dt)            (no-op for single-rate)
//   c.fastLoop(sensors, gains, dt, motor)
//                          → { torque_left, torque_right }
//
// Each controller owns its own pitch + yaw conversion to per-wheel
// torque. The motor module then takes per-wheel torque, runs the
// inverse-model FF + force-tracking PI + deadband + saturation, and
// returns the chassis force / yaw torque the sim integrates.
//
// Rollerbot owns the recorder (App injects it). Recording happens
// inline inside updateInner — the data dictionary for each recorded
// tuple is right next to the layer that produces it.
//
// No scheduling here — Loop decides when updateOuter and updateInner
// fire. No pilot input here either — Pilot decides what command to
// send. Rollerbot just steps the active controller when asked.

import { PitchHoldController }   from './controllers/pitch-hold.js';
import { ArduBalanceController } from './controllers/ardubalance.js';
import { NNController }          from './controllers/nn.js';
import { ControllerStack }       from './controllers/stack/index.js';

export class Rollerbot {
	constructor({ ui, recorder, controllerType }) {
		this.ui       = ui;          // for gains; controllers each read their own bag
		this.recorder = recorder;    // pushed inline during updateInner

		this.controllers = {
			pid:         new PitchHoldController(),   // 'pid' key kept for HTML/preset compat
			ardubalance: new ArduBalanceController(),
			nn:          new NNController(),
			cascade:     new ControllerStack(),
		};
		this.controllerType = controllerType;   // 'pid' | 'ardubalance' | 'nn' | 'cascade'

		// Convenient cascade alias — App still pokes at this.stack to
		// install trained MLPs into specific layers.
		this.stack = this.controllers.cascade;

		// Last actuator outputs, held by ZOH between inner-loop firings
		// so the integrator at 500 Hz has a steady force to apply while
		// the inner loop is between updates.
		this.lastForce        = 0;
		this.lastYawTorque    = 0;
		this.lastTorqueLeft   = 0;
		this.lastTorqueRight  = 0;
		this.lastPwmLeft      = 0;
		this.lastPwmRight     = 0;

		// The current command, set by applyCommand() once per RAF and
		// read by the outer/inner firings.
		this._command = null;

		// Outer/inner timing — bot owns its own multi-rate schedule.
		// Loop hands DT (physics step) per tick; we accumulate and fire
		// the slow loop at outerHz, fast loop at innerHz.
		this._tOuter = 0;
		this._tInner = 0;
		this.outerHz = 100;
		this.innerHz = 400;
	}

	isCascade() { return this.controllerType === 'cascade'; }
	currentController() { return this.controllers[this.controllerType]; }

	// Active controller's gain bag. Cascade reads its own (per-layer)
	// gains; legacy controllers each take a single bag merged with yaw
	// terms (Kyaw, MaxTauYaw) since each controller owns its yaw branch.
	currentGains() {
		if (this.controllerType === 'cascade') return this.ui.readCascadeGains();
		const yaw = this.ui.readYawGains();
		const nav = this.ui.readNavGains();   // for setCruise's heading P-loop
		const dt  = this.ui.readDrivetrain();
		const wb  = { wheelbase: dt.wheelbase };
		if (this.controllerType === 'ardubalance') return { ...this.ui.readArduGains(), ...yaw, ...nav, ...wb };
		return { ...this.ui.readGains(), ...yaw, ...nav, ...wb };
	}

	reset() {
		for (const c of Object.values(this.controllers)) c.reset();
		this.lastForce        = 0;
		this.lastYawTorque    = 0;
		this.lastTorqueLeft   = 0;
		this.lastTorqueRight  = 0;
		this.lastPwmLeft      = 0;
		this.lastPwmRight     = 0;
		this._tOuter          = 0;
		this._tInner          = 0;
	}

	// Pre-step setup. Pilot has decided what it wants; stash the command
	// and forward it to the active controller. Called once per RAF.
	//
	// command: {
	//   intent: 'cruise' | 'attitude',
	//   speed, heading,           — when cruise
	//   tilt, yawRate,            — when attitude
	//   navTarget: { x, z },      — cascade auto path & recording
	//   navVelDesired,            — for NN training shadow
	// }
	applyCommand(command) {
		this._command = command;
		const controller = this.currentController();

		if (command.intent === 'cruise') {
			if (typeof controller.setCruise === 'function') {
				controller.setCruise(command.speed, command.heading);
				
			} else {
				// Attitude-only controller (PitchHold) can't be driven by
				// nav. Quietly hold attitude at zero so the bot doesn't
				// fall over; UI surfaces the mismatch.
				controller.setAttitude(0, 0);
			}
			
		} else {
			controller.setAttitude(command.tilt, command.yawRate);
		}
	}

	// Apply per-frame rate UI. Bot uses outerHz/innerHz directly for
	// legacy controllers; cascade also propagates them into its
	// per-layer schedule (Nav fixed at 60 Hz — human-perceptible).
	applyRates({ outerHz, innerHz }) {
		this.outerHz = outerHz;
		this.innerHz = innerHz;
		if (this.isCascade()) {
			this.stack.setRates({
				nav:      60,
				mixer:    outerHz,
				attitude: outerHz,
				wheels:   innerHz,
			});
		}
	}


	// Loop-facing tick. Called once per physics step (DT). Accumulates
	// time and fires slowLoop / fastLoop when their periods elapse.
	// Cascade owns its own internal scheduling — for cascade we tick
	// fastLoop every step and the stack throttles inside.
	tick(measured, dt, motor) {
		this._tOuter += dt;
		this._tInner += dt;

		const dtOuter = 1 / Math.max(1, this.outerHz);
		const dtInner = 1 / Math.max(1, this.innerHz);

		if (!this.isCascade() && this._tOuter >= dtOuter) {
			this.currentController().slowLoop(measured, this.currentGains(), dtOuter);
			this._tOuter = 0;
		}

		if (this._tInner >= dtInner) {
			this._fastLoop(measured, dtInner, motor);
			this._tInner = 0;
		}
	}


	// Fast loop — every controller emits per-wheel torque, motor turns
	// it into chassis force + yaw torque, sim integrates with ZOH
	// between firings.
	_fastLoop(measured, dt, motor) {
		const controller   = this.currentController();
		const gains        = this.currentGains();
		const drivetrain   = this.ui.readDrivetrain();

		const wheelOut = controller.fastLoop(measured, gains, dt, motor);
		const applied  = motor.applyTorque(wheelOut, measured, dt, drivetrain);

		this.lastForce        = applied.force;
		this.lastYawTorque    = applied.yaw_torque;
		this.lastTorqueLeft   = wheelOut.torque_left;
		this.lastTorqueRight  = wheelOut.torque_right;
		this.lastPwmLeft      = applied.pwm_left;
		this.lastPwmRight     = applied.pwm_right;

		// Shadow forward pass of the NN on the same sensor state —
		// regardless of who's driving. Lets us plot what the NN would
		// say alongside the active controller. Skip when NN is the
		// active controller (it already ran).
		if (this.controllers.nn.mlp && controller !== this.controllers.nn) {
			this.controllers.nn.vel_cart_target = this._command.navVelDesired ?? 0;
			this.controllers.nn.fastLoop(measured, this.currentGainsFor('nn'), dt, motor);
		}

		this._record(measured);

		return { force: this.lastForce, torque: this.lastYawTorque };
	}

	// Build a gain bag for a non-active controller (used for the NN
	// shadow pass when something else is driving).
	currentGainsFor(type) {
		if (type === 'cascade') return this.ui.readCascadeGains();
		const yaw = this.ui.readYawGains();
		const nav = this.ui.readNavGains();
		const dt  = this.ui.readDrivetrain();
		const wb  = { wheelbase: dt.wheelbase };
		if (type === 'ardubalance') return { ...this.ui.readArduGains(), ...yaw, ...nav, ...wb };
		return { ...this.ui.readGains(), ...yaw, ...nav, ...wb };
	}

	// Uniform telemetry — pushHistory and per-panel plotters drain
	// from this. Every controller exposes lastForceFwd / lastTorqueYaw
	// (PitchHold and NN populate them in fastLoop; cascade reads
	// from its Attitude layer; ArduBalance from _produceForce). Where
	// a field doesn't apply, it's 0 — the callers tolerate zeros so
	// every signal in the schema can be plotted regardless of
	// controller.
	telemetry() {
		const ctrl       = this.currentController();
		const isCascade  = this.isCascade();
		const stack      = this.stack;
		const ardu       = this.controllers.ardubalance;

		const force_fwd = isCascade ? (stack.attitude.lastForceFwd  ?? 0) : (ctrl.lastForceFwd  ?? 0);
		const torque_yaw = isCascade ? (stack.attitude.lastTorqueYaw ?? 0) : (ctrl.lastTorqueYaw ?? 0);

		// Scalar PWM trace = larger-magnitude wheel.
		const activePwm = Math.abs(this.lastPwmLeft) > Math.abs(this.lastPwmRight)
			? this.lastPwmLeft
			: this.lastPwmRight;

		// Cascade exposes its mixer's pitch_target directly; legacy
		// controllers use whatever was set via setAttitude (or 0 in
		// cruise mode, where the angle target is held vertical).
		const pitch_target = isCascade ? (stack.mixer.pitch_target ?? 0) : (ctrl.tilt_target ?? 0);

		return {
			force:        this.lastForce,
			yaw_torque:   this.lastYawTorque,
			torque_left:  this.lastTorqueLeft,
			torque_right: this.lastTorqueRight,
			pwm:          activePwm,
			pwm_left:     this.lastPwmLeft,
			pwm_right:    this.lastPwmRight,
			pwm_nn:       this.controllers.nn.lastPWM ?? 0,
			vel_command:  ardu.vel_command ?? 0,
			pitch_target,
			force_fwd, torque_yaw,
		};
	}

	_record(measured) {
		if (!this.recorder.recording) return;
		const command = this._command;

		if (this.isCascade()) {
			const stack = this.stack;
			this.recorder.recordCascadeMixer({
				vel_lpf:      stack.mixer.vel_lpf,
				vel_target:   stack.mixer.vel_target,
				pitch_target: stack.mixer.pitch_target,
			});
			this.recorder.recordCascadePitch({
				pitch:        measured.pitch,
				pitch_rate:   measured.pitch_rate,
				pitch_target: stack.mixer.pitch_target,
				force_fwd:    stack.attitude.lastForceFwd,
			});
			this.recorder.recordCascadeYaw({
				heading_err: stack.attitude.lastHeadingErr,
				yaw_rate:    measured.yaw_rate,
				torque_yaw:  stack.attitude.lastTorqueYaw,
			});
			return;
		}

		// ArduBalance whole-stack distillation tuples. PitchHold and NN
		// are not training targets; skip.
		const controller = this.currentController();
		if (controller instanceof ArduBalanceController) {
			this.recorder.record({
				pitch:           measured.pitch,
				pitch_rate:      measured.pitch_rate,
				vel_cart:        measured.vel_cart,
				vel_cart_target: command.navVelDesired,
				pwm:             controller.lastPWM ?? 0,
			});
		}
	}
}
