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
//   c.outerUpdate(sensors, gains, dt)            (no-op for single-rate)
//   c.innerUpdate(sensors, gains, dt, motor)
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
		// read by updateOuter/updateInner each time they fire.
		this._command = null;
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

		// Mirror nav target onto cascade's internal Nav (kept for the
		// existing NN training path that imitates updateAuto).
		if (this.isCascade()) {
			this.stack.nav.target_x = command.navTarget.x;
			this.stack.nav.target_z = command.navTarget.z;
		}

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

	// Apply per-frame rate UI to the cascade layer schedule. Nav is
	// fixed at 60 Hz (the "human-perceptible" decision rate); mixer/
	// attitude/wheels track outer/inner.
	applyRates({ outerHz, innerHz }) {
		if (!this.isCascade()) return;
		this.stack.setRates({
			nav:      60,
			mixer:    outerHz,
			attitude: outerHz,
			wheels:   innerHz,
		});
	}

	// Slow loop — legacy controllers run their angle PD here. Cascade
	// owns its own internal scheduling and ignores this hook.
	updateOuter(measured, dt) {
		if (this.isCascade()) return;
		this.currentController().outerUpdate(measured, this.currentGains(), dt);
	}

	// Fast loop — every controller emits per-wheel torque, motor turns
	// it into chassis force + yaw torque, sim integrates with ZOH
	// between firings.
	updateInner(measured, dt, motor) {
		const controller   = this.currentController();
		const gains        = this.currentGains();
		const drivetrain   = this.ui.readDrivetrain();

		const wheelOut = controller.innerUpdate(measured, gains, dt, motor);
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
			this.controllers.nn.innerUpdate(measured, this.currentGainsFor('nn'), dt, motor);
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
			if (command.intent === 'cruise') {
				const dx = command.navTarget.x - measured.x;
				const dz = command.navTarget.z - (measured.z ?? 0);
				this.recorder.recordCascadeNav({
					dx, dz,
					heading:         measured.heading,
					vel_cart:        measured.vel_cart,
					vel_target_body: stack.navOut.vel_target_body,
					heading_err:     stack.nav.heading_err,
				});
			}
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
