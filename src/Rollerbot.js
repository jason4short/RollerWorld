// Rollerbot — the brain. Controllers (legacy + cascade), the yaw torque
// branch, and the actuator output cache. Takes sensors + a command,
// produces force and yaw torque.
//
// The teaching contrast lives here. Four controllers run from the
// same I/O contract:
//
//   pid          — idealized baseline (no actuator model)
//   ardubalance  — the firmware Roller is modeled on (whole-stack reference)
//   nn           — single MLP trained to imitate ArduBalance
//   cascade      — modern layered: Nav → Mixer → Attitude → Wheels
//
// Rollerbot owns the recorder (App injects it at construction). Recording
// happens inline inside updateInner — the data dictionary for each
// recorded tuple is right next to the layer that produces it.
//
// No scheduling here — Loop decides when updateOuter and updateInner
// fire. No pilot input here either — Pilot decides what command to
// send. Rollerbot just steps the controllers when asked.

import { PitchHoldController }   from './controllers/pitch-hold.js';
import { ArduBalanceController } from './controllers/ardubalance.js';
import { NNController }          from './controllers/nn.js';
import { YawController }         from './controllers/yaw.js';
import { ControllerStack }       from './controllers/stack/index.js';

export class Rollerbot {
	constructor({ ui, recorder, controllerType }) {
		this.ui       = ui;          // for gains; controllers each read their own bag
		this.recorder = recorder;    // pushed inline during updateInner

		this.controllers = {
			pid:         new PitchHoldController(),   // 'pid' key kept for HTML/preset compat
			ardubalance: new ArduBalanceController(),
			nn:          new NNController(),
		};
		this.controllerType = controllerType;   // 'pid' | 'ardubalance' | 'nn' | 'cascade'

		this.yawController = new YawController();   // legacy yaw branch
		this.stack         = new ControllerStack(); // modern cascade

		// Last actuator outputs, held by ZOH between inner-loop firings
		// so the integrator at 500 Hz has a steady force to apply while
		// the inner loop is between updates.
		this.lastForce     = 0;
		this.lastYawTorque = 0;
		this._lastStackOut = null;   // for between-firing plot panels

		// The current command, set by applyCommand() once per RAF requestAnimationFrame and
		// read by updateOuter/updateInner each time they fire.
		this._command = null;
	}

	isCascade() { return this.controllerType === 'cascade'; }
	currentController() { return this.controllers[this.controllerType]; }

	// Active controller's gain bag. Cascade reads its own (per-layer)
	// gains inside the stack; legacy controllers each take a single bag.
	currentGains() {
		if (this.controllerType === 'ardubalance') return this.ui.readArduGains();
		if (this.controllerType === 'cascade')     return null;
		return this.ui.readGains();
	}

	reset() {
		for (const c of Object.values(this.controllers)) c.reset();
		this.stack.reset();
		this.lastForce     = 0;
		this.lastYawTorque = 0;
		this._lastStackOut = null;
	}

	// Pre-step setup. Pilot has decided what it wants; stash everything
	// the controllers need before the inner loop fires. Called once per
	// RAF, before the accumulator drains.
	//
	// command: {
	//   tiltSetpoint, yawRateSetpoint,         // legacy scalars
	//   cascadeCommand: { mode, stick?, pitch_target?, ... },
	//   navTarget:      { x, z },              // for cascade nav mirror & recording
	//   navVelDesired,                          // for NN's vel_cart_target
	// }
	applyCommand(command) {
		this._command = command;
		const controller = this.currentController();

		if (this.isCascade()) {
			// Single source of truth for the waypoint target — the
			// cascade's internal Nav layer mirrors what Pilot decided.
			this.stack.nav.target_x = command.navTarget.x;
			this.stack.nav.target_z = command.navTarget.z;
		} else if (controller instanceof ArduBalanceController) {
			controller.target_angle = command.tiltSetpoint;
		} else if (controller instanceof NNController) {
			// NN swallows the velocity-tracking step; takes vel_cart_target
			// (post-slew) directly from pilot rather than a tilt setpoint.
			controller.vel_cart_target = command.navVelDesired;
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

	// Outer attitude loop — legacy controllers only. Cascade is single-
	// rate at innerHz; its layered structure already separates the
	// timescales conceptually.
	updateOuter(measured, dt) {
		if (this.isCascade()) return;
		const controller = this.currentController();
		const gains      = this.currentGains();
		// PID biases its error by the tilt setpoint; ArduBalance reads
		// target_angle which was set in applyCommand.
		const measOuter = controller instanceof PitchHoldController
			? { ...measured, pitch: measured.pitch - this._command.tiltSetpoint }
			: measured;
		controller.updateVelocity(measOuter, gains, dt);
	}

	// Inner motor loop. Returns { force, torque } and also caches them
	// on this.lastForce / this.lastYawTorque so the integrator can hold
	// them between firings via ZOH.
	updateInner(measured, dt, motor) {
		const controller = this.currentController();
		const gains      = this.currentGains();
		const command    = this._command;

		if (this.isCascade()) {
			const cgains = this.ui.readCascadeGains();
			const out = this.stack.update(measured, command.cascadeCommand, cgains, motor, dt);
			this.lastForce     = out.wheelOut.force_fwd_actual;
			this.lastYawTorque = out.wheelOut.torque_yaw_actual;
			this._lastStackOut = out;
			this._recordCascade(measured, command, out);
		} else {
			const measInner = controller instanceof PitchHoldController
				? { ...measured, pitch: measured.pitch - command.tiltSetpoint }
				: measured;
			this.lastForce = controller.produceForce(measInner, gains, dt, motor);

			// Yaw runs at the inner rate too. Pilot sets the target rate;
			// YawController applies P feedback to drive measured rate to
			// target.
			this.yawController.target_yaw_rate = command.yawRateSetpoint;
			this.lastYawTorque = this.yawController.update(measured, this.ui.readYawGains());

			// "Shadow" forward pass of the NN on the same sensor state —
			// regardless of which controller is driving. Lets us plot
			// what the NN would say alongside the active controller.
			if (this.controllers.nn.mlp && controller !== this.controllers.nn) {
				this.controllers.nn.vel_cart_target = command.navVelDesired;
				this.controllers.nn.produceForce(measInner, gains, dt, motor);
			}

			this._recordLegacy(measured, command, controller);
		}

		return { force: this.lastForce, torque: this.lastYawTorque };
	}

	// Cascade tuples — every layer's stream so a single recording session
	// feeds any of the layer NN trainers. Schema is one push per layer
	// per inner tick.
	_recordCascade(measured, command, out) {
		if (!this.recorder.recording) return;
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
		this.recorder.recordCascadeWheels({
			force_fwd:   stack.attOut.force_fwd,
			torque_yaw:  stack.attOut.torque_yaw,
			vel_cart:    measured.vel_cart,
			yaw_rate:    measured.yaw_rate,
			pwm_left:    out.wheelOut.pwm_left,
			pwm_right:   out.wheelOut.pwm_right,
		});
		// Nav recording only meaningful in auto mode.
		if (command.cascadeCommand?.mode === 'auto') {
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
	}

	// Legacy ArduBalance recording — distill the cascaded firmware
	// controller specifically. PID is ignored on purpose.
	_recordLegacy(measured, command, controller) {
		if (!this.recorder.recording) return;
		if (!(controller instanceof ArduBalanceController)) return;
		this.recorder.record({
			pitch:           measured.pitch,
			pitch_rate:      measured.pitch_rate,
			vel_cart:        measured.vel_cart,
			vel_cart_target: command.navVelDesired,
			pwm:             controller.lastPWM ?? 0,
		});
	}
}
