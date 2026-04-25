// ControllerStack — assembles the four cascade layers and runs them in
// order. The orchestrator is small by design: each tick is a four-line
// tour through the building.
//
//   nav     → { vel_target_body, heading_target }
//   mixer   → { pitch_target, yaw_target }
//   attitude → { force_fwd, torque_yaw }
//   wheels  → { pwm_left, pwm_right, force_fwd_actual, torque_yaw_actual }
//
// Each layer is owned by the stack and can be inspected by name —
// stack.mixer.pitch_target, stack.attitude.balance_offset, etc. — for
// plotting, recording, or curiosity.
//
// Three pilot modes share the cascade. Mode determines what feeds Nav:
//
//   auto  — waypoint nav. Nav picks vel/heading from world target.
//   fbw   — fly-by-wire. Pilot stick → vel and heading-rate; Nav owns the
//           heading integration so the lower layers don't have to know.
//   tilt  — debug. Pilot's pitch_target injected directly, Mixer skipped.
//           (No velocity feedback at all; you're flying open-loop on tilt.)

import { Nav      } from './nav.js';
import { NavMixer } from './mixer.js';
import { Attitude } from './attitude.js';
import { Wheels   } from './wheels.js';

export class ControllerStack {
	constructor() {
		this.nav      = new Nav();
		this.mixer    = new NavMixer();
		this.attitude = new Attitude();
		this.wheels   = new Wheels();
	}

	reset() {
		this.nav.reset();
		this.mixer.reset();
		this.attitude.reset();
		this.wheels.reset();
	}

	// command: { mode: 'auto' | 'fbw' | 'tilt', stick?, pitch_target?, yaw_target? }
	// gains:   { nav, mixer, attitude, wheels }
	update(sensors, command, gains, motor, dt) {
		const navOut   = this._runNav(sensors, command, gains.nav, dt);
		const mixerOut = navOut === null
			? this._tiltDirect(command)
			: this.mixer.update(navOut, sensors, gains.mixer, dt);
		const attOut   = this.attitude.update(mixerOut, sensors, gains.attitude, dt);
		const wheelOut = this.wheels.update(attOut,    sensors, gains.wheels, dt, motor);
		return { navOut, mixerOut, attOut, wheelOut };
	}

	_runNav(sensors, command, navGains, dt) {
		switch (command.mode) {
			case 'auto': return this.nav.updateAuto(sensors, navGains);
			case 'fbw':  return this.nav.updateFbw(sensors, command.stick ?? {}, navGains, dt);
			case 'tilt': return this.nav.updateTilt();   // returns null
			default:     return this.nav.updateTilt();
		}
	}

	// Tilt mode: caller supplies pitch_target (and yaw_target = current
	// heading by default). Bypasses Mixer entirely — useful for tuning
	// the Attitude layer in isolation.
	_tiltDirect(command) {
		return {
			pitch_target: command.pitch_target ?? 0,
			yaw_target:   command.yaw_target   ?? 0,
		};
	}
}
