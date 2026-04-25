// ControllerStack — assembles the four cascade layers and runs them at
// their own rates. Each tick is a four-line tour through the building.
//
//   nav     → { vel_target_body, heading_target }
//   mixer   → { pitch_target, yaw_target }
//   attitude → { force_fwd, torque_yaw }
//   wheels  → { pwm_left, pwm_right, force_fwd_actual, torque_yaw_actual }
//
// Rate hierarchy
// --------------
// Real flight-control firmware doesn't run every layer at the same rate.
// Higher layers think slowly (a nav decision every 1/60 s is plenty for a
// human-driven robot); lower layers run fast (motors need fresh PWM at
// hundreds of Hz to feel stiff). This stack owns its own scheduling — the
// caller ticks `update()` at the wheels rate, and the stack throttles the
// slower layers internally with a zero-order hold on their outputs between
// firings.
//
// Defaults match common balance-bot practice:
//   Nav      —  60 Hz   (browser frame; GPS-class on real hw)
//   Mixer    — 100 Hz
//   Attitude — 100 Hz
//   Wheels   — 400 Hz   (must equal the rate at which update() is called)
//
// What this teaches: change wheelsHz from 400 to 50 and the bot tips —
// not because the controller is wrong, but because the actuator can't
// keep up with the dynamics. Same lesson as on real hardware.
//
// Pilot modes share the cascade. Mode determines what feeds Nav:
//
//   auto  — waypoint nav. Nav picks vel/heading from world target.
//   fbw   — fly-by-wire. Pilot stick → vel and heading-rate; Nav owns the
//           heading integration so the lower layers don't have to know.
//   tilt  — debug. Pilot's pitch_target injected directly, Nav and Mixer
//           skipped. Used to tune Attitude in isolation.

import { Nav      } from './nav.js';
import { NavMixer } from './mixer.js';
import { Attitude } from './attitude.js';
import { Wheels   } from './wheels.js';

const DEFAULT_RATES = { nav: 60, mixer: 100, attitude: 100, wheels: 400 };

export class ControllerStack {
	constructor(rates = DEFAULT_RATES) {
		this.nav      = new Nav();
		this.mixer    = new NavMixer();
		this.attitude = new Attitude();
		this.wheels   = new Wheels();
		this.setRates(rates);

		// Last output of each layer (zero-order hold between firings).
		this.navOut   = { vel_target_body: 0, heading_target: 0 };
		this.mixerOut = { pitch_target: 0,    yaw_target: 0 };
		this.attOut   = { force_fwd: 0,       torque_yaw: 0 };
		this.wheelOut = { pwm_left: 0, pwm_right: 0, F_left: 0, F_right: 0,
		                  force_fwd_actual: 0, torque_yaw_actual: 0 };

		// Time-since-last-firing per layer (s). When ≥ the layer's period,
		// the layer fires and the accumulator resets.
		this.tNav = 0; this.tMixer = 0; this.tAttitude = 0; this.tWheels = 0;
	}

	setRates(rates) {
		this.rates       = { ...DEFAULT_RATES, ...rates };
		this.dtNav       = 1 / this.rates.nav;
		this.dtMixer     = 1 / this.rates.mixer;
		this.dtAttitude  = 1 / this.rates.attitude;
		this.dtWheels    = 1 / this.rates.wheels;
	}

	reset() {
		this.nav.reset();
		this.mixer.reset();
		this.attitude.reset();
		this.wheels.reset();
		this.tNav = 0; this.tMixer = 0; this.tAttitude = 0; this.tWheels = 0;
	}

	// Caller ticks this at the wheels rate. Slower layers fire when their
	// period has elapsed; everyone else sees the cached output above.
	//
	// command: { mode: 'auto' | 'fbw' | 'tilt', stick?, pitch_target?, yaw_target? }
	// gains:   { nav, mixer, attitude, wheels }  (one bag per layer)
	update(sensors, command, gains, motor, dt) {
		this.tNav += dt; this.tMixer += dt; this.tAttitude += dt; this.tWheels += dt;

		const tiltMode = command.mode === 'tilt';

		// Nav and Mixer are skipped entirely in tilt mode — the pilot sets
		// pitch_target directly, no velocity loop runs.
		if (!tiltMode && this.tNav >= this.dtNav) {
			this.navOut = this._runNav(sensors, command, gains.nav, this.tNav);
			this.tNav = 0;
		}
		if (!tiltMode && this.tMixer >= this.dtMixer) {
			this.mixerOut = this.mixer.update(this.navOut, sensors, gains.mixer, this.tMixer);
			this.tMixer = 0;
		}
		if (tiltMode) {
			// Pilot sets the angle target directly; refreshed every tick so
			// stick changes propagate without waiting for a Mixer firing.
			this.mixerOut = {
				pitch_target: command.pitch_target ?? 0,
				yaw_target:   command.yaw_target   ?? 0,
			};
		}

		if (this.tAttitude >= this.dtAttitude) {
			this.attOut = this.attitude.update(this.mixerOut, sensors, gains.attitude, this.tAttitude);
			this.tAttitude = 0;
		}
		if (this.tWheels >= this.dtWheels) {
			this.wheelOut = this.wheels.update(this.attOut, sensors, gains.wheels, this.tWheels, motor);
			this.tWheels = 0;
		}

		return {
			navOut:   this.navOut,
			mixerOut: this.mixerOut,
			attOut:   this.attOut,
			wheelOut: this.wheelOut,
		};
	}

	_runNav(sensors, command, navGains, dt) {
		switch (command.mode) {
			case 'auto': return this.nav.updateAuto(sensors, navGains);
			case 'fbw':  return this.nav.updateFbw(sensors, command.stick ?? {}, navGains, dt);
			default:     return this.navOut;   // unknown mode: hold last
		}
	}
}
