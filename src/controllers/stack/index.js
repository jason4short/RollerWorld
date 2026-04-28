// ControllerStack — assembles the three cascade layers and runs them at
// their own rates. Each tick is a three-line tour through the building.
//
//   mixer    → { pitch_target, yaw_target }
//   attitude → { force_fwd, torque_yaw }
//   wheels   → { torque_left, torque_right }                              (motor module owns PWM)
//
// Navigation lives in Pilot — Pilot.NavController emits speed/heading,
// Pilot calls cascade.setCruise(speed, heading), and the Mixer takes
// those directly. The cascade no longer carries its own Nav layer; one
// navigator, one source of waypoint-following truth.
//
// Rate hierarchy
// --------------
// Real flight-control firmware doesn't run every layer at the same rate.
// Higher layers think slowly; lower layers run fast (motors need fresh
// PWM at hundreds of Hz to feel stiff). This stack owns its own
// scheduling — the caller ticks `update()` at the wheels rate, and the
// stack throttles the slower layers internally with a zero-order hold
// on their outputs between firings.
//
// Defaults match common balance-bot practice:
//   Mixer    — 100 Hz
//   Attitude — 100 Hz
//   Wheels   — 400 Hz   (must equal the rate at which update() is called)
//
// What this teaches: change wheelsHz from 400 to 50 and the bot tips —
// not because the controller is wrong, but because the actuator can't
// keep up with the dynamics. Same lesson as on real hardware.
//
// Pilot-supplied modes:
//
//   cruise — Pilot's nav (or stick) emits speed + heading; pass-through.
//   tilt   — Pilot's pitch_target injected directly, Mixer skipped.
//            Used by raw mode and to tune Attitude in isolation.

import { NavMixer } from './Mixer.js';
import { Attitude } from './Attitude.js';
import { Wheels   } from './Wheels.js';
import { Safety   } from './Safety.js';

const DEFAULT_RATES = { mixer: 100, attitude: 100, wheels: 400 };

export class ControllerStack {
	constructor(rates = DEFAULT_RATES) {
		this.mixer   	 = new NavMixer();
		this.attitude	 = new Attitude();
		this.wheels  	 = new Wheels();
		this.safety  	 = new Safety();
		this.setRates(rates);

		// Pilot-supplied command (mode + speed/heading or pitch_target).
		// Set by Rollerbot via setCruise / setAttitude each frame;
		// consumed by the inner loop. Default is benign tilt-zero — bot
		// holds vertical until Pilot's first frame lands.
		this._command = { mode: 'tilt', pitch_target: 0, yaw_target: 0, heading_rate_ff: 0 };

		// Last output of each layer (zero-order hold between firings).
		// navOut is now just whatever setCruise stashed — the Nav layer
		// is gone; this field stays as Mixer's input shape.
		this.navOut   = { vel_target_body: 0, heading_target: 0, heading_rate_ff: 0 };
		this.mixerOut = { pitch_target: 0,    yaw_target: 0 };
		this.attOut   = { force_fwd: 0,       torque_yaw: 0 };
		this.wheelOut = { torque_left: 0, torque_right: 0 };

		// Time-since-last-firing per layer (s).
		this.tMixer = 0; this.tAttitude = 0; this.tWheels = 0;
	}

	setRates(rates) {
		this.rates       = { ...DEFAULT_RATES, ...rates };
		this.dtMixer     = 1 / this.rates.mixer;
		this.dtAttitude  = 1 / this.rates.attitude;
		this.dtWheels    = 1 / this.rates.wheels;
	}

	reset() {
		this.mixer.reset();
		this.attitude.reset();
		this.wheels.reset();
		this.tMixer = 0; this.tAttitude = 0; this.tWheels = 0;
	}

	// Uniform controller surface — Rollerbot calls these for every
	// controller, cascade included. setAttitude maps to 'tilt' mode
	// (Mixer skipped); setCruise maps to 'cruise' mode (Mixer takes
	// speed + heading like any other Nav out).
	setAttitude(tilt, yawRate) {
		this._command = {
			mode:            'tilt',
			pitch_target:    tilt    ?? 0,
			yaw_target:      yawRate ?? 0,   // raw key-yaw uses heading; fbw uses rate
			heading_rate_ff: yawRate ?? 0,
		};
	}

	setCruise(speed, heading) {
		this._command = { mode: 'cruise', speed, heading };
	}

	slowLoop() { /* internal scheduling owns the slow layers */ }

	fastLoop(sensors, gains, dt, motor) {
		const out = this.update(sensors, this._command, gains, motor, dt);
		return out.wheelOut;   // { torque_left, torque_right }
	}

	// Caller ticks this at the wheels rate. Slower layers fire when their
	// period has elapsed; everyone else sees the cached output above.
	//
	// command: { mode: 'cruise', speed, heading } | { mode: 'tilt', pitch_target, yaw_target, heading_rate_ff }
	// gains:   { mixer, attitude, wheels, safety }
	update(sensors, command, gains, motor, dt)
	{
		this.tMixer += dt; this.tAttitude += dt; this.tWheels += dt;

		const tiltMode = command.mode === 'tilt';

		// Cruise mode: Pilot's nav (or stick) already produced
		// vel_target_body and heading_target. Stash them where Mixer
		// expects to read.
		if (!tiltMode) {
			this.navOut = {
				vel_target_body: command.speed   ?? 0,
				heading_target:  command.heading ?? 0,
				heading_rate_ff: 0,
			};
			// Safety governor runs at wheels rate so a sudden close
			// obstacle reacts faster than Pilot's RAF cadence.
			this.navOut = this.safety.apply(this.navOut, sensors, gains.safety);

			if (this.tMixer >= this.dtMixer) {
				this.mixerOut = this.mixer.update(this.navOut, sensors, gains.mixer, this.tMixer);
				this.tMixer = 0;
			}
		} else {
			// Pilot sets the angle target directly — refreshed every tick
			// so stick changes propagate without waiting for a Mixer
			// firing. heading_rate_ff lets Attitude rotate smoothly
			// between integrated-target updates instead of stepping.
			this.mixerOut = {
				pitch_target:    command.pitch_target    ?? 0,
				yaw_target:      command.yaw_target      ?? 0,
				heading_rate_ff: command.heading_rate_ff ?? 0,
			};
		}

		if (this.tAttitude >= this.dtAttitude) {
			this.attOut = this.attitude.update(this.mixerOut, sensors, gains.attitude, this.tAttitude);
			this.tAttitude = 0;
		}
		if (this.tWheels >= this.dtWheels) {
			this.wheelOut = this.wheels.update(this.attOut, sensors, gains.wheels);
			this.tWheels = 0;
		}

		return {
			navOut:   this.navOut,
			mixerOut: this.mixerOut,
			attOut:   this.attOut,
			wheelOut: this.wheelOut,
		};
	}
}
