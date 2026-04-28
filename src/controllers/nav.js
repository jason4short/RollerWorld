// 2D Navigation — emits a body-frame speed setpoint and an absolute
// target heading. The controller (ArduBalance / NN / Cascade) translates
// those into tilt and yaw torque internally; Nav stays a pure navigator
// with no knowledge of how the bot moves.
//
// Ported from the ArduBalance firmware approach (ArduBalance.pde:1323):
//
//   distance_error = long_error * cos_yaw + lat_error * sin_yaw
//
// Project the world-frame (target − bot) vector onto the bot's forward
// axis to get a SIGNED distance. Positive = target is ahead; negative =
// behind. That signed distance feeds the same square-root braking
// profile used in 1D, plus an alignment cosine so the bot ramps up
// drive as it finishes turning instead of jolting at ±90°.
//
// Heading is just the bearing to the target. The controller's own yaw
// loop closes on it (or its setCruise's stabilize_yaw helper does).

export class NavController {
	constructor() {
		this.target_x  = 0;
		this.target_z  = 0;
		this.vel_lpf   = 0;
		this.mode      = 'profile';   // 'pd' | 'profile'

		// Diagnostic state (exposed for plotting)
		this.err_last         = 0;    // signed projected distance
		this.heading_err_last = 0;
		this.target_heading_last = 0;
		this.vel_desired_last = 0;
	}

	reset() { this.vel_lpf = 0; this.vel_desired_last = 0; }

	// Rate-limit vel_desired by a_max so a step nav input (clicked target,
	// "arrived" branch zeroing) doesn't propagate as a step into the
	// controller's speed reference. The bot can't accelerate faster than
	// a_max anyway — this just stops asking it to.
	_slewVelDesired(target, gains, dt) {
		const dv_max	= (gains.a_max ?? 1.5) * dt;
		const prev		= this.vel_desired_last ?? 0;
		if (target > prev + dv_max) return prev + dv_max;
		if (target < prev - dv_max) return prev - dv_max;
		return target;
	}

	// Returns { speed, heading } — body-frame forward speed and the
	// absolute heading to face. The controller is responsible for
	// translating those into actuator commands via its setCruise.
	update(sensors, gains, dt = 1 / 60) {
		// LPF the noisy encoder velocity for the alignment-aware drive
		// shaping below.
		const time_constant = 0.1;
		const alpha         = dt / (time_constant + dt);
		this.vel_lpf        = (1 - alpha) * this.vel_lpf + alpha * sensors.vel_bot;

		// World-frame error to target.
		const dx = this.target_x - sensors.x;
		const dz = this.target_z - (sensors.z ?? 0);
		const distance_2d = Math.sqrt(dx * dx + dz * dz);

		// Signed projection onto bot's forward axis — positive ahead, negative
		// behind. Lets the bot reverse for small overshoots instead of
		// pirouetting 180°.
		const projected = dx * Math.cos(sensors.heading) - dz * Math.sin(sensors.heading);
		this.err_last = projected;

		// Always face the target — no reverse-driving shortcut.
		const target_heading = Math.atan2(-dz, dx);
		let heading_err = target_heading - sensors.heading;
		while (heading_err >  Math.PI) heading_err -= 2 * Math.PI;
		while (heading_err < -Math.PI) heading_err += 2 * Math.PI;
		this.heading_err_last    = heading_err;
		this.target_heading_last = target_heading;

		// Drive command: only drive forward when roughly aligned. Outside
		// ±90°, hold speed at zero and let the controller's yaw loop
		// rotate in place. Within ±90°, soften by cos(heading_err) so the
		// drive ramps up as the bot completes the turn.
		const { yaw_disable_radius = 0.2 } = gains;
		let speed_target;
		if (distance_2d < yaw_disable_radius) {
			speed_target = 0;
		} else if (Math.abs(heading_err) > Math.PI / 2) {
			speed_target = 0;
		} else {
			const align = Math.cos(heading_err);
			const raw = this.mode === 'profile'
				? this._profileSpeed(distance_2d, gains)
				: this._pdSpeed(distance_2d, gains);
			speed_target = Math.max(0, align * raw);
		}
		const speed = this._slewVelDesired(speed_target, gains, dt);
		this.vel_desired_last = speed;

		return { speed, heading: target_heading };
	}

	_pdSpeed(projected, gains) {
		const { Kp_nav } = gains;
		return Kp_nav * projected;
	}

	_profileSpeed(projected, gains) {
		const { v_max, a_max, linear_zone, lookahead } = gains;
		// Lookahead in the direction of motion — start braking earlier
		// given current speed.
		const eff = projected - this.vel_lpf * lookahead;
		const absErr = Math.abs(eff);
		let v_brake;
		if (absErr > linear_zone) {
			v_brake = Math.sqrt(2 * a_max * absErr);
		} else {
			const slope = Math.sqrt(2 * a_max / linear_zone);
			v_brake = slope * absErr;
		}
		return Math.sign(eff) * Math.min(v_brake, v_max);
	}
}
