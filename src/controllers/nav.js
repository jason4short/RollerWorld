// 2D Navigation outer-outer loop.
//
// Ported from the ArduBalance firmware's approach (ArduBalance.pde line 1323):
//
//   distance_error = long_error * cos_yaw + lat_error * sin_yaw
//
// We project the world-frame (target − bot) vector onto the bot's forward
// axis to get a SIGNED distance. Positive = target is ahead; negative =
// behind; zero = perpendicular. That signed distance becomes the input to
// the same square-root braking profile we already use in 1D — and then a
// pitch tilt that drives forward velocity to match the profile's output.
//
// Yaw runs as a separate channel: target heading is the bearing to the
// target, and a P loop on heading error commands a yaw rate.
//
// The projection is the secret. There's no need for cos-scaling, gating, or
// special "target behind me" logic — the sign of the projected distance
// flips smoothly as the bot turns, so the controller naturally goes through
// brake → stop → forward as the heading swings around.

export class NavController {
	constructor() {
		this.target_x  = 0;
		this.target_z  = 0;
		this.vel_lpf     = 0;
		this.mode      = 'profile';   // 'pd' | 'profile'
		// Diagnostic state (exposed for plotting)
		this.err_last           = 0;    // signed projected distance
		this.tilt_last          = 0;
		this.yaw_rate_last      = 0;
		this.heading_err_last   = 0;
		this.vel_desired_last     = 0;
	}

	reset() { this.vel_lpf = 0; this.vel_desired_last = 0; }

	// Rate-limit vel_desired by a_max so a step nav input (clicked target,
	// stick yank, "arrived" branch zeroing) doesn't propagate as a step
	// into the pitch controller's target_angle. The bot can't physically
	// accelerate faster than a_max anyway — this just stops asking it to.
	_slewVelDesired(target, gains, dt) {
		const dv_max	= (gains.a_max ?? 1.5) * dt;
		const prev		= this.vel_desired_last ?? 0;
		if (target > prev + dv_max) return prev + dv_max;
		if (target < prev - dv_max) return prev - dv_max;
		return target;
	}

	update(sensors, gains, dt = 1 / 60) {
		// LPF the noisy encoder velocity for damping use.
		const time_constant = 0.1;
		const alpha         = dt / (time_constant + dt);
		this.vel_lpf          = (1 - alpha) * this.vel_lpf + alpha * sensors.vel_cart;

		// World-frame error to target.
		const dx = this.target_x - sensors.x;
		const dz = this.target_z - (sensors.z ?? 0);
		const distance_2d = Math.sqrt(dx * dx + dz * dz);

		// Project onto bot's forward axis — signed (positive ahead, negative
		// behind). This lets the bot reverse for small overshoots instead of
		// pirouetting 180°.
		const projected = dx * Math.cos(sensors.heading) - dz * Math.sin(sensors.heading);
		this.err_last = projected;

		// Always face the target — no reverse-driving shortcut. Bot rotates
		// the full way around if needed instead of backing up.
		const target_heading = Math.atan2(-dz, dx);
		let heading_err = target_heading - sensors.heading;
		while (heading_err >  Math.PI) heading_err -= 2 * Math.PI;
		while (heading_err < -Math.PI) heading_err += 2 * Math.PI;
		this.heading_err_last = heading_err;

		// --- Yaw command --------------------------------------------------
		// P on heading error, fading out near the target. No speed-scaling
		// (the orbit failure that triggered that hack came from a different
		// bug in the forward command — sign-flipping projected distance —
		// which is now fixed below by alignment-gated forward drive).
		const { Kheading = 2, MaxYawRate = 1.5, yaw_disable_radius = 0.2 } = gains;
		const radius_scale = Math.min(1, Math.max(0,
			(distance_2d - yaw_disable_radius) / yaw_disable_radius));
		let yaw_rate = Kheading * heading_err * radius_scale;
		if (yaw_rate >  MaxYawRate) yaw_rate =  MaxYawRate;
		if (yaw_rate < -MaxYawRate) yaw_rate = -MaxYawRate;
		this.yaw_rate_last = yaw_rate;

		// --- Drive command ------------------------------------------------
		// Body-frame velocity setpoint comes from the signed projection of
		// the world error onto the bot's forward axis — positive ahead,
		// negative behind. With heading_err bounded to ±90° (above), this
		// projection's sign is now well-behaved during rotation.
		// Only drive forward when roughly aligned with the target. Outside
		// ±90°, hold velocity at zero and let the yaw loop rotate in place.
		// Within ±90°, soften by cos(heading_err) so the drive ramps up as
		// the bot completes the turn — no jolt at the alignment boundary.
		let vel_target;
		if (distance_2d < yaw_disable_radius) {
			vel_target = 0;
		} else if (Math.abs(heading_err) > Math.PI / 2) {
			vel_target = 0;
		} else {
			const align = Math.cos(heading_err);
			const raw = this.mode === 'profile'
				? this._profileSpeed(distance_2d, gains)
				: this._pdSpeed(distance_2d, gains);
			vel_target = Math.max(0, align * raw);
		}
		const vel_desired = this._slewVelDesired(vel_target, gains, dt);
		this.vel_desired_last = vel_desired;
		let tilt = gains.Kvel * (vel_desired - this.vel_lpf);

		const lim = gains.tiltLimit;
		if (tilt >  lim) tilt =  lim;
		if (tilt < -lim) tilt = -lim;
		this.tilt_last = tilt;

		return { tilt, yaw_rate };
	}

	// Fly-by-wire: pilot stick directly sets the body-frame velocity setpoint
	// and yaw rate. The same vel_lpf / Kvel inner-loop math from update() runs,
	// so centering the stick brakes hard via `Kvel * (0 - vel_lpf)`.
	updateFBW(sensors, stick, gains, dt = 1 / 60) {
		const time_constant 	= 0.1;
		const alpha				= dt / (time_constant + dt);
		this.vel_lpf			= (1 - alpha) * this.vel_lpf + alpha * sensors.vel_cart;

		const vel_target		= (stick.fwd ?? 0) * gains.v_max;
		const vel_desired		= this._slewVelDesired(vel_target, gains, dt);
		this.vel_desired_last	= vel_desired;
		this.err_last			= 0;
		this.heading_err_last	= 0;

		let tilt = gains.Kvel * (vel_desired - this.vel_lpf);
		const lim = gains.tiltLimit;
		if (tilt >  lim) tilt =  lim;
		if (tilt < -lim) tilt = -lim;
		this.tilt_last = tilt;

		const max_yaw	= gains.MaxYawRate;
		let yaw_rate	= (stick.yaw ?? 0) * max_yaw;
		if (yaw_rate >  max_yaw) yaw_rate =  max_yaw;
		if (yaw_rate < -max_yaw) yaw_rate = -max_yaw;
		this.yaw_rate_last = yaw_rate;

		return { tilt, yaw_rate };
	}

	_pdSpeed(projected, gains) {
		// Signed: matches projection's sign for forward/backward.
		const { Kp_nav } = gains;
		return Kp_nav * projected;
	}

	_profileSpeed(projected, gains) {
		const { v_max, a_max, linear_zone, lookahead } = gains;
		// Lookahead in the direction of motion (subtract velocity·lookahead so
		// we start braking earlier given current speed).
		const eff = projected - this.vel_lpf * lookahead;
		const absErr = Math.abs(eff);
		let v_brake;
		if (absErr > linear_zone) {
			v_brake = Math.sqrt(2 * a_max * absErr);
		} else {
			const slope = Math.sqrt(2 * a_max / linear_zone);
			v_brake = slope * absErr;
		}
		// Sign comes from the signed projection — ahead → forward, behind → reverse.
		return Math.sign(eff) * Math.min(v_brake, v_max);
	}
}
