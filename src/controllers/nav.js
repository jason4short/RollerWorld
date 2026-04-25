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
		this.v_lpf     = 0;
		this.mode      = 'profile';   // 'pd' | 'profile'
		// Diagnostic state (exposed for plotting)
		this.err_last         = 0;    // signed projected distance
		this.tilt_last        = 0;
		this.yawRate_last     = 0;
		this.heading_err_last = 0;
		this.v_desired_last   = 0;
	}

	reset() { this.v_lpf = 0; this.v_desired_last = 0; }

	// Rate-limit v_desired by a_max so a step nav input (clicked target,
	// stick yank, "arrived" branch zeroing) doesn't propagate as a step
	// into the pitch controller's target_angle. The bot can't physically
	// accelerate faster than a_max anyway — this just stops asking it to.
	_slewVDesired(target, gains, dt) {
		const dv_max	= (gains.a_max ?? 1.5) * dt;
		const prev		= this.v_desired_last ?? 0;
		if (target > prev + dv_max) return prev + dv_max;
		if (target < prev - dv_max) return prev - dv_max;
		return target;
	}

	update(sensors, gains, dt = 1 / 60) {
		// LPF the noisy encoder velocity for damping use.
		const tau   = 0.1;
		const alpha = dt / (tau + dt);
		this.v_lpf  = (1 - alpha) * this.v_lpf + alpha * sensors.v;

		// World-frame error to target.
		const dx = this.target_x - sensors.x;
		const dz = this.target_z - (sensors.z ?? 0);
		const distance_2d = Math.sqrt(dx * dx + dz * dz);

		// Project onto bot's forward axis — signed (positive ahead, negative
		// behind). This lets the bot reverse for small overshoots instead of
		// pirouetting 180°.
		const projected = dx * Math.cos(sensors.psi) - dz * Math.sin(sensors.psi);
		this.err_last = projected;

		// Alignment to the bot-target line. The bot can drive forward OR
		// backward along that line, so we want yaw to align to whichever
		// orientation is closer — facing the target, OR facing 180° away
		// from it. With this, an overshoot becomes a small backward drive
		// (zero yaw), not a 180° pirouette.
		const target_heading = Math.atan2(-dz, dx);
		let heading_err = target_heading - sensors.psi;
		while (heading_err >  Math.PI) heading_err -= 2 * Math.PI;
		while (heading_err < -Math.PI) heading_err += 2 * Math.PI;
		// If aligning to target_heading is more than 90° away, flip — aligning
		// to target_heading + π is closer (and the projection-based drive
		// will then naturally command negative body velocity to reverse
		// toward the target).
		if (heading_err >  Math.PI / 2) heading_err -= Math.PI;
		if (heading_err < -Math.PI / 2) heading_err += Math.PI;
		this.heading_err_last = heading_err;

		// --- Yaw command --------------------------------------------------
		// P on heading error, fading out near the target. No speed-scaling
		// (the orbit failure that triggered that hack came from a different
		// bug in the forward command — sign-flipping projected distance —
		// which is now fixed below by alignment-gated forward drive).
		const { Kheading = 2, MaxYawRate = 1.5, yaw_disable_radius = 0.2 } = gains;
		const radius_scale = Math.min(1, Math.max(0,
			(distance_2d - yaw_disable_radius) / yaw_disable_radius));
		let yawRate = Kheading * heading_err * radius_scale;
		if (yawRate >  MaxYawRate) yawRate =  MaxYawRate;
		if (yawRate < -MaxYawRate) yawRate = -MaxYawRate;
		this.yawRate_last = yawRate;

		// --- Drive command ------------------------------------------------
		// Body-frame velocity setpoint comes from the signed projection of
		// the world error onto the bot's forward axis — positive ahead,
		// negative behind. With heading_err bounded to ±90° (above), this
		// projection's sign is now well-behaved during rotation.
		let v_target;
		if (distance_2d < yaw_disable_radius) {
			// "Arrived" — brake to zero, don't chase further.
			v_target = 0;
		} else if (this.mode === 'profile') {
			v_target = this._profileSpeed(projected, gains);
		} else {
			v_target = this._pdSpeed(projected, gains);
		}
		const v_desired = this._slewVDesired(v_target, gains, dt);
		this.v_desired_last = v_desired;
		let tilt = gains.Kvel * (v_desired - this.v_lpf);

		const lim = gains.tiltLimit;
		if (tilt >  lim) tilt =  lim;
		if (tilt < -lim) tilt = -lim;
		this.tilt_last = tilt;

		return { tilt, yawRate };
	}

	// Fly-by-wire: pilot stick directly sets the body-frame velocity setpoint
	// and yaw rate. The same v_lpf / Kvel inner-loop math from update() runs,
	// so centering the stick brakes hard via `Kvel * (0 - v_lpf)`.
	updateFbw(sensors, stick, gains, dt = 1 / 60) {
		const tau	= 0.1;
		const alpha	= dt / (tau + dt);
		this.v_lpf	= (1 - alpha) * this.v_lpf + alpha * sensors.v;

		const v_target			= (stick.fwd ?? 0) * gains.v_max;
		const v_desired			= this._slewVDesired(v_target, gains, dt);
		this.v_desired_last		= v_desired;
		this.err_last			= 0;
		this.heading_err_last	= 0;

		let tilt = gains.Kvel * (v_desired - this.v_lpf);
		const lim = gains.tiltLimit;
		if (tilt >  lim) tilt =  lim;
		if (tilt < -lim) tilt = -lim;
		this.tilt_last = tilt;

		const maxYaw	= gains.MaxYawRate;
		let yawRate		= (stick.yaw ?? 0) * maxYaw;
		if (yawRate >  maxYaw) yawRate =  maxYaw;
		if (yawRate < -maxYaw) yawRate = -maxYaw;
		this.yawRate_last = yawRate;

		return { tilt, yawRate };
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
		const eff = projected - this.v_lpf * lookahead;
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
