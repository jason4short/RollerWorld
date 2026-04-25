// Nav — the highest layer. Knows where the bot is, where it's going, and
// nothing about how the bot moves.
//
// Inputs:  current world position + heading, a list of waypoints (or a
//          stick reading in fly-by-wire mode).
// Outputs: { vel_target_body, heading_target } — a body-frame forward
//          velocity command and a target heading.
//
// What Nav does NOT do:
//   - tilt math               (that's the Mixer)
//   - angle → torque          (Attitude)
//   - PWM                     (Wheels)
//
// Two strategies for waypoint nav, picked via `mode`:
//
//   profile  — square-root braking curve. Good for real driving: full speed
//              far from the target, smooth deceleration to zero on arrival.
//              v_brake = √(2·a_max·distance) capped at v_max.
//   pd       — simple Kp·distance. Educational; shows what overshoot looks
//              like when you don't pre-compute a brake profile.
//
// Heading: the bot always faces its target. The signed projection of the
// world error onto the body's forward axis lets the bot brake (or briefly
// reverse for small overshoots) without pirouetting 180° to chase a target
// that's right behind it. This is the trick that makes a single waypoint
// loop work on a real bot — see ArduBalance.pde:1323 in the reference fw.

export class Nav {
	constructor() {
		this.target_x         = 0;
		this.target_z         = 0;
		this.mode             = 'profile';   // 'pd' | 'profile'
		this.fbw_heading_ref  = 0;           // integrated stick.yaw → heading_target

		// Diagnostics (exposed for plotting / introspection)
		this.distance_err     = 0;
		this.heading_err      = 0;
		this.vel_target_last  = 0;
	}

	reset() {
		this.fbw_heading_ref = 0;
		this.distance_err    = 0;
		this.heading_err     = 0;
		this.vel_target_last = 0;
	}

	// ── Auto mode ──────────────────────────────────────────────────────────
	// command: ignored (auto uses target_x, target_z)
	// sensors: { x, z, heading }
	updateAuto(sensors, gains) {
		const { v_max, a_max, linear_zone, lookahead, yaw_disable_radius = 0.2 } = gains;

		const dx = this.target_x - sensors.x;
		const dz = this.target_z - (sensors.z ?? 0);
		const distance_2d = Math.sqrt(dx * dx + dz * dz);

		// Project world error onto body-forward axis. Sign tells us "ahead vs
		// behind"; the bot can reverse a small overshoot instead of spinning.
		const projected = dx * Math.cos(sensors.heading) - dz * Math.sin(sensors.heading);
		this.distance_err = projected;

		// Heading target = bearing to point.
		const heading_target = Math.atan2(-dz, dx);
		this.heading_err = this._wrap(heading_target - sensors.heading);

		// Drive command. Three guards before we ask for forward velocity:
		//   1. Inside the deadzone — stop, let yaw align without thrashing.
		//   2. More than 90° off heading — stop, rotate in place first.
		//   3. Within ±90° — soften by cos(heading_err) so drive ramps up
		//      smoothly as the bot completes the turn (no jolt at boundary).
		let vel_target;
		if (distance_2d < yaw_disable_radius) {
			vel_target = 0;
		} else if (Math.abs(this.heading_err) > Math.PI / 2) {
			vel_target = 0;
		} else {
			const align = Math.cos(this.heading_err);
			const raw   = this.mode === 'profile'
				? this._profileSpeed(projected, sensors, gains)
				: gains.Kp_nav * projected;
			vel_target = Math.max(0, align * raw);
		}

		this.vel_target_last = vel_target;
		return { vel_target_body: vel_target, heading_target };
	}

	// ── Fly-by-wire mode ───────────────────────────────────────────────────
	// stick.fwd, stick.yaw ∈ [-1, 1]
	// Heading is integrated from stick.yaw; the bot has no absolute heading
	// reference in FBW (no waypoint), so we maintain our own.
	updateFbw(sensors, stick, gains, dt) {
		const vel_target = (stick.fwd ?? 0) * gains.v_max;

		const yaw_rate_cmd = (stick.yaw ?? 0) * (gains.MaxYawRate ?? 1.5);
		this.fbw_heading_ref = this._wrap(this.fbw_heading_ref + yaw_rate_cmd * dt);

		this.vel_target_last = vel_target;
		this.heading_err     = 0;
		this.distance_err    = 0;
		return { vel_target_body: vel_target, heading_target: this.fbw_heading_ref };
	}

	// ── Tilt mode (debug, no nav at all) ───────────────────────────────────
	// Used by raw arrow-key piloting. Bypasses velocity feedback entirely;
	// caller injects pitch_target into Attitude directly. Returns null so
	// the orchestrator knows to skip Mixer.
	updateTilt() { return null; }

	// ── Internals ──────────────────────────────────────────────────────────
	_profileSpeed(projected, sensors, gains) {
		const { v_max, a_max, linear_zone, lookahead } = gains;
		// Lookahead in the direction of motion: start braking earlier when
		// already moving fast in the projection's direction.
		const eff = projected - sensors.vel_cart * lookahead;
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

	_wrap(angle) {
		while (angle >  Math.PI) angle -= 2 * Math.PI;
		while (angle < -Math.PI) angle += 2 * Math.PI;
		return angle;
	}

	setTarget(x, z) { this.target_x = x; this.target_z = z; }
}
