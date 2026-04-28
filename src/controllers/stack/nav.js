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
//
// Rule vs NN
// ----------
// Nav is the messiest of the four layers to learn. It has discrete logic
// (deadzone gate, ±90° heading gate, alignment cosine softening) which an
// MLP smooths over rather than reproducing exactly. NN substitution is
// supported for auto mode only — FBW stays rule-based since stick → vel/
// heading is essentially trivial. Honest limitation, surfaced via comments
// and the "Use NN" toggle being scoped to the Auto path.

const NAV_INPUT_SCALES  = [1 / 5, 1 / 5, 1 / Math.PI, 1 / 3];   // dx, dz, heading, vel_cart
const NAV_OUTPUT_SCALES = [1 / 3, 1 / Math.PI];                  // vel_target_body, heading_err

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

		// NN pluggability for the Auto path.
		this.autoMode = 'rule';   // 'rule' | 'nn'
		this.autoMlp  = null;
	}

	setAutoMode(mode, mlp = null) {
		this.autoMode = mode;
		this.autoMlp  = mlp;
	}

	static get NAV_INPUT_SCALES()  { return NAV_INPUT_SCALES; }
	static get NAV_OUTPUT_SCALES() { return NAV_OUTPUT_SCALES; }

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
		const dx = this.target_x - sensors.x;
		const dz = this.target_z - (sensors.z ?? 0);

		if (this.autoMode === 'nn' && this.autoMlp) {
			return this._autoNN({ dx, dz, heading: sensors.heading, vel_cart: sensors.vel_cart ?? 0 });
		}

		const out = Nav.computeAutoRule(
			{ dx, dz, heading: sensors.heading, vel_cart: sensors.vel_cart ?? 0 },
			gains, this.mode,
		);
		this.distance_err   = out.distance_err;
		this.heading_err    = out.heading_err;
		this.vel_target_last = out.vel_target_body;
		// heading_rate_ff = 0 in auto mode: the bot tracks heading_target by
		// closing heading_err; the implicit rate is small and the PD handles it.
		return {
			vel_target_body: out.vel_target_body,
			heading_target:  out.heading_target,
			heading_rate_ff: 0,
		};
	}

	// Stateless Auto-mode logic. Same single-source-of-truth pattern as the
	// other layers — rule branch and trainer call this. Emits the diagnostic
	// fields too so the live updateAuto can populate plotting state.
	static computeAutoRule({ dx, dz, heading, vel_cart }, gains, mode = 'profile') {
		const { v_max = 3, a_max = 1.5, linear_zone = 0.05, lookahead = 0.3,
		        Kp_nav = 0.15, yaw_disable_radius = 0.2 } = gains;

		const distance_2d = Math.sqrt(dx * dx + dz * dz);
		const projected   = dx * Math.cos(heading) - dz * Math.sin(heading);

		const heading_target = Math.atan2(-dz, dx);
		let heading_err = heading_target - heading;
		while (heading_err >  Math.PI) heading_err -= 2 * Math.PI;
		while (heading_err < -Math.PI) heading_err += 2 * Math.PI;

		let vel_target_body;
		if (distance_2d < yaw_disable_radius) {
			vel_target_body = 0;
		} else if (Math.abs(heading_err) > Math.PI / 2) {
			vel_target_body = 0;
		} else {
			const align = Math.cos(heading_err);
			let raw;
			if (mode === 'profile') {
				const eff = projected - vel_cart * lookahead;
				const absErr = Math.abs(eff);
				let v_brake;
				if (absErr > linear_zone) {
					v_brake = Math.sqrt(2 * a_max * absErr);
				} else {
					const slope = Math.sqrt(2 * a_max / linear_zone);
					v_brake = slope * absErr;
				}
				raw = Math.sign(eff) * Math.min(v_brake, v_max);
			} else {
				raw = Kp_nav * projected;
			}
			vel_target_body = Math.max(0, align * raw);
		}

		return { vel_target_body, heading_target, heading_err, distance_err: projected };
	}

	_autoNN({ dx, dz, heading, vel_cart }) {
		const inS  = NAV_INPUT_SCALES;
		const outS = NAV_OUTPUT_SCALES;
		const x = [
			dx       * inS[0],
			dz       * inS[1],
			heading  * inS[2],
			vel_cart * inS[3],
		];
		const y = this.autoMlp.forward(x);
		const vel_target_body = y[0] / outS[0];
		const heading_err     = y[1] / outS[1];
		// Reconstruct heading_target from current heading + predicted err.
		const heading_target = this._wrap(heading + heading_err);
		this.heading_err     = heading_err;
		this.distance_err    = Math.sqrt(dx * dx + dz * dz);   // fallback: unsigned
		this.vel_target_last = vel_target_body;
		return { vel_target_body, heading_target, heading_rate_ff: 0 };
	}

	// ── Fly-by-wire mode ───────────────────────────────────────────────────
	// stick.fwd, stick.yaw ∈ [-1, 1]
	// Heading is integrated from stick.yaw; the bot has no absolute heading
	// reference in FBW (no waypoint), so we maintain our own.
	//
	// heading_rate_ff: the rate we're integrating into the reference. Attitude
	// uses this as a feedforward term so the bot rotates smoothly between
	// Nav firings instead of stepping (Nav at 60 Hz, Attitude at 100 Hz —
	// without FF the heading_target was a 60 Hz staircase and Attitude would
	// catch up in a few ms then idle until the next step, producing a
	// rotate-stop-rotate stutter while the user held the yaw stick).
	updateFBW(sensors, stick, gains, dt) {
		const vel_target = (stick.fwd ?? 0) * gains.v_max;

		const yaw_rate_cmd = (stick.yaw ?? 0) * (gains.MaxYawRate ?? 1.5);
		this.fbw_heading_ref = this._wrap(this.fbw_heading_ref + yaw_rate_cmd * dt);

		this.vel_target_last = vel_target;
		this.heading_err     = 0;
		this.distance_err    = 0;
		return {
			vel_target_body: vel_target,
			heading_target:  this.fbw_heading_ref,
			heading_rate_ff: yaw_rate_cmd,
		};
	}

	// ── Tilt mode (debug, no nav at all) ───────────────────────────────────
	// Used by raw arrow-key piloting. Bypasses velocity feedback entirely;
	// caller injects pitch_target into Attitude directly. Returns null so
	// the orchestrator knows to skip Mixer.
	updateTilt() { return null; }

	// ── Internals ──────────────────────────────────────────────────────────
	_wrap(angle) {
		while (angle >  Math.PI) angle -= 2 * Math.PI;
		while (angle < -Math.PI) angle += 2 * Math.PI;
		return angle;
	}

	setTarget(x, z) { this.target_x = x; this.target_z = z; }
}
