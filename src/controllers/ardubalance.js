import { PWMTable } from '../physics/pwm-table.js';

// Mode enums — match the firmware (ArduBalance/defines.h:19-31). Pilot
// sets these via setAttitude / setCruise / setAuto, and the controller's
// update_roll_pitch_mode / update_yaw_mode switch on them just like the
// firmware switches on roll_pitch_mode / yaw_mode.
export const ROLL_PITCH_STABLE = 0;   // pilot's target_angle (raw)
export const ROLL_PITCH_AUTO   = 1;   // waypoint nav, controller-internal
export const ROLL_PITCH_FBW    = 3;   // pilot's desired_speed

export const YAW_HOLD            = 0; // hold nav_yaw
export const YAW_ACRO            = 1; // pilot rate stick
export const YAW_LOOK_AT_NEXT_WP = 2; // nav_yaw = wp_bearing

// ArduBalance — port of the firmware's main control flow, kept as close
// to the original ArduBalance.pde structure as possible so the codepaths
// in this file map line-for-line to the .pde files in /ArduBalance.
//
// Per-tick flow (mirrors the firmware's main loop):
//
//   update_roll_pitch_mode()   — sets this.pitch_speed
//                                ArduBalance.pde:1245 (FBW), 1357 (AUTO)
//   update_yaw_mode()          — sets this.yaw_speed
//                                ArduBalance.pde:1209
//   update_servos()            — pitch_speed + yaw_speed → per-wheel PWM
//                                motors.pde:60
//
// Sub-helpers (one per firmware function):
//
//   get_stabilize_pitch        — bal_P · angle_err + bal_D · gyro
//                                Attitude.pde:14
//   get_velocity_pitch         — p_vel · wheel.speed
//                                Attitude.pde:40
//   get_nav_pitch              — speed PID + ff_out, returns nav_out − ff_out
//                                navigation.pde:160
//   get_stabilize_yaw          — heading P → yaw_speed in PWM units
//                                Attitude.pde:48
//   get_dist_err               — projected world error onto forward axis
//                                navigation.pde:183
//   convert_distance_to_speed  — dist_err → desired_ticks (+ accel limit,
//                                + min_speed floor); first three lines of
//                                get_nav_pitch (navigation.pde:167)
//   get_wp_bearing             — bearing from current pose to next_WP
//                                navigation.pde:93
//
// Two pilot modes match the firmware's split:
//
//   ROLL_PITCH_FBW  (setCruise)  — pilot hands desired_speed + nav_yaw
//   ROLL_PITCH_AUTO (setAuto)    — pilot hands a waypoint; controller
//                                  computes get_dist_err, convert(), and
//                                  derives desired_ticks itself
//
// In both, target_angle stays vertical (= 0). nav_out biases pitch_speed
// and the bot leans dynamically into the drive — same trick the firmware
// uses (ff_out makes pitch_speed land on the side that tips the bot
// forward; balance loop catches the lean and the cart accelerates).
//
// Sign note: signs below assume our sim convention (+pitch = bob leaning
// in +x = forward; +PWM → +force → cart goes forward). The firmware lived
// in a different unit system and the wheel-mixer kP could be negative;
// here we collapse all that into a single "+PWM = forward" assumption.
// If forward driving feels reversed, flip the sign of get_nav_pitch's
// return (or invert ff_per_mps in the gains panel).

export class ArduBalanceController {
	constructor() {
		// Mode flags — set by the pilot via setAttitude/setCruise/setAuto.
		// update_roll_pitch_mode and update_yaw_mode switch on these.
		this.roll_pitch_mode = ROLL_PITCH_STABLE;
		this.yaw_mode        = YAW_ACRO;

		// Pilot-set state ----------------------------------------------------
		this.target_angle    = 0;        // raw-mode pitch target (rad)
		this.yaw_rate_target = 0;        // raw-mode pilot yaw rate (rad/s)

		// FBW state — desired speed and absolute heading from pilot stick.
		this.cruise_speed    = 0;        // m/s
		this.cruise_heading  = 0;        // rad

		// next_WP — world target the controller drives toward. AUTO sets
		// it explicitly via setAuto; FBW and STABLE snapshot it to
		// current_loc each tick while the pilot is actively driving, so
		// when the pilot releases, next_WP is frozen at "where you
		// stopped pushing" and get_dist_err pulls the bot back to it.
		// Same variable plays both roles in the firmware (next_WP is
		// next_WP whether you're in AUTO or FBW); keeping it that way.
		this.target_x        = 0;
		this.target_z        = 0;
		this.min_speed       = 0;        // floor for desired_ticks (m/s)

		this.pwmTable        = new PWMTable();   // FF curve (linear until calibrated)

		this.reset();
	}

	reset() {
		// Per-tick scratch ----------------------------------------------------
		this.pitch_speed        = 0;     // output of update_roll_pitch_mode
		this.yaw_speed          = 0;     // output of update_yaw_mode
		this.pwm_left           = 0;     // output of update_servos
		this.pwm_right          = 0;
		this.nav_yaw            = 0;     // current yaw target (rad)

		// Persistent state ----------------------------------------------------
		this.balance_offset     = 0;     // learned IMU zero-offset (rad)
		this.speed_I            = 0;     // get_nav_pitch integrator
		this.last_vel_cart_meas = 0;     // get_nav_pitch D-on-measurement
		this.speed_d_lpf        = 0;     // low-passed derivative
		this.desired_ticks_old  = 0;     // get_nav_pitch accel limit memory

		// Diagnostics (drained by telemetry / plotter) -----------------------
		this.dist_err           = 0;     // most recent get_dist_err
		this.desired_ticks      = 0;     // most recent desired_ticks
		this.lastPWM            = 0;     // larger-magnitude wheel (firmware-style scalar)
		this.lastForceFwd       = 0;
		this.lastTorqueYaw      = 0;
	}

	// Pilot interface ---------------------------------------------------------
	// Each setter selects the firmware mode pair (roll_pitch_mode + yaw_mode)
	// matching the original code:
	//
	//   raw keyboard stick  → STABLE + ACRO              (pilot owns angle + rate)
	//   FBW joystick        → FBW    + HOLD              (pilot owns speed + heading)
	//   auto waypoint       → AUTO   + LOOK_AT_NEXT_WP   (controller owns everything)
	setAttitude(tilt, yawRate) {
		this.roll_pitch_mode = ROLL_PITCH_STABLE;
		this.yaw_mode        = YAW_ACRO;
		this.target_angle    = tilt    ?? 0;
		this.yaw_rate_target = yawRate ?? 0;
	}

	setCruise(speed, heading) {
		this.roll_pitch_mode = ROLL_PITCH_FBW;
		this.yaw_mode        = YAW_HOLD;
		this.cruise_speed    = speed   ?? 0;
		this.cruise_heading  = heading ?? 0;
		this.target_angle    = 0;
	}

	setAuto(target_x, target_z, min_speed = 0) {
		this.roll_pitch_mode = ROLL_PITCH_AUTO;
		this.yaw_mode        = YAW_LOOK_AT_NEXT_WP;
		this.target_x        = target_x ?? 0;
		this.target_z        = target_z ?? 0;
		this.min_speed       = min_speed ?? 0;
		this.target_angle    = 0;
	}

	// Single-rate firmware port — no slow loop work. The whole control flow
	// runs in fastLoop, mirroring the firmware's main loop.
	slowLoop() {}

	// Main loop port. Mirrors ArduBalance.pde's per-tick sequence:
	//
	//   update_roll_pitch_mode();
	//   update_yaw_mode();
	//   update_servos();
	//
	// Returns the per-wheel torque shape Rollerbot consumes.
	fastLoop(sensors, gains, dt, motor) {
		this.update_roll_pitch_mode(sensors, gains, dt);
		this.update_yaw_mode(sensors, gains);
		return this.update_servos(sensors, gains, motor);
	}

	// ──────────────────────────────────────────────────────────────────────
	// update_roll_pitch_mode  — ArduBalance.pde:1245
	// ──────────────────────────────────────────────────────────────────────
	// Each case is an inlined port of the firmware's matching block —
	// same components, same summation order, same sign. Read top-to-bottom
	// against the .pde file.
	update_roll_pitch_mode(sensors, gains, dt) {
		let pitch_speed = 0;

		switch (this.roll_pitch_mode) {

			// ──────────────────────────────────────────────────────────
			// ROLL_PITCH_STABLE  — pilot owns the target angle.
			// ──────────────────────────────────────────────────────────
			// Three-line shape mirroring the firmware's AUTO case below
			// (ArduBalance.pde:1357), with the FBW position-hold snapshot
			// (lines 1316-1320) wrapped around it: while pilot is leaning,
			// next_WP trails the bot; when pilot releases, snapshot
			// freezes and get_nav_pitch(0, get_dist_err()) drives back.
			// Without this the bot rolls away on encoder bias even though
			// the pitch loop is balancing it.
			case ROLL_PITCH_STABLE:
				if (this.target_angle !== 0) {
					this.target_x = sensors.x;
					this.target_z = sensors.z ?? 0;
					this.speed_I  = 0;
				}
				this.dist_err = this.get_dist_err(sensors);

				// in this mode we command the target angle
				pitch_speed  = this.get_stabilize_pitch(this.target_angle, sensors, gains, dt);
				// speed control:
				pitch_speed += this.get_velocity_pitch(sensors, gains);
				// maintain location:
				pitch_speed += this.get_nav_pitch(0, this.dist_err, sensors, gains, dt);
				break;

			// ──────────────────────────────────────────────────────────
			// ROLL_PITCH_FBW  — ArduBalance.pde:1314
			// ──────────────────────────────────────────────────────────
			// Literal port of the firmware case, lines 1316-1354. Stick
			// deflected → snapshot next_WP at current_loc and reset PID I.
			// Stick centered → use distance_error as desired_speed (so
			// the bot drives back to where you let go). Then four named
			// contributions summed line 1354:
			//
			//   pitch_speed = bal_out + vel_out + nav_out − ff_out
			//
			// Inlined here (rather than calling get_nav_pitch like AUTO)
			// because FBW computes desired_speed from the stick instead
			// of from dist_err — different shape, same components.
			case ROLL_PITCH_FBW: {
				// hold position if we let go of sticks
				if (this.cruise_speed !== 0) {
					// reset position
					this.target_x = sensors.x;
					this.target_z = sensors.z ?? 0;
					this.speed_I  = 0;
				}

				this.dist_err = this.get_dist_err(sensors);

				// defaulting to 500 / 12 = 41cm/s = 1.5r/s = 1200e/s
				let desired_speed;
				if (this.cruise_speed === 0) {
					desired_speed = this.dist_err;
				} else {
					const v_max = gains.v_max ?? 0.8;
					desired_speed = Math.max(-v_max, Math.min(v_max, this.cruise_speed));
				}
				this.desired_ticks = desired_speed;

				const speed_error = sensors.vel_cart - desired_speed;

				// 4 components of stability and navigation
				const bal_out = this.get_stabilize_pitch(0, sensors, gains, dt);    // hold as vertical as possible
				const vel_out = this.get_velocity_pitch(sensors, gains);            // magic
				const ff_out  = this.get_ff_out(desired_speed, gains);              // allows us to roll while vertical
				const nav_out = this.pid_nav(speed_error, sensors, gains, dt);      // allows us to accelerate

				// sum the output
				pitch_speed = bal_out + vel_out + nav_out - ff_out;
				break;
			}

			// ──────────────────────────────────────────────────────────
			// ROLL_PITCH_AUTO  — ArduBalance.pde:1357
			// ──────────────────────────────────────────────────────────
			// Three-line port:
			//
			//   pitch_speed  = get_stabilize_pitch(0);
			//   pitch_speed += get_velocity_pitch();
			//   pitch_speed += get_nav_pitch(min_speed, get_dist_err());
			//
			// min_speed = 0 in LOITER, 500 ticks/sec elsewhere — kept here
			// as the controller's `min_speed` field (m/s in our units).
			case ROLL_PITCH_AUTO:
				this.dist_err = this.get_dist_err(sensors);
				pitch_speed   = this.get_stabilize_pitch(0, sensors, gains, dt);
				pitch_speed  += this.get_velocity_pitch(sensors, gains);
				pitch_speed  += this.get_nav_pitch(this.min_speed, this.dist_err, sensors, gains, dt);
				break;
		}

		this.pitch_speed = pitch_speed;
	}

	// ──────────────────────────────────────────────────────────────────────
	// update_yaw_mode  — ArduBalance.pde:1209
	// ──────────────────────────────────────────────────────────────────────
	// Sets this.yaw_speed (in PWM units; same magnitude as pitch_speed so
	// they sum cleanly at update_servos).
	//
	//   YAW_HOLD          (FBW)  — nav_yaw = cruise_heading
	//   YAW_LOOK_AT_NEXT_WP (AUTO) — nav_yaw = wp_bearing
	//   YAW_ACRO          (raw)  — yaw_speed = pilot yaw_rate stick (scaled)
	update_yaw_mode(sensors, gains) {
		// Firmware ArduBalance.pde:1213 — if pitched/rolled past 40°, kill
		// yaw and snap nav_yaw to current heading. Stops the bot from
		// trying to rotate while it's tipping over.
		if (Math.abs(sensors.pitch) > Math.PI * 4 / 18) {   // 40° = 4000 cd
			this.yaw_speed = 0;
			this.nav_yaw   = sensors.heading;
			return;
		}

		switch (this.yaw_mode) {
			case YAW_HOLD:
				// FBW pair — hold cruise_heading. (Firmware also had a
				// "released-stick" snapshot that we skip here: pilot's
				// joystick handler integrates the heading itself.)
				this.nav_yaw   = this.cruise_heading;
				this.yaw_speed = this.get_stabilize_yaw(this.nav_yaw, sensors, gains);
				break;

			case YAW_LOOK_AT_NEXT_WP:
				// AUTO pair — nav_yaw = wp_bearing (firmware line 1239).
				this.nav_yaw   = this.get_wp_bearing(sensors);
				this.yaw_speed = this.get_stabilize_yaw(this.nav_yaw, sensors, gains);
				break;

			case YAW_ACRO:
			default: {
				// Firmware: yaw_speed = rc_1.control_in (pilot stick in
				// PWM units, no PID). Same here — pilot's rate target
				// scaled into PWM via Kheading.
				const { Kheading = 2 } = gains;
				this.yaw_speed = Kheading * this.yaw_rate_target * 100;
			}
		}
	}

	// ──────────────────────────────────────────────────────────────────────
	// update_servos  — motors.pde:60
	// ──────────────────────────────────────────────────────────────────────
	// Differential mix of pitch_speed and yaw_speed into per-wheel PWMs,
	// then deadband + saturation. Firmware also wrote the PWMs straight to
	// hardware; here we convert per-wheel PWM → per-wheel force via the
	// motor model so Rollerbot's downstream pipeline can integrate into the
	// plant.
	update_servos(sensors, gains, motor) {
		const { dead_zone = 0, PWM_max = 2000 } = gains;

		// Differential mix. Firmware multiplies by wheel_mixer.kP per side;
		// here we leave that gain at 1 (it just scales magnitude).
		let pwm_left  = this.pitch_speed + this.yaw_speed;
		let pwm_right = this.pitch_speed - this.yaw_speed;

		// Deadband-jump compensation, applied symmetrically to both wheels.
		if (pwm_left  > 0) pwm_left  += dead_zone;
		if (pwm_left  < 0) pwm_left  -= dead_zone;
		if (pwm_right > 0) pwm_right += dead_zone;
		if (pwm_right < 0) pwm_right -= dead_zone;

		pwm_left  = Math.max(-PWM_max, Math.min(PWM_max, pwm_left));
		pwm_right = Math.max(-PWM_max, Math.min(PWM_max, pwm_right));

		this.pwm_left  = pwm_left;
		this.pwm_right = pwm_right;
		this.lastPWM   = Math.abs(pwm_left) > Math.abs(pwm_right) ? pwm_left : pwm_right;

		// Per-wheel velocity for the back-EMF term in motor.forceFromPWM.
		const wb     = gains.wheelbase ?? motor.wheelbase;
		const half   = wb / 2;
		const v_left  = sensors.vel_cart - sensors.yaw_rate * half;
		const v_right = sensors.vel_cart + sensors.yaw_rate * half;

		const torque_left  = motor.forceFromPWM(pwm_left,  v_left);
		const torque_right = motor.forceFromPWM(pwm_right, v_right);

		// Realized chassis quantities (telemetry / plotter only).
		this.lastForceFwd  = torque_left + torque_right;
		this.lastTorqueYaw = (torque_right - torque_left) * half;

		// Returning pwm_left/right alongside the per-wheel forces tells
		// Rollerbot to bypass motor.applyTorque's inverse-model PI for us
		// — the PWMs we just computed already ARE the firmware's final
		// motor output, so re-deriving them via FF would be a round trip
		// through the motor model. Direct path keeps the firmware codepath
		// reaching the plant unchanged.
		return { torque_left, torque_right, pwm_left, pwm_right };
	}

	// ──────────────────────────────────────────────────────────────────────
	// get_stabilize_pitch  — Attitude.pde:14
	// ──────────────────────────────────────────────────────────────────────
	// Angle PD plus a slow auto-trim of balance_offset when the bot is
	// quiet near vertical. Returns PWM contribution for pitch_speed.
	get_stabilize_pitch(target_angle, sensors, gains, dt) {
		const { bal_P, bal_D, bal_I } = gains;
		const pitch     = sensors.pitch + this.balance_offset;
		const angle_err = pitch - target_angle;

		// Firmware gated auto-trim on (target_angle == 0); we add a quiet
		// check on angle_err so a real sustained lean doesn't get absorbed
		// as bias, and a 60 s leak so a stale offset can't survive.
		const quiet = Math.abs(target_angle) < 0.01
		           && Math.abs(angle_err)    < 0.04;
		if (quiet) this.balance_offset += bal_I * angle_err * dt;
		this.balance_offset *= (1 - dt / 60);

		// D on raw gyro, not d(err)/dt — avoids a derivative kick when the
		// pilot snaps target_angle. (Firmware did the same: its PID helper
		// ate gyro directly as the rate input.)
		return bal_P * angle_err + bal_D * sensors.pitch_rate;
	}

	// ──────────────────────────────────────────────────────────────────────
	// get_velocity_pitch  — Attitude.pde:40
	// ──────────────────────────────────────────────────────────────────────
	get_velocity_pitch(sensors, gains) {
		return gains.p_vel * sensors.vel_cart;
	}

	// ──────────────────────────────────────────────────────────────────────
	// get_nav_pitch  — navigation.pde:160
	// ──────────────────────────────────────────────────────────────────────
	// Firmware signature: get_nav_pitch(int16_t speed, int16_t dist_err).
	// Same here: takes a min-speed floor and the projected distance error,
	// computes desired_ticks via convert + accel limit + min floor, then
	// returns (nav_out − ff_out).
	//
	//   desired_ticks = convert_distance_to_encoder_speed(dist_err);
	//   desired_ticks = min(desired_ticks, desired_ticks_old + 10);
	//   desired_ticks = max(desired_ticks, speed);
	//   wheel_speed_error = wheel.speed - desired_ticks;
	//   ff_out  = desired_ticks * throttle;
	//   nav_out = pid_nav.get_pid(wheel_speed_error, G_Dt);
	//   return constrain(nav_out − ff_out, ±2000);
	get_nav_pitch(min_speed, dist_err, sensors, gains, dt) {
		const { PWM_max = 2000 } = gains;

		this.desired_ticks = this.convert_distance_to_speed(dist_err, min_speed, gains, dt);

		const wheel_speed_error = sensors.vel_cart - this.desired_ticks;
		const ff_out            = this.get_ff_out(this.desired_ticks, gains);
		const nav_out           = this.pid_nav(wheel_speed_error, sensors, gains, dt);

		const result = nav_out - ff_out;
		return Math.max(-PWM_max, Math.min(PWM_max, result));
	}


static int16_t
get_nav_pitch(int16_t speed, int16_t dist_err)
{
	static int16_t desired_ticks_old = 0;
	int16_t nav_out, ff_out, wheel_speed_error;

    // this is the speed of the wheels: 1000 = 1 rotation of the wheels.
    // We convert cm to rpm based on wheel diamter and encoder ticks per revolution
    desired_ticks 	= convert_distance_to_encoder_speed(dist_err);

	// accleration is limited to prevent wobbly starts towards waypoints
	desired_ticks 	= min(desired_ticks, desired_ticks_old + 10);// limit going faster
	desired_ticks 	= max(desired_ticks, speed);
	desired_ticks_old = desired_ticks;

	// grab the wheel speed error
	wheel_speed_error 	= wheel.speed - desired_ticks;

    ff_out          = (float)desired_ticks * g.throttle; // allows us to roll while vertical
	nav_out      	= g.pid_nav.get_pid(wheel_speed_error, G_Dt);

    return constrain((nav_out - ff_out), -2000, 2000);
}




	// ──────────────────────────────────────────────────────────────────────
	// pid_nav.get_pid  — Parameters / firmware PID helper
	// ──────────────────────────────────────────────────────────────────────
	// The speed PID's P/I/D piece. Pulled out so FBW (which sums components
	// inline) and AUTO (which delegates to get_nav_pitch) share the same
	// integrator + derivative state. Anti-windup gates I on PWM saturation.
	pid_nav(speed_error, sensors, gains, dt) {
		const { wheel_P, wheel_I, wheel_D, PWM_max = 2000 } = gains;

		const saturated = Math.abs(this.lastPWM) >= (PWM_max - 1);
		if (!saturated) this.speed_I += speed_error * dt;

		// D on measurement (no kick when desired jumps), 20 ms LPF.
		const raw_d = (sensors.vel_cart - this.last_vel_cart_meas) / dt;
		this.last_vel_cart_meas = sensors.vel_cart;
		const alpha = dt / (0.02 + dt);
		this.speed_d_lpf = (1 - alpha) * this.speed_d_lpf + alpha * raw_d;

		return wheel_P * speed_error
		     + wheel_I * this.speed_I
		     + wheel_D * this.speed_d_lpf;
	}

	// ──────────────────────────────────────────────────────────────────────
	// ff_out  — navigation.pde:177  (the "throttle while vertical" term)
	// ──────────────────────────────────────────────────────────────────────
	//
	//   ff_out = (float)desired_ticks * g.throttle;
	//
	// Firmware used a single throttle scalar; we reuse the calibrated
	// PWM-from-speed table (with ff_per_mps as the linear-fallback slope).
	get_ff_out(desired_speed, gains) {
		this.pwmTable.linearSlope = gains.ff_per_mps;
		return this.pwmTable.pwmFromSpeed(desired_speed);
	}

	// ──────────────────────────────────────────────────────────────────────
	// get_stabilize_yaw  — Attitude.pde:48
	// ──────────────────────────────────────────────────────────────────────
	// Heading P → yaw_speed in PWM units. Firmware divided by wheel_ratio
	// to bring the output into the same units as pitch_speed; here we use
	// a magnitude-matching constant so update_servos can sum them
	// directly.
	get_stabilize_yaw(target_yaw, sensors, gains) {
		const { Kheading = 2, MaxYawRate = 1.5 } = gains;
		let heading_err = target_yaw - sensors.heading;
		while (heading_err >  Math.PI) heading_err -= 2 * Math.PI;
		while (heading_err < -Math.PI) heading_err += 2 * Math.PI;

		// First a clamped rate target (rad/s), then scale to PWM units.
		// Splitting it this way keeps MaxYawRate's meaning in physical
		// units while letting yaw_speed live in PWM space.
		let rate = Kheading * heading_err;
		if (rate >  MaxYawRate) rate =  MaxYawRate;
		if (rate < -MaxYawRate) rate = -MaxYawRate;

		// Convert rate (rad/s) → PWM units. 100 PWM per rad/s lands
		// MaxYawRate ≈ 1.5 at 150 PWM, well below pitch_speed magnitudes
		// so yaw stays a small correction at top-speed cruise. Tunable.
		return rate * 100;
	}

	// ──────────────────────────────────────────────────────────────────────
	// get_dist_err  — navigation.pde:183
	// ──────────────────────────────────────────────────────────────────────
	// Project (target − current_pos) onto the bot's forward axis.
	// Firmware constrained the result to [0, 45] cm — never reverses for
	// nav, never overshoots more than 45 cm of "forward speed worth" of
	// error. We keep the clamp-≥-0 (no reverse) but skip the upper bound
	// (Pilot's own waypoint cycling caps it).
	get_dist_err(sensors) {
		const dx = this.target_x - sensors.x;
		const dz = this.target_z - (sensors.z ?? 0);
		// Our heading convention: forward = (cos h, −sin h).
		const proj = dx * Math.cos(sensors.heading) - dz * Math.sin(sensors.heading);
		return Math.max(0, proj);
	}

	// ──────────────────────────────────────────────────────────────────────
	// get_wp_bearing  — navigation.pde:93
	// ──────────────────────────────────────────────────────────────────────
	get_wp_bearing(sensors) {
		const dx = this.target_x - sensors.x;
		const dz = this.target_z - (sensors.z ?? 0);
		return Math.atan2(-dz, dx);
	}

	// ──────────────────────────────────────────────────────────────────────
	// convert_distance_to_encoder_speed  — first three lines of
	// get_nav_pitch (navigation.pde:167-172)
	// ──────────────────────────────────────────────────────────────────────
	//
	//   desired_ticks = convert_distance_to_encoder_speed(dist_err);
	//   desired_ticks = min(desired_ticks, desired_ticks_old + 10);
	//   desired_ticks = max(desired_ticks, speed);
	//
	// Firmware mapped cm-of-distance → ticks/sec via wheel radius and
	// encoder constants. In our SI units this collapses to a gain
	// (Kp_nav, m/s per m of distance) plus a per-tick accel limit
	// plus a min-speed floor.
	//
	// AVC field-tape note: that "+10" in the firmware was duct tape.
	// During practice runs at the venue, every waypoint hop kicked
	// desired_ticks hard enough to knock the bot over before the
	// balance loop could catch it. Capping the per-tick growth at +10
	// (firmware) softened the impulse so the bot could absorb the
	// step. Won the race. Here it's a_max · dt — same idea, scaled to
	// our timestep instead of the firmware's main-loop rate.
	convert_distance_to_speed(dist_err, min_speed, gains, dt) {
		const { Kp_nav = 1.0, a_max = 1.5 } = gains;
		let desired = Kp_nav * dist_err;
		desired = Math.min(desired, this.desired_ticks_old + a_max * dt);
		desired = Math.max(desired, min_speed);
		this.desired_ticks_old = desired;
		return desired;
	}

	// vel_command stayed in our telemetry schema across the refactor;
	// expose it as the active speed setpoint regardless of mode so the
	// plotter has something meaningful to draw.
	get vel_command() {
		if (this.roll_pitch_mode === ROLL_PITCH_AUTO) return this.desired_ticks;
		if (this.roll_pitch_mode === ROLL_PITCH_FBW)  return this.cruise_speed;
		return 0;
	}
}
