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

// Firmware ran angles in centidegrees (hundredths of a degree, int32_t).
// 1° = 100 cd. 90° = 9000 cd. ±180° = ±18000 cd.
// We sample angles in radians (sim convention), then convert at the
// controller boundary so bal_P / bal_D / Kheading take their old
// firmware-tuned values straight from Parameters.pde.
const RAD_TO_CD = 18000 / Math.PI;   // ≈ 5729.578

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
		this.min_speed       = .17;        // floor for desired_ticks (m/s)

		this.pwmTable        = new PWMTable();   // FF curve (linear until calibrated)

		this.reset();
	}

	reset() {
		// Per-tick scratch ----------------------------------------------------
		this.pitch_speed        = 0;     // output of update_roll_pitch_mode
		this.yaw_speed          = 0;     // output of update_yaw_mode
		this.pwm_left           = 0;     // outputs of update_servos (PWM only;
		this.pwm_right          = 0;     // motor sim happens in Rollerbot)
		this.nav_yaw            = 0;     // current yaw target (rad)

		// Persistent state ----------------------------------------------------
		this.balance_offset     = 0;     // learned IMU zero-offset (cd, firmware-native)
		this.speed_I            = 0;     // get_nav_pitch integrator
		this.desired_ticks_old  = 0;     // get_nav_pitch accel limit memory

		// Persistent state for pid_nav (firmware: APM_PID instance fields)
		this._last_error        = 0;
		this._last_derivative   = 0;

		// Diagnostics (drained by telemetry / plotter) -----------------------
		this.lastPWM            = 0;     // larger-magnitude wheel (firmware-style scalar)

		// First-tick re-anchor of next_WP. Firmware called init_home() at
		// boot which zeroed current_loc/next_WP to "wherever the bot is now,"
		// so the FBW header's `if(stick) next_WP = current_loc` was correct
		// even on the first frame. Our sim uses absolute world coords —
		// next_WP=(0,0) doesn't equal the bot's spawn pose, so without this
		// sentinel the first FBW/STABLE tick reads a stale target tens of
		// meters away. Snapshot once on mode entry; the firmware-style
		// stick-deflection check takes over from there.
		this._needsHoldSnapshot = true;
	}

	// Pilot interface ---------------------------------------------------------
	// Each setter selects the firmware mode pair (roll_pitch_mode + yaw_mode)
	// matching the original code:
	//
	//   raw keyboard stick  → STABLE + ACRO              (pilot owns angle + rate)
	//   FBW joystick        → FBW    + HOLD              (pilot owns speed + heading)
	//   auto waypoint       → AUTO   + LOOK_AT_NEXT_WP   (controller owns everything)
	setAttitude(tilt, yawRate) {
		if (this.roll_pitch_mode !== ROLL_PITCH_STABLE) this._needsHoldSnapshot = true;
		this.roll_pitch_mode = ROLL_PITCH_STABLE;
		this.yaw_mode        = YAW_ACRO;
		this.target_angle    = tilt    ?? 0;
		this.yaw_rate_target = yawRate ?? 0;
	}

	setCruise(speed, heading) {
		if (this.roll_pitch_mode !== ROLL_PITCH_FBW) this._needsHoldSnapshot = true;
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
		this.min_speed       = min_speed ?? 0.17;
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
	// All three are void — they read/write member state, just like the
	// firmware. fastLoop hands Rollerbot the per-wheel PWMs that
	// update_servos wrote; Rollerbot simulates the motor (PWM → force)
	// in its own _fastLoop since that's the physical world's job, not
	// the controller's.
	fastLoop(sensors, gains, dt /*, motor */) {
		this.update_roll_pitch_mode(sensors, gains, dt);
		this.update_yaw_mode(sensors, gains);
		this.update_servos(sensors, gains);

		return {
			pwm_left:  this.pwm_left,
			pwm_right: this.pwm_right,
		};
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
			case ROLL_PITCH_STABLE: {
				// we always hold position
				if (this.target_angle !== 0 || this._needsHoldSnapshot) {
					// reset position
					this.target_x = sensors.x;
					this.target_z = sensors.z ?? 0;
					this.speed_I  = 0;
					this._needsHoldSnapshot = false;
				}

				// in this mode we command the target angle
				let bal_out = this.get_stabilize_pitch(this.target_angle, sensors, gains, dt);

				// speed control:
				let vel_out = this.get_velocity_pitch(sensors, gains);

				// maintain location:
				let nav_out = this.get_nav_pitch(0, this.get_dist_err(sensors), sensors, gains, dt);

				pitch_speed = (bal_out + vel_out + nav_out);
				break;
			}

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
				if (this.cruise_speed !== 0 || this._needsHoldSnapshot) {
					// reset position
					this.target_x = sensors.x;
					this.target_z = sensors.z ?? 0;
					this.speed_I  = 0;
					this._needsHoldSnapshot = false;
				}

				// distance_error = (float)long_error * cos_yaw_x + (float)lat_error * sin_yaw_y;
				const dx = this.target_x - sensors.x;
				const dz = this.target_z - (sensors.z ?? 0);
				
				const distance_error = dx * Math.cos(sensors.heading) - dz * Math.sin(sensors.heading);

				// defaulting to 500 / 12 = 41cm/s = 1.5r/s = 1200e/s
				let desired_speed;
				if (this.cruise_speed === 0) {
					desired_speed = distance_error;
				} else {
					desired_speed = this.cruise_speed;                                 // units = m/s (firmware: cm/s)
					desired_speed = Math.max(-0.8, Math.min(0.8, desired_speed));      // ±80 cm/s = ±0.8 m/s
				}

				// switching units to ticks  (firmware: convert_distance_to_encoder_speed —
				// no-op here, we're already in m/s; line kept for shape parity)
				let speed_error = sensors.vel_bot - desired_speed;

				// 4 components of stability and navigation
				let bal_out = this.get_stabilize_pitch(0, sensors, gains, dt);    // hold as vertical as possible
				let vel_out = this.get_velocity_pitch(sensors, gains);            // magic
				let ff_out  = desired_speed * gains.ff_per_mps;                   // allows us to roll while vertical
				let nav_out = this.pid_nav(speed_error, gains, dt);               // allows us to accelerate

				// sum the output
				pitch_speed = (bal_out + vel_out + nav_out - ff_out);
				//pitch_speed = (bal_out + vel_out);
				console.log("bal_out",bal_out ,"vel_out", vel_out ,"nav_out", nav_out ,"ff_out", ff_out);
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
				// in this mode we command the target angle
				pitch_speed  = this.get_stabilize_pitch(0, sensors, gains, dt);

				// speed control:
				pitch_speed += this.get_velocity_pitch(sensors, gains);

				// maintain location:
				pitch_speed += this.get_nav_pitch(this.min_speed, this.get_dist_err(sensors), sensors, gains, dt);
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
		//this.yaw_speed = 0;
	}

	// ──────────────────────────────────────────────────────────────────────
	// update_servos  — motors.pde:60
	// ──────────────────────────────────────────────────────────────────────
	// Void in the firmware (writes to motor_out[] and to hardware). Void
	// here too — writes only PWM. The PWM → force response is the
	// physical world's job in the firmware; in our sim it lives in
	// Rollerbot, which reads pwm_left/pwm_right and asks the motor what
	// force they produce. Keeps update_servos a pure firmware port.
	update_servos(_sensors, gains, _motor) {
		const { dead_zone = 0, PWM_max = 2000 } = gains;



		// Differential mix. Firmware lines:
		//
		//   motor_out[LEFT]  = (pitch_speed + yaw_speed) * pid_wheel_left_mixer.kP();
		//   motor_out[RIGHT] = (pitch_speed - yaw_speed) * pid_wheel_right_mixer.kP();
		//
		// The two wheel-mixer kPs typically had OPPOSITE SIGNS in firmware
		// tuning — the right motor on the real bot was mounted facing the
		// opposite way from the left, so equal PWMs would have spun the
		// chassis. Flipping the right-side kP made +pitch_speed produce
		// equal-magnitude opposite-sign PWMs that BOTH push the chassis
		// forward. Same convention here: identical pwm_left/pwm_right
		// will spin the bot, just like setting both motor channels to
		// 2000 on the real hardware.
		// Firmware: pid_wheel_left_mixer.kP() was NEGATIVE, pid_wheel_right_mixer.kP()
		// was POSITIVE. Two sign tricks combined: (a) firmware's pitch_speed is
		// "neg = forward lean" so kP_L's negative sign flips that into a chassis-
		// forward push for the left wheel; (b) the right motor is physically
		// mirrored, so its kP is the opposite sign of kP_L. Same convention here.
		const kP_L = -0.5;
		const kP_R = -kP_L;   // = +0.5; firmware mirror
		let pwm_left  = (this.pitch_speed + this.yaw_speed) * kP_L;
		let pwm_right = (this.pitch_speed - this.yaw_speed) * kP_R;

		// Deadband-jump compensation, applied symmetrically to both wheels.
		if (pwm_left  > 0) pwm_left  += dead_zone;
		if (pwm_left  < 0) pwm_left  -= dead_zone;
		if (pwm_right > 0) pwm_right += dead_zone;
		if (pwm_right < 0) pwm_right -= dead_zone;

		pwm_left  = Math.max(-PWM_max, Math.min(PWM_max, pwm_left));
		pwm_right = Math.max(-PWM_max, Math.min(PWM_max, pwm_right));

		// Firmware here calls hal.rcout->write(CH_1, ...). We write to
		// instance fields and let Rollerbot (the "hardware") read them.
		this.pwm_left  = pwm_left;
		this.pwm_right = pwm_right;
//		console.log(this.pwm_left);
		this.lastPWM   = Math.abs(pwm_left) > Math.abs(pwm_right) ? pwm_left : pwm_right;
	}

	// ──────────────────────────────────────────────────────────────────────
	// get_stabilize_pitch  — Attitude.pde:15
	// ──────────────────────────────────────────────────────────────────────
	// Direct port. The only forced changes are:
	//   - convert sensors from rad to cd at the top (sim samples in rad)
	//   - `this.` instead of firmware globals (balance_offset, gains)
	//   - `dt` parameter instead of `G_Dt` global
	// Everything below the conversion block reads against the .pde file.
	get_stabilize_pitch(target_angle, sensors, gains, dt) {
		const pitch_sensor = sensors.pitch * RAD_TO_CD;
		target_angle       = target_angle * RAD_TO_CD;
		const omega_y      = sensors.pitch_rate * RAD_TO_CD;

		let angle_error = this.wrap_180(target_angle - (pitch_sensor + this.balance_offset));

		// dynamically adjust the CG when we are supposed to be at vertical
		if (target_angle === 0) {
			this.balance_offset += gains.bal_I * angle_error * dt;
		}
		let rate_error = 0 - omega_y;

		let bal_P = gains.bal_P * angle_error;
		let bal_D = gains.bal_D * rate_error;

		let torque = bal_P + bal_D;
		return torque;
	}

	// ──────────────────────────────────────────────────────────────────────
	// get_velocity_pitch  — Attitude.pde:40
	// ──────────────────────────────────────────────────────────────────────
	get_velocity_pitch(sensors, gains) {
		// 1.2
		return gains.p_vel * sensors.vel_bot;
	}

	// ──────────────────────────────────────────────────────────────────────
	// get_nav_pitch  — navigation.pde:160
	// ──────────────────────────────────────────────────────────────────────
	// Direct port. Forced changes only:
	//   - sensors / gains / dt parameters instead of firmware globals
	//   - `pid_nav.get_pid` is our `pid_nav` method (no PID class)
	//   - desired_ticks_old is `this._desired_ticks_old` (no `static` locals in JS)
	get_nav_pitch(speed, dist_err, sensors, gains, dt) {
		let nav_out, ff_out, wheel_speed_error;

		// this is the speed of the wheels: 1000 = 1 rotation of the wheels.
		// We convert cm to rpm based on wheel diamter and encoder ticks per revolution
		let desired_ticks = this.convert_distance_to_encoder_speed(dist_err, gains);

		// accleration is limited to prevent wobbly starts towards waypoints
		desired_ticks = Math.min(desired_ticks, this.desired_ticks_old + (gains.a_max ?? 1.5) * dt);
		desired_ticks = Math.max(desired_ticks, speed);
		this.desired_ticks_old = desired_ticks;

		// grab the wheel speed error
		wheel_speed_error = sensors.vel_bot - desired_ticks;

		ff_out  = desired_ticks * gains.ff_per_mps;            // allows us to roll while vertical
		nav_out = this.pid_nav(wheel_speed_error, gains, dt);

		return Math.max(-2000, Math.min(2000, nav_out - ff_out));
	}

	// ──────────────────────────────────────────────────────────────────────
	// pid_nav.get_pid  — APM_PID::get_pid (firmware PID helper class)
	// ──────────────────────────────────────────────────────────────────────
	// Direct port of the firmware PID class's get_pid method. Forced
	// changes: gains read via the gains arg (no PID class with kP/kI/kD
	// member fields); state stored on `this` (no PID instance).
	pid_nav(error, gains, dt) {
		const _kp     = gains.wheel_P;
		const _ki     = gains.wheel_I;
		const _kd     = gains.wheel_D;
		const _filter = 0.02;            // discrete LPF time constant
		const _imax   = gains.wheel_I_max ?? 500;

		let output = error * _kp;

		if (Math.abs(_kd) > 0 && dt > 0) {
			let derivative = (error - this._last_error) / dt;
			// discrete low pass filter, cuts out the high frequency noise
			// that can drive the controller crazy
			derivative = this._last_derivative + (dt / (_filter + dt)) * (derivative - this._last_derivative);
			this._last_error      = error;
			this._last_derivative = derivative;
			output += _kd * derivative;
		}

		if (Math.abs(_ki) > 0 && dt > 0) {
			this.speed_I += (error * _ki) * dt;
			if (this.speed_I < -_imax) this.speed_I = -_imax;
			else if (this.speed_I >  _imax) this.speed_I =  _imax;
			output += this.speed_I;
		}

		return output;
	}

	// ──────────────────────────────────────────────────────────────────────
	// get_stabilize_yaw  — Attitude.pde:48
	// ──────────────────────────────────────────────────────────────────────
	// Direct port. Forced changes only:
	//   - convert sensors from rad to cd at the top
	//   - `gains.Kheading` instead of `g.pid_yaw.get_p` PID helper
	//   - `wheel_ratio` divisor folded into the per-side kP at update_servos
	//     (kP_L = +1, kP_R = −1), so it's not present here
	get_stabilize_yaw(target_angle, sensors, gains) {
		const yaw_sensor = sensors.heading * RAD_TO_CD;
		target_angle     = target_angle * RAD_TO_CD;

		let angle_error;

		// angle error
		angle_error = this.wrap_180(target_angle - yaw_sensor);

		// limit the error we're feeding to the PID
		angle_error = Math.max(-1000, Math.min(1000, angle_error));
		let output  = gains.Kheading * angle_error;
		return output;
	}

	// ──────────────────────────────────────────────────────────────────────
	// get_dist_err  — navigation.pde:183
	// ──────────────────────────────────────────────────────────────────────
	// Project (target − current_pos) onto the bot's forward axis, then
	// clamp to [0, 0.45 m]. Firmware was [0, 45] cm — same cap, just SI
	// units. This is the safety the firmware relied on so a far waypoint
	// (or stale next_WP) couldn't make desired_ticks blow up via
	// wheel_P · speed_error. Without it the inner PID happily commands
	// 45 m/s when the snapshot is stale and the bot launches into orbit.
	get_dist_err(sensors) {
		const dx = this.target_x - sensors.x;
		const dz = this.target_z - (sensors.z ?? 0);
		// Our heading convention: forward = (cos h, −sin h).
		const proj = dx * Math.cos(sensors.heading) - dz * Math.sin(sensors.heading);
		return Math.max(0, Math.min(0.45, proj));
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
	// wrap_180  — firmware utility (used everywhere a heading error is
	//             fed to a P loop)
	// ──────────────────────────────────────────────────────────────────────
	// Folds an angle into [-18000, +18000] cd — i.e., the shortest signed
	// rotation between two headings. Without this, a target at 10° and a
	// measured heading at 350° would produce a -340° error and the bot
	// would pirouette instead of just turning 20° right.
	//
	// While-loops are cheap when the input is usually within ±36000;
	// modulo'd version ((angle + 18000) % 36000 - 18000) is faster for
	// arbitrary big inputs but uglier.
	wrap_180(angle_cd) {
		while (angle_cd >  18000) angle_cd -= 36000;
		while (angle_cd < -18000) angle_cd += 36000;
		return angle_cd;
	}

	// ──────────────────────────────────────────────────────────────────────
	// convert_distance_to_encoder_speed  — firmware helper called from
	// get_nav_pitch (navigation.pde:167)
	// ──────────────────────────────────────────────────────────────────────
	// Firmware mapped cm-of-distance → ticks/sec via wheel diameter and
	// encoder ticks/rev. Here we work in SI units so it collapses to a
	// single gain (Kp_nav, m/s per m of distance).
	convert_distance_to_encoder_speed(dist_err, gains) {
		return (gains.Kp_nav ?? 1.0) * dist_err;
	}
	
	convert_distance_to_encoder_speed(_distance)
	{
		const WHEEL_DIAMETER_CM = 28.27;
		const wheel_encoder_speed = 815
		return (_distance * wheel_encoder_speed ) / WHEEL_DIAMETER_CM;
	}
	convert_groundspeed_to_encoder_speed(_ground_speed)
	{
		const WHEEL_DIAMETER_CM = 28.27;
		const wheel_encoder_speed = 815
		return (_ground_speed * wheel_encoder_speed ) / WHEEL_DIAMETER_CM;
	}

	convert_encoder_speed_to_ground_speed(encoder_speed)
	{
		const WHEEL_DIAMETER_CM = 28.27;
		const wheel_encoder_speed = 815
		return (encoder_speed * WHEEL_DIAMETER_CM) / wheel_encoder_speed;
	}

	// vel_command stayed in our telemetry schema across the refactor.
	// Returns the active speed setpoint at controller scope (cruise_speed
	// in FBW; 0 elsewhere — AUTO's desired_ticks is now a function-local).
	get vel_command() {
		return this.roll_pitch_mode === ROLL_PITCH_FBW ? this.cruise_speed : 0;
	}
}
