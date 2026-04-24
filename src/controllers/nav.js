// Navigation outer-outer loop.
//
// Given a target x position, produces a tilt setpoint (target_angle) that
// the balance controller then tracks. Two modes:
//
//   pd       — classic PD on position error (fast to tune, asymptotic,
//              overshoot/settling tradeoff).
//
//   profile  — square-root braking velocity profile. At every instant,
//              compute the fastest velocity that can still brake to zero
//              at the target given a bounded deceleration, then lean to
//              match that velocity. Behaves like a human driver: starts
//              braking well before the stop sign, arrives at zero velocity
//              exactly at the target. No overshoot.

export class NavController {
	constructor() {
		this.target_x = 0;
		this.v_lpf    = 0;        // low-passed velocity (encoder is quantized)
		this.mode     = 'profile';   // 'pd' | 'profile'
	}

	reset() { this.v_lpf = 0; }

	update(sensors, gains, dt = 1 / 60) {
		// LPF the CoM velocity, not the cart/wheel velocity. The cart does a
		// non-minimum-phase shimmy at startup (goes backward briefly so the
		// body can tip forward); the CoM doesn't. Closing the loop on CoM
		// prevents nav from over-reacting to wheel-shuffle transients.
		const v_meas = sensors.v_CoM ?? sensors.v;
		const x_meas = sensors.x_CoM ?? sensors.x;

		const tau   = 0.1;
		const alpha = dt / (tau + dt);
		this.v_lpf  = (1 - alpha) * this.v_lpf + alpha * v_meas;

		const err = this.target_x - x_meas;
		this.err_last = err;

		let tilt = this.mode === 'profile'
			? this._profileTilt(err, gains)
			: this._pdTilt(err, gains);

		const lim = gains.tiltLimit;
		if (tilt >  lim) tilt =  lim;
		if (tilt < -lim) tilt = -lim;
		this.tilt_last = tilt;
		return tilt;
	}

	// --- Simple PD on position error ---------------------------------------
	_pdTilt(err, gains) {
		const { Kp_nav, Kd_nav } = gains;
		return Kp_nav * err - Kd_nav * this.v_lpf;
	}

	// --- Square-root braking velocity profile ------------------------------
	// Far from target: "what's the fastest I can be going right now and still
	// brake to zero by arrival?"   v_brake = √(2·a_max·|err|)
	//
	// Near the target (|err| < linear_zone): the sqrt curve becomes a problem
	// because its slope goes to infinity at zero — a 1 mm error commands a
	// significant velocity, faster than the bot can realistically shed, and
	// the controller drives itself into a limit-cycle shuffle.
	//
	// Inside the linear zone we use a straight line with the same value at
	// the boundary, so v_desired smoothly → 0 as err → 0. Industrial servos
	// call this "linear in the closet, square-root in the hall."
	_profileTilt(err, gains) {
		const { v_max, a_max, Kvel, linear_zone, lookahead } = gains;

		// Look-ahead: the bot can't stop on a dime — it takes `lookahead`
		// seconds for the balance loop to reverse tilt and start braking.
		// During that time it travels v·lookahead further. So we plan as if
		// we're already that far along, which moves the start-of-braking
		// earlier in real space while leaving acceleration untouched.
		const err_eff = err - this.v_lpf * lookahead;
		const absErr  = Math.abs(err_eff);

		let v_brake;
		if (absErr > linear_zone) {
			v_brake = Math.sqrt(2 * a_max * absErr);
		} else {
			const slope = Math.sqrt(2 * a_max / linear_zone);
			v_brake = slope * absErr;
		}
		const v_desired = Math.sign(err_eff) * Math.min(v_brake, v_max);
		this.v_desired_last = v_desired;
		return Kvel * (v_desired - this.v_lpf);
	}
}
