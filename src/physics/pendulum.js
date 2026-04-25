// Physics: inverted pendulum on wheels (two-wheel balancer, 2D side view).
// Body = point mass m at distance L above axle, with +pitch = bob tilted in +x.
//
//   (M_eff + m)·ẍ + m·L·cos(pitch)·pitcḧ = F + m·L·pitch_rate²·sin(pitch) - cx·ẋ
//          cos(pitch)·ẍ  + L·pitcḧ       = g·sin(pitch) - F·R/(m·L) - cp·pitch_rate/(m·L²)
//
// where M_eff = M + Iw/R² (wheel rotational inertia adds effective cart mass
// since accelerating a rolling wheel takes extra force).
//
// The -F·R/(m·L) term is motor reaction torque: when the motors torque the
// wheels in the direction that drives the cart in +x, Newton's 3rd law
// torques the body in the direction that tips it in -pitch (backward
// relative to the cart's acceleration — exactly the Segway "lean back when
// you accelerate forward" effect). It opposes the cart-coupling catch
// response, so overall effect is to make balancing harder as R grows.
//
// Solve 2x2, integrate with RK4.

export const G = 9.81;

// State includes 2D world position (x, z), heading, and yaw rate alongside
// the pitch dynamics. Yaw is decoupled from pitch — they don't share dynamics
// in this model — so a single RK4 over the unified 7-state vector is enough.

export class Pendulum {
	constructor(params) {
		// params: {M, m, L, R, Iw, cx, cp, I_yaw, c_yaw}
		this.params = params;
		this.state  = {
			x: 0, z: 0, vel_cart: 0,
			pitch: 0, pitch_rate: 0,
			heading: 0, yaw_rate: 0,
			// Per-wheel rotation angles (radians), integrated from the
			// differential-drive kinematics so the renderer can show real
			// wheel rotation regardless of yaw or world-x.
			wheel_left_angle: 0, wheel_right_angle: 0,
		};
	}

	// Distance between left/right wheel contact patches (m). Used by the
	// differential-drive kinematics to back out per-wheel angular velocity
	// from the body's forward speed and yaw rate.
	wheelbase() { return this.params.wheelbase ?? 0.48; }

	setState(s) { this.state = { ...this.state, ...s }; }

	derivs(s, F, yaw_torque) {
		const { M, m, L, R, Iw, cx, cp, I_yaw, c_yaw } = this.params;

		// Pitch dynamics — vel_cart is body-frame forward velocity, F is the
		// body-frame forward force on the chassis from the wheel motors.
		const sn = Math.sin(s.pitch), cs = Math.cos(s.pitch);
		const Meff = M + (Iw / (R * R));
		const D = Meff + m * sn * sn;
		const b1 = F + m * L * s.pitch_rate * s.pitch_rate * sn - cx * s.vel_cart;
		const b2 = G * sn - (F * R) / (m * L) - (cp * s.pitch_rate) / (m * L * L);
		const vel_cart_dot   = (b1 - m * cs * b2) / D;
		const pitch_rate_dot = (-cs * b1 + (Meff + m) * b2) / (L * D);

		// Yaw dynamics — simple rigid-body rotation, decoupled from pitch.
		const yaw_rate_dot = (yaw_torque - c_yaw * s.yaw_rate) / I_yaw;

		// Differential-drive wheel kinematics. Forward speed vel_cart plus a
		// yaw component on each side: outer wheel travels (vel_cart + yaw_rate·d/2),
		// inner wheel (vel_cart − yaw_rate·d/2). Right wheel is the outer one
		// when yaw_rate is positive (CCW from above = left turn).
		const half_wheelbase     = this.wheelbase() / 2;
		const wheel_left_dot     = (s.vel_cart - s.yaw_rate * half_wheelbase) / R;
		const wheel_right_dot    = (s.vel_cart + s.yaw_rate * half_wheelbase) / R;

		// World-frame motion derived from body-frame vel_cart and heading.
		// three.js's mesh.rotation.y = heading rotates the local +X axis to
		// (cos heading, 0, -sin heading) in world space, so we use the
		// matching sign convention for position derivatives. With this,
		// positive yaw rate turns the bot CCW as viewed from +Y (looking down).
		return {
			x:                 s.vel_cart * Math.cos(s.heading),
			z:                -s.vel_cart * Math.sin(s.heading),
			vel_cart:          vel_cart_dot,
			pitch:             s.pitch_rate,
			pitch_rate:        pitch_rate_dot,
			heading:           s.yaw_rate,
			yaw_rate:          yaw_rate_dot,
			wheel_left_angle:  wheel_left_dot,
			wheel_right_angle: wheel_right_dot,
		};
	}

	step(F, yaw_torque, dt) {
		const s = this.state;
		const add = (a, b, k) => {
			const out = {};
			for (const key of Object.keys(a)) out[key] = a[key] + (b[key] || 0) * k;
			return out;
		};
		const k1 = this.derivs(s, F, yaw_torque);
		const k2 = this.derivs(add(s, k1, dt / 2), F, yaw_torque);
		const k3 = this.derivs(add(s, k2, dt / 2), F, yaw_torque);
		const k4 = this.derivs(add(s, k3, dt),     F, yaw_torque);
		const out = {};
		for (const key of Object.keys(s)) {
			out[key] = s[key] + dt / 6 * (k1[key] + 2 * k2[key] + 2 * k3[key] + k4[key]);
		}
		this.state = out;
		return this.state;
	}
}
