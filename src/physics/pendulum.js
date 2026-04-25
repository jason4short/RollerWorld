// Physics: inverted pendulum on wheels (two-wheel balancer, 2D side view).
// Body = point mass m at distance L above axle, with +θ = bob tilted in +x.
//
//   (M_eff + m)·ẍ + m·L·cosθ·θ̈ = F + m·L·θ̇²·sinθ - cx·ẋ
//          cosθ·ẍ  + L·θ̈       = g·sinθ - F·R/(m·L) - cp·θ̇/(m·L²)
//
// where M_eff = M + Iw/R² (wheel rotational inertia adds effective cart mass
// since accelerating a rolling wheel takes extra force).
//
// The -F·R/(m·L) term is motor reaction torque: when the motors torque the
// wheels in the direction that drives the cart in +x, Newton's 3rd law
// torques the body in the direction that tips it in -θ (backward relative
// to the cart's acceleration — exactly the Segway "lean back when you
// accelerate forward" effect). It opposes the cart-coupling catch response,
// so overall effect is to make balancing harder as R grows.
//
// Solve 2x2, integrate with RK4.

export const G = 9.81;

// State now includes 2D world position (x, z), heading psi, and yaw rate
// alongside the existing pitch dynamics. Yaw is decoupled from pitch — they
// don't share dynamics in this model — so a single RK4 over the unified
// 7-state vector is sufficient.

export class Pendulum {
	constructor(params) {
		// params: {M, m, L, R, Iw, cx, cp, I_yaw, c_yaw}
		this.params = params;
		this.state  = { x: 0, z: 0, v: 0, th: 0, w: 0, psi: 0, yawRate: 0 };
	}

	setState(s) { this.state = { ...this.state, ...s }; }

	derivs(s, F, tau_yaw) {
		const { M, m, L, R, Iw, cx, cp, I_yaw, c_yaw } = this.params;

		// Pitch dynamics — unchanged. v is body-frame forward velocity, F is
		// body-frame forward force from the pitch controller.
		const sn = Math.sin(s.th), cs = Math.cos(s.th);
		const Meff = M + (Iw / (R * R));
		const D = Meff + m * sn * sn;
		const b1 = F + m * L * s.w * s.w * sn - cx * s.v;
		const b2 = G * sn - (F * R) / (m * L) - (cp * s.w) / (m * L * L);
		const v_dot     = (b1 - m * cs * b2) / D;
		const w_dot     = (-cs * b1 + (Meff + m) * b2) / (L * D);

		// Yaw dynamics — simple rigid-body rotation, decoupled from pitch.
		const yawRate_dot = (tau_yaw - c_yaw * s.yawRate) / I_yaw;

		// World-frame motion derived from body-frame v and heading psi.
		// three.js's mesh.rotation.y = psi rotates the local +X axis to
		// (cos psi, 0, -sin psi) in world space, so we use the matching
		// sign convention for position derivatives. With this, positive
		// yaw rate turns the bot CCW as viewed from +Y (looking down).
		return {
			x:       s.v * Math.cos(s.psi),
			z:      -s.v * Math.sin(s.psi),
			v:       v_dot,
			th:      s.w,
			w:       w_dot,
			psi:     s.yawRate,
			yawRate: yawRate_dot,
		};
	}

	step(F, tau_yaw, dt) {
		const s = this.state;
		const add = (a, b, k) => {
			const out = {};
			for (const key of Object.keys(a)) out[key] = a[key] + (b[key] || 0) * k;
			return out;
		};
		const k1 = this.derivs(s, F, tau_yaw);
		const k2 = this.derivs(add(s, k1, dt / 2), F, tau_yaw);
		const k3 = this.derivs(add(s, k2, dt / 2), F, tau_yaw);
		const k4 = this.derivs(add(s, k3, dt),     F, tau_yaw);
		const out = {};
		for (const key of Object.keys(s)) {
			out[key] = s[key] + dt / 6 * (k1[key] + 2 * k2[key] + 2 * k3[key] + k4[key]);
		}
		this.state = out;
		return this.state;
	}
}
