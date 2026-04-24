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

export class Pendulum {
  constructor(params) {
    this.params = params;                       // {M, m, L, R, Iw, cx, cp}
    this.state  = { x: 0, v: 0, th: 0, w: 0 };
  }

  setState(s) { this.state = { ...s }; }

  derivs(s, F) {
    const { M, m, L, R, Iw, cx, cp } = this.params;
    const sn = Math.sin(s.th), cs = Math.cos(s.th);
    const Meff = M + (Iw / (R * R));
    const D = Meff + m * sn * sn;
    const b1 = F + m * L * s.w * s.w * sn - cx * s.v;
    const b2 = G * sn - (F * R) / (m * L) - (cp * s.w) / (m * L * L);
    const a     = (b1 - m * cs * b2) / D;
    const alpha = (-cs * b1 + (Meff + m) * b2) / (L * D);
    return { x: s.v, v: a, th: s.w, w: alpha };
  }

  step(F, dt) {
    const s = this.state;
    const add = (a, b, k) => ({
      x:  a.x  + b.x  * k,
      v:  a.v  + b.v  * k,
      th: a.th + b.th * k,
      w:  a.w  + b.w  * k,
    });
    const k1 = this.derivs(s, F);
    const k2 = this.derivs(add(s, k1, dt / 2), F);
    const k3 = this.derivs(add(s, k2, dt / 2), F);
    const k4 = this.derivs(add(s, k3, dt),     F);
    this.state = {
      x:  s.x  + dt / 6 * (k1.x  + 2 * k2.x  + 2 * k3.x  + k4.x),
      v:  s.v  + dt / 6 * (k1.v  + 2 * k2.v  + 2 * k3.v  + k4.v),
      th: s.th + dt / 6 * (k1.th + 2 * k2.th + 2 * k3.th + k4.th),
      w:  s.w  + dt / 6 * (k1.w  + 2 * k2.w  + 2 * k3.w  + k4.w),
    };
    return this.state;
  }
}
