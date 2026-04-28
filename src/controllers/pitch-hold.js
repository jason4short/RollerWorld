// PitchHold — single-loop PD-on-pitch (with optional integrator and
// position terms). The simplest of the four controllers in the bot:
// pitch error in, force out, no outer velocity loop, no actuator model.
//
// "PitchHold" names what it does, not what it's built from — every
// controller in here uses PID-style math, so naming this one "PID"
// would be misleading.
//
// Interface (uniform across controllers):
//   c.reset()
//   c.updateVelocity(sensors, gains, dt)   // no-op for single-rate
//   c.produceForce(sensors, gains, dt, motor)  → force (N) on cart
//
// sensors: { x, v, pitch, pitch_rate }   quantized/noisy measurements
// gains:   { Kp, Ki, Kd, Kx, Kv, Fmax }

export class PitchHoldController {
  constructor() {
    this.pitch_integral = 0;
  }

  reset() {
    this.pitch_integral = 0;
  }

  // Single-rate controller — outer loop is a no-op; all work happens in
  // produceForce() which runs at the inner-loop rate.
  updateVelocity() {}

  update(state, gains, dt) {
    const { Kp, Ki, Kd, Kx, Kv, Fmax } = gains;
    this.pitch_integral += state.pitch * dt;

    // F = +(Kp·pitch + Kd·pitch_rate + Ki·∫pitch) + (Kx·x + Kv·ẋ)
    // With +pitch = bob tilting in +x direction, catching the lean requires
    // pushing the cart in +x (+F). Angle loop dominates; cart-position
    // terms keep the bot from drifting.
    // Position term uses encoder-measured body-frame distance (`x_body`)
    // not world-frame `x` — otherwise yawing leaves a stale world-x error
    // that the controller can't reduce by leaning along the new heading.
    const x_pos = state.x_body ?? state.x;
    let F = (Kp * state.pitch + Kd * state.pitch_rate + Ki * this.pitch_integral)
          + (Kx * x_pos + Kv * state.vel_cart);

    if (F >  Fmax) F =  Fmax;
    if (F < -Fmax) F = -Fmax;
    return F;
  }

  produceForce(sensors, gains, dt /*, motor */) {
    return this.update(sensors, gains, dt);
  }
}
