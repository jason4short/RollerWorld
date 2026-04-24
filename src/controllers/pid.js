// PID controller for the balance bot.
// Swap this file out to experiment with different control laws.
//
// Interface (uniform across controllers):
//   c.reset()
//   c.updateOuter(sensors, gains, dt)   // no-op for single-rate PID
//   c.produceForce(sensors, gains, dt, motor)  → force (N) on cart
//
// sensors: { x, v, th, w }        quantized/noisy measurements from Sensors
// gains:   { Kp, Ki, Kd, Kx, Kv, Fmax }

export class PIDController {
  constructor() {
    this.I = 0;
  }

  reset() {
    this.I = 0;
  }

  // Single-rate controller — outer loop is a no-op; all work happens in
  // produceForce() which runs at the inner-loop rate.
  updateOuter() {}

  update(state, gains, dt) {
    const { Kp, Ki, Kd, Kx, Kv, Fmax } = gains;
    this.I += state.th * dt;

    // F = +(Kp·θ + Kd·θ̇ + Ki·∫θ) + (Kx·x + Kv·ẋ)
    // With +θ = bob tilting in +x direction, catching the lean requires
    // pushing the cart in +x (+F). Angle loop dominates; cart-position
    // terms keep the bot from drifting.
    let F = (Kp * state.th + Kd * state.w + Ki * this.I)
          + (Kx * state.x + Kv * state.v);

    if (F >  Fmax) F =  Fmax;
    if (F < -Fmax) F = -Fmax;
    return F;
  }

  produceForce(sensors, gains, dt /*, motor */) {
    return this.update(sensors, gains, dt);
  }
}
