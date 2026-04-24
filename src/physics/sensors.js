// Simulates what the firmware actually observes:
//   - IMU pitch angle + gyro rate, with Gaussian noise
//   - Wheel encoder: quantized position, speed derived from tick delta over
//     the sensor period. Firmware reads an I2C encoder module at ~100 Hz.
//
// Without this layer, controllers see "god-mode" state and tune too hot.

export class Sensors {
  constructor() {
    this.reset();
  }

  reset() {
    this.lastTicks = 0;
    this.lastTime  = 0;
    this.lastSpeed = 0;
  }

  // Gaussian via Box–Muller.
  _randn() {
    const u = Math.max(1e-12, Math.random());
    const v = Math.random();
    return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
  }

  // params: plant params (we need R and L)
  // cfg:    { ticks_per_rev, imu_noise, gyro_noise }
  sample(plantState, params, cfg, now) {
    const { R, L } = params;
    const circ = 2 * Math.PI * R;
    const ticks = Math.round((plantState.x / circ) * cfg.ticks_per_rev);

    let speed = this.lastSpeed;
    const dt = now - this.lastTime;
    if (dt > 1e-9) {
      const dTicks = ticks - this.lastTicks;
      speed = (dTicks * circ / cfg.ticks_per_rev) / dt;
      this.lastTicks = ticks;
      this.lastTime  = now;
      this.lastSpeed = speed;
    }

    const x  = ticks * circ / cfg.ticks_per_rev;
    const v  = speed;
    const th = plantState.th + this._randn() * cfg.imu_noise;
    const w  = plantState.w  + this._randn() * cfg.gyro_noise;

    // CoM position / velocity in world X.
    //   x_CoM = x_cart + L·sin(θ)
    //   v_CoM = v_cart + L·cos(θ)·θ̇
    // For small θ these are ~x + L·θ and v + L·ω. The point: during a
    // commanded forward push the cart briefly moves backward (cart-coupling),
    // but L·sin(θ) grows positive as the body tips forward — the two terms
    // cancel and x_CoM barely moves. That's what nav should close the loop
    // on, not x_cart, or it'll over-react to the non-minimum-phase startup.
    const cs = Math.cos(th);
    const sn = Math.sin(th);
    const x_CoM = x + L * sn;
    const v_CoM = v + L * cs * w;

    return { x, v, th, w, x_CoM, v_CoM };
  }
}
