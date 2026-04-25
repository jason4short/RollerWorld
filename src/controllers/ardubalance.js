import { PWMTable } from './pwm-table.js';

// Cascaded controller inspired by the ArduBalance firmware.
//
// Two loops at different rates:
//
//   OUTER (~100 Hz)   angle PD → commanded cart velocity (vel_command)
//                     plus a slow auto-trim on the IMU zero-offset
//   INNER (~400 Hz)   speed PID + feed-forward → PWM → force
//
// Output is PWM; a Motor converts PWM + current wheel speed into a force
// on the cart (so motor back-EMF naturally limits top speed).

export class ArduBalanceController {
  constructor() {
    this.target_angle = 0;
    this.pwmTable = new PWMTable();   // feed-forward curve (linear until calibrated)
    this.reset();
  }

  reset() {
    this.balance_offset = 0;   // learned IMU zero-offset (rad)
    this.vel_command    = 0;   // outer-loop output: commanded cart velocity (m/s)
    this.speed_I        = 0;   // inner integrator (m·s because err·dt)
    this.last_vel_cart_meas     = 0;   // for derivative-on-measurement in inner loop
    this.speed_d_lpf    = 0;   // low-passed derivative estimate
  }

  // ------------------------------------------------------------------------
  // Outer angle loop — commands a cart velocity to catch the lean.
  // ------------------------------------------------------------------------
  updateVelocity(sensors, gains, dt) {
    const { bal_P, bal_D, bal_I, p_vel } = gains;

    // Apply learned sensor offset so the controller sees "true" tilt.
    const pitch     = sensors.pitch + this.balance_offset;
    const angle_err = pitch - this.target_angle;

    // Auto-trim the IMU zero: slowly integrate angle error WHEN quiet (near
    // upright and not being driven). The original firmware gated this on
    // `target_angle == 0` only — which meant long drives would leave the
    // trim frozen. Here we also check that the error itself is small, and
    // leak toward zero so it can't drift off during transients.
    const quiet = Math.abs(this.target_angle) < 0.01
               && Math.abs(angle_err)       < 0.04;   // ~2.3°
  
    if (quiet) this.balance_offset += bal_I * angle_err * dt;
    this.balance_offset *= (1 - dt / 60);   // ~60 s leak time constant

    // D on raw gyro, not d(err)/dt — avoids a derivative kick when the
    // pilot snaps target_angle to a new value. (The original firmware did
    // the same thing, probably by accident: its PID helper consumed gyro
    // directly as the rate input.)
    //
    // Combined effect: vel_command = PD(angle) + p_vel · v_measured.
    // See produceForce() for what the p_vel term actually controls.
    this.vel_command = bal_P * angle_err + bal_D * sensors.pitch_rate + p_vel * sensors.vel_cart;
  }

  // ------------------------------------------------------------------------
  // Inner speed loop — tracks vel_command with PID + feed-forward.
  // ------------------------------------------------------------------------
  produceForce(sensors, gains, dt, motor) {
    const { wheel_P, wheel_I, wheel_D, ff_per_mps, PWM_max, dead_zone } = gains;

    // Substituting vel_command's definition:
    //   speed_err = vel_command − v = bal_PD + (p_vel − 1) · v
    // So p_vel controls the inner loop's effective velocity coefficient:
    //   p_vel < 1  → net braking on wheel speed
    //   p_vel = 1  → neutral (pure angle tracking)
    //   p_vel > 1  → net boosting (amplifies outer-loop authority; must be
    //                matched by enough bal_P to stay stable)
    const speed_err = this.vel_command - sensors.vel_cart;

    // Integrator with anti-windup: stop accumulating while PWM is saturated.
    const saturated = this.lastPWM !== undefined
                   && Math.abs(this.lastPWM) >= PWM_max - 1;
    if (!saturated) this.speed_I += speed_err * dt;

    // Derivative on measurement (not error) — no kick when vel_command jumps.
    // Low-passed because the sensor samples slower than the inner loop, so
    // raw Δv/dt has aliasing spikes at the sensor rate. Time constant ~20 ms
    // cleanly filters the 100 Hz sensor boundaries at 400 Hz inner rate.
    const raw_d = -(sensors.vel_cart - this.last_vel_cart_meas) / dt;
    this.last_vel_cart_meas = sensors.vel_cart;
    const time_constant = 0.02;
    const alpha = dt / (time_constant + dt);
    this.speed_d_lpf = (1 - alpha) * this.speed_d_lpf + alpha * raw_d;

    // Feed-forward from the PWM table (calibrated) or linear fallback
    // (using ff_per_mps as the slope). The table steps over motor deadband
    // and handles the nonlinear top end, so the PID doesn't have to wind up.
    this.pwmTable.linearSlope = ff_per_mps;
    const ff = this.pwmTable.pwmFromSpeed(this.vel_command);
    let pwm = ff
            + wheel_P * speed_err
            + wheel_I * this.speed_I
            + wheel_D * this.speed_d_lpf;

    // Motor driver deadband compensation: hops over the "stuck" PWM region
    // around zero. Applied AFTER the feedback path so zero command is still
    // zero PWM.
    if (pwm > 0) pwm += dead_zone;
    if (pwm < 0) pwm -= dead_zone;

    pwm = Math.max(-PWM_max, Math.min(PWM_max, pwm));
    this.lastPWM = pwm;
    return motor.forceFromPWM(pwm, sensors.vel_cart);
  }
}
