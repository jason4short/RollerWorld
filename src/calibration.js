// Motor calibration — what you'd do on the real bot with the wheels lifted
// off the ground: ramp PWM through a set of values, let each settle, and
// record the steady-state wheel speed. The resulting (pwm, speed) pairs
// build the PWM feed-forward table.
//
// The sim equivalent: run the physics with the bot pinned upright (bench
// mode) and drive the motor directly at each PWM level.

export class MotorCalibrator {
  constructor({ dt = 1 / 500 } = {}) {
    this.dt = dt;
  }

  // Returns a promise-free result synchronously (JS is fast enough that this
  // whole calibration takes <100 ms of wall time for ~20 s of sim time).
  //
  // opts: { pwmSteps, settleSec, sampleSec, params, motor }
  run({ pwmSteps, settleSec = 1.5, sampleSec = 0.5, params, motor }) {
    const results = [];   // {pwm, speed}[]
    for (const pwm of pwmSteps) {
      // Bench: wheels spinning freely, body held vertical. We model this by
      // integrating only the cart's 1-D equation of motion with force from
      // the motor alone (no pendulum coupling, since the body is pinned).
      //
      // M_eff · ẍ = F - cx·ẋ      with F = motor.forceFromPWM(pwm, v)
      const Meff = params.M + params.Iw / (params.R * params.R);
      let v = 0;

      // Settle
      const nSettle = Math.round(settleSec / this.dt);
      for (let i = 0; i < nSettle; i++) {
        const F = motor.forceFromPWM(pwm, v) - params.cx * v;
        v += (F / Meff) * this.dt;
      }

      // Sample — average velocity over the sample window
      const nSample = Math.round(sampleSec / this.dt);
      let sum = 0;
      for (let i = 0; i < nSample; i++) {
        const F = motor.forceFromPWM(pwm, v) - params.cx * v;
        v += (F / Meff) * this.dt;
        sum += v;
      }
      results.push({ pwm, speed: sum / nSample });
    }
    return results;
  }
}
