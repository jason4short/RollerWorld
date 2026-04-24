// DC motor model (two motors combined into one horizontal-force term).
//
//   F = Km * duty - Kv * v   where duty = PWM / PWM_max
//
// Km lumps V_bat, torque constant, wheel radius, and number of motors.
// Kv is the back-EMF-induced drag referred to cart linear velocity.
// Not modeled: winding inductance, current limit, saturation nonlinearity,
// gear backlash, cogging torque.

export class Motor {
  constructor({ Km = 60, Kv = 10, PWM_max = 2000, deadband = 80 } = {}) {
    this.Km = Km;
    this.Kv = Kv;
    this.PWM_max = PWM_max;
    // deadband (in PWM counts): drive-train friction + driver stiction means
    // the cart doesn't move at all until |pwm| exceeds this. Real ArduRoller-
    // class hardware had deadbands around 5–10% of PWM_max.
    this.deadband = deadband;
  }

  forceFromPWM(pwm, v) {
    const abs = Math.abs(pwm);
    // Below the deadband no useful torque reaches the ground.
    const eff = abs <= this.deadband ? 0 : Math.sign(pwm) * (abs - this.deadband);
    const duty = eff / this.PWM_max;
    return this.Km * duty - this.Kv * v;
  }
}
