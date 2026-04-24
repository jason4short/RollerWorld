// UI layer — wraps DOM reads so the rest of the app uses plain objects.

export class UI {
  constructor() {
    this.logEl   = document.getElementById('log');
    this.statsEl = document.getElementById('stats');
  }

  num(id) { return +document.getElementById(id).value; }
  setNum(id, v) {
    const el = document.getElementById(id);
    if (el) el.value = v;
  }

  readParams() {
    return {
      M:  this.num('M'),
      m:  this.num('m'),
      L:  this.num('L'),
      R:  this.num('R'),
      Iw: this.num('Iw'),
      cx: this.num('cx'),
      cp: this.num('cp'),
    };
  }

  readGains() {
    return {
      Kp:   this.num('Kp'),
      Ki:   this.num('Ki'),
      Kd:   this.num('Kd'),
      Kx:   this.num('Kx'),
      Kv:   this.num('Kv'),
      Fmax: this.num('Fmax'),
    };
  }

  readArduGains() {
    return {
      bal_P:      this.num('bal_P'),
      bal_D:      this.num('bal_D'),
      bal_I:      this.num('bal_I'),
      p_vel:      this.num('p_vel'),
      wheel_P:    this.num('wheel_P'),
      wheel_I:    this.num('wheel_I'),
      wheel_D:    this.num('wheel_D'),
      ff_per_mps: this.num('ff_per_mps'),
      PWM_max:    this.num('PWM_max'),
      dead_zone:  this.num('dead_zone'),
    };
  }

  readMotor() {
    return {
      Km:       this.num('Km'),
      Kv:       this.num('Kv_motor'),
      PWM_max:  this.num('PWM_max'),
      deadband: this.num('deadband'),
    };
  }

  readController() {
    return document.getElementById('ctrlType').value;
  }

  readInit() {
    return { th0: this.num('th0'), noise: this.num('noise') };
  }

  readSensors() {
    return {
      ticks_per_rev: this.num('ticks_per_rev'),
      imu_noise:     this.num('imu_noise'),
      gyro_noise:    this.num('gyro_noise'),
    };
  }

  navEnabled() { return document.getElementById('navEnabled').checked; }

  readNavGains() {
    return {
      Kp_nav:      this.num('Kp_nav'),
      Kd_nav:      this.num('Kd_nav'),
      v_max:       this.num('nav_v_max'),
      a_max:       this.num('nav_a_max'),
      Kvel:        this.num('nav_Kvel'),
      linear_zone: this.num('nav_linear_zone'),
      lookahead:   this.num('nav_lookahead'),
      tiltLimit:   this.num('nav_tiltLimit'),
    };
  }

  readNavMode() { return document.getElementById('navMode').value; }

  readRates() {
    return {
      outerHz:  this.num('outerHz'),
      innerHz:  this.num('innerHz'),
      sensorHz: this.num('sensorHz'),
    };
  }

  log(t, msg) {
    this.logEl.textContent = `[${t.toFixed(2)}] ${msg}\n` + this.logEl.textContent;
  }

  // Snapshot of every numeric + select input (the full "tuning"). Matches
  // the key names used in presets so the same shape round-trips.
  readAll() {
    return {
      ...this.readParams(),
      th0: this.num('th0'), noise: this.num('noise'),
      ...this.readSensors(),
      ...this.readRates(),
      Km: this.num('Km'), Kv_motor: this.num('Kv_motor'),
      PWM_max: this.num('PWM_max'), deadband: this.num('deadband'),
      ...this.readGains(),
      ...this.readArduGains(),
      ...this.readNavGains(),
      navMode: this.readNavMode(),
      shoveOmega: this.num('shoveOmega'),
    };
  }

  writeAll(t) {
    for (const [k, v] of Object.entries(t)) {
      if (k === 'navMode') {
        const el = document.getElementById('navMode');
        if (el) { el.value = v; el.dispatchEvent(new Event('change')); }
      } else {
        this.setNum(k, v);
      }
    }
  }

  setStats(t, s, pilotTilt = 0) {
    const pilot = pilotTilt !== 0
      ? `   pilot=${(pilotTilt * 180 / Math.PI).toFixed(1)}°`
      : '';
    this.statsEl.textContent =
      `t=${t.toFixed(2)}s   θ=${(s.th * 180 / Math.PI).toFixed(1)}°   ` +
      `x=${s.x.toFixed(2)}m   v=${s.v.toFixed(2)}${pilot}`;
  }
}
