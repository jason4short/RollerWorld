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
      M:     this.num('M'),
      m:     this.num('m'),
      L:     this.num('L'),
      R:     this.num('R'),
      Iw:    this.num('Iw'),
      cx:    this.num('cx'),
      cp:    this.num('cp'),
      I_yaw: this.num('I_yaw'),
      c_yaw: this.num('c_yaw'),
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

  readPilotMode() {
    const el = document.getElementById('pilotMode');
    return el ? el.value : 'raw';
  }

  readYawGains() {
    return {
      Kyaw:      this.num('Kyaw'),
      MaxTauYaw: this.num('MaxTauYaw'),
    };
  }

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
      Kheading:           this.num('Kheading'),
      MaxYawRate:         this.num('MaxYawRate'),
      yaw_disable_radius: this.num('yaw_disable_radius'),
      yaw_speed_softness: this.num('yaw_speed_softness'),
    };
  }

  readNavMode() { return document.getElementById('navMode').value; }

  // Cascade gains — grouped by layer to mirror src/controllers/stack/.
  // Each layer sees only its own group; the orchestrator never sees a flat
  // bag. Mixer reuses Kvel/tiltLimit/a_max from the existing nav panel
  // since they describe the same physical quantities.
  readCascadeGains() {
    return {
      nav: {
        v_max:              this.num('nav_v_max'),
        a_max:              this.num('nav_a_max'),
        linear_zone:        this.num('nav_linear_zone'),
        lookahead:          this.num('nav_lookahead'),
        yaw_disable_radius: this.num('yaw_disable_radius'),
        Kp_nav:             this.num('Kp_nav'),
        MaxYawRate:         this.num('MaxYawRate'),
      },
      mixer: {
        Kvel:       this.num('nav_Kvel'),
        tiltLimit:  this.num('nav_tiltLimit'),
        a_max:      this.num('nav_a_max'),
        vel_lpf_tc: 0.1,
      },
      attitude: {
        pitch_P:      this.num('att_pitch_P'),
        pitch_D:      this.num('att_pitch_D'),
        pitch_I:      this.num('att_pitch_I'),
        force_max:    this.num('att_force_max'),
        heading_P:    this.num('att_heading_P'),
        yaw_rate_max: this.num('att_yaw_rate_max'),
        yaw_rate_P:   this.num('att_yaw_rate_P'),
        torque_max:   this.num('att_torque_max'),
      },
      wheels: {
        wheelbase:      this.num('wheelbase'),
        force_P:        this.num('force_P'),
        force_I:        this.num('force_I'),
        force_I_max:    this.num('force_I_max'),
        deadband_extra: this.num('deadband_extra'),
        PWM_max:        this.num('PWM_max'),
      },
    };
  }

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

  // Single source of truth for "what fields make up a tuning."
  // Numeric inputs read with `num()`. Select inputs and checkboxes need
  // special handling — listed separately.
  static TUNING_NUMERIC_IDS = [
    // physics
    'M', 'm', 'L', 'R', 'Iw', 'cx', 'cp', 'I_yaw', 'c_yaw',
    'th0', 'noise',
    // sensors & timing
    'ticks_per_rev', 'imu_noise', 'gyro_noise',
    'sensorHz', 'outerHz', 'innerHz',
    // motor
    'Km', 'Kv_motor', 'PWM_max', 'deadband',
    // PID
    'Kp', 'Ki', 'Kd', 'Kx', 'Kv', 'Fmax',
    // ArduBalance
    'bal_P', 'bal_D', 'bal_I', 'p_vel',
    'wheel_P', 'wheel_I', 'wheel_D', 'ff_per_mps', 'dead_zone',
    // Yaw control
    'Kyaw', 'MaxTauYaw',
    // Nav (drive)
    'Kp_nav', 'Kd_nav',
    'nav_v_max', 'nav_a_max', 'nav_Kvel',
    'nav_linear_zone', 'nav_lookahead', 'nav_tiltLimit',
    // Nav (yaw)
    'Kheading', 'MaxYawRate', 'yaw_disable_radius', 'yaw_speed_softness',
    // Cascade — Attitude pitch arm
    'att_pitch_P', 'att_pitch_D', 'att_pitch_I', 'att_force_max',
    // Cascade — Attitude yaw arm
    'att_heading_P', 'att_yaw_rate_max', 'att_yaw_rate_P', 'att_torque_max',
    // Cascade — Wheels
    'wheelbase', 'force_P', 'force_I', 'force_I_max', 'deadband_extra',
    // disturbance
    'shoveOmega',
    'disturbForceImpulse', 'disturbTauImpulse', 'disturbImuBias',
    // NN training
    'nnHidden', 'nnEpochs', 'nnSamples', 'nnLR',
  ];

  static TUNING_SELECT_IDS = ['ctrlType', 'navMode', 'nnMode', 'pilotMode', 'pitch_arm_mode', 'mixer_mode'];

  // Snapshot of every tuning-relevant input. All input IDs are kept
  // verbatim, so writeAll() is a clean inverse of readAll().
  readAll() {
    const out = {};
    for (const id of UI.TUNING_NUMERIC_IDS) {
      const el = document.getElementById(id);
      if (el) out[id] = +el.value;
    }
    for (const id of UI.TUNING_SELECT_IDS) {
      const el = document.getElementById(id);
      if (el) out[id] = el.value;
    }
    return out;
  }

  writeAll(t) {
    for (const [k, v] of Object.entries(t)) {
      if (UI.TUNING_SELECT_IDS.includes(k)) {
        const el = document.getElementById(k);
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
      `t=${t.toFixed(2)}s   pitch=${(s.pitch * 180 / Math.PI).toFixed(1)}°   ` +
      `x=${s.x.toFixed(2)}m   v=${s.vel_cart.toFixed(2)}${pilot}`;
  }
}
