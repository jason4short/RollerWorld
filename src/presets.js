// Named parameter bundles. Click a preset button in the UI to load.

export const PRESETS = {
  arduroller: {
    // physics — M/Iw sized for heavy RC offroad tires (mass concentrated at
    // rim, ~1 kg each, total Iw ≈ 2·m_wheel·R²), plus a beefier base.
    M: 1.0, m: 1.5, L: 0.5, R: 0.1, Iw: 0.02, cx: 0.3, cp: 0.005,
    I_yaw: 0.05, c_yaw: 0.2,
    Kyaw: 4, MaxTauYaw: 6,
    th0: 4, noise: 0,
    // sensors & timing
    ticks_per_rev: 815, imu_noise: 0.002, gyro_noise: 0.01,
    outerHz: 100, innerHz: 400, sensorHz: 100,
    // motor
    Km: 80, Kv_motor: 10, PWM_max: 2000, deadband: 80,
    // idealized PID (force)
    Kp: 800, Ki: 0, Kd: 60, Kx: 2, Kv: 1, Fmax: 60,
    // ArduBalance cascaded
    bal_P: 120, bal_D: 1.0, bal_I: 0.0,
    p_vel: 1.2,
    wheel_P: 600, wheel_I: 40, wheel_D: 5, ff_per_mps: 250,
    dead_zone: 0,
    // navigation — PD fallback + square-root braking profile params
    Kp_nav: 0.15,
    nav_v_max: 3.0, nav_a_max: 1.5, nav_Kvel: 0.15,
    nav_linear_zone: 0.05, nav_lookahead: 0.3, nav_tiltLimit: 0.25,
    Kheading: 2, MaxYawRate: 1.5,
    yaw_disable_radius: 0.2,
  },
  testbot: {
    M: 1.0, m: 0.3, L: 0.5, R: 0.06, Iw: 0.001, cx: 0.2, cp: 0.005,
    I_yaw: 0.02, c_yaw: 0.15,
    Kyaw: 3, MaxTauYaw: 3,
    th0: 6, noise: 0,
    ticks_per_rev: 400, imu_noise: 0.001, gyro_noise: 0.005,
    outerHz: 100, innerHz: 400, sensorHz: 100,
    Km: 30, Kv_motor: 3, PWM_max: 2000, deadband: 50,
    Kp: 200, Ki: 0, Kd: 10, Kx: 5, Kv: 3, Fmax: 25,
    bal_P: 60, bal_D: 0.8, bal_I: 0.0,
    p_vel: 1.2,
    wheel_P: 500, wheel_I: 30, wheel_D: 3, ff_per_mps: 200,
    dead_zone: 0,
    Kp_nav: 0.2,
    nav_v_max: 1.5, nav_a_max: 2.0, nav_Kvel: 0.1,
    nav_linear_zone: 0.03, nav_lookahead: 0.2, nav_tiltLimit: 0.20,
    Kheading: 2.5, MaxYawRate: 1.5,
    yaw_disable_radius: 0.15,
  },
};
