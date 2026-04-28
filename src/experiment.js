import { Pendulum } from './physics/pendulum.js';
import { PitchHoldController } from './controllers/pitch-hold.js';

// Headless experiment runner — sweeps a parameter and reports survival + IAE.
// Uses god-mode state (no sensors), so its tuning suggestions may be hotter
// than what actually works in the live sim with encoder quantization + noise.

export class ExperimentRunner {
  constructor(dt = 1 / 500) { this.dt = dt; }

  runTrial(params, gains, { th0 = 6, duration = 10, noise = 0 } = {}) {
    const plant = new Pendulum(params);
    plant.setState({ x: 0, vel_cart: 0, pitch: th0 * Math.PI / 180, pitch_rate: 0 });
    const pid = new PitchHoldController();
    // Headless trial — no actuator model, no yaw. innerUpdate returns
    // per-wheel torque; we just sum to recover the chassis force.
    const motorStub = { wheelbase: 1 };
    let t = 0, fell = false, iae = 0;
    while (t < duration) {
      const s = plant.state;
      const measured = { ...s, pitch: s.pitch + (Math.random() * 2 - 1) * noise };
      const { torque_left, torque_right } = pid.innerUpdate(measured, gains, this.dt, motorStub);
      const F = torque_left + torque_right;
      plant.step(F, 0, this.dt);
      t += this.dt;
      iae += Math.abs(plant.state.pitch) * this.dt;
      if (Math.abs(plant.state.pitch) > Math.PI / 2) { fell = true; break; }
    }
    return { survived: t, fell, iae };
  }

  sweep(paramName, values, baseParams, baseGains, opts, perTrial) {
    const results = [];
    for (const v of values) {
      const gains  = paramName in baseGains  ? { ...baseGains,  [paramName]: v } : baseGains;
      const params = paramName in baseParams ? { ...baseParams, [paramName]: v } : baseParams;
      const r = this.runTrial(params, gains, opts);
      results.push({ [paramName]: v, ...r });
      if (perTrial) perTrial({ [paramName]: v, ...r });
    }
    return results;
  }
}
