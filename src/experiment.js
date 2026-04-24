import { Pendulum } from './physics/pendulum.js';
import { PIDController } from './controllers/pid.js';

// Headless experiment runner — sweeps a parameter and reports survival + IAE.
// Uses god-mode state (no sensors), so its tuning suggestions may be hotter
// than what actually works in the live sim with encoder quantization + noise.

export class ExperimentRunner {
  constructor(dt = 1 / 500) { this.dt = dt; }

  runTrial(params, gains, { th0 = 6, duration = 10, noise = 0 } = {}) {
    const plant = new Pendulum(params);
    plant.setState({ x: 0, v: 0, th: th0 * Math.PI / 180, w: 0 });
    const pid = new PIDController();
    let t = 0, fell = false, iae = 0;
    while (t < duration) {
      const s = plant.state;
      const measured = { ...s, th: s.th + (Math.random() * 2 - 1) * noise };
      const F = pid.update(measured, gains, this.dt);
      plant.step(F, this.dt);
      t += this.dt;
      iae += Math.abs(plant.state.th) * this.dt;
      if (Math.abs(plant.state.th) > Math.PI / 2) { fell = true; break; }
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
