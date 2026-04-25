// Procedural terrain — a height function `h(x, z)` that defines rolling
// hills across the park. Single source of truth: the ground mesh, the
// road meshes, the trees, and (later) the bot's render position and
// slope-force physics all sample from this same function. Anything
// height-related in the world goes through `heightAt`.
//
// Implementation: a sum of three sin/cos octaves with carefully chosen
// frequencies and amplitudes. No noise library — the explicit form is
// fast, deterministic, and easy to read. The lowest octave is the gross
// landform; the higher octaves are rolling detail.
//
// Why hand-rolled instead of Perlin/simplex: this is for a teaching
// world, not a game world. Rolling sin-hills are smooth, gradient-
// continuous (so slope physics behaves), and don't have any of the
// banding artifacts that low-octave Perlin can show without proper
// fractal stacking.

const A1 = 1.4,  FX1 = 0.045, FZ1 = 0.055, PX1 = 0.0,  PZ1 =  0.0;
const A2 = 0.7,  FX2 = 0.115, FZ2 = 0.090, PX2 = 1.3,  PZ2 = -0.7;
const A3 = 0.25, FX3 = 0.240, FZ3 = 0.220, PX3 = -2.0, PZ3 =  1.5;

export function heightAt(x, z) {
	return (
		A1 * Math.sin(x * FX1 + PX1) * Math.cos(z * FZ1 + PZ1) +
		A2 * Math.sin(x * FX2 + PX2) * Math.cos(z * FZ2 + PZ2) +
		A3 * Math.sin(x * FX3 + PX3) * Math.cos(z * FZ3 + PZ3)
	);
}

// Slope (rise/run) along the unit forward direction (forwardX, forwardZ).
// Approximates the local terrain gradient with a 4-sample finite
// difference. Returns a dimensionless slope ≈ tan(slope_angle); positive
// = climbing.
//
// Used by the bot's render to align the visual chassis to the ground if
// we ever want it (currently the bot stays gravity-vertical, per the
// "balance bots don't care about slope" principle), and by the future
// slope-force disturbance: F_slope ≈ -m·g·slope.
export function slopeAlong(x, z, forwardX, forwardZ) {
	const eps = 0.2;
	const dh_dx = (heightAt(x + eps, z) - heightAt(x - eps, z)) / (2 * eps);
	const dh_dz = (heightAt(x, z + eps) - heightAt(x, z - eps)) / (2 * eps);
	return dh_dx * forwardX + dh_dz * forwardZ;
}
