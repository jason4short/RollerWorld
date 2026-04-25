// Park-style road network — a small graph of nodes (intersections, dead
// ends, through-points) connected by curved edges. The edges are sampled
// as Catmull-Rom splines through optional control points, so authoring is
// "drop a few waypoints and let the math draw a smooth path." The
// renderer builds road meshes from this data; later phases will sample
// the same data for the painted-ground texture and the color sensor.
//
// Why a graph and not a single closed loop:
//   - forks are decision points reactive controllers can't make on their
//     own — they need a goal or memory. That's the next teaching arc.
//   - blocking individual edges is a single boolean flip, so the same
//     world can host "single loop" demos and "open park" demos without
//     rebuilding the geometry.
//
// Coordinates: world x/z, same convention as the bot (right-handed,
// +y up, +z is the bot's right when heading is 0).

export class RoadNetwork {
	constructor() {
		this.nodes = [];     // [{ x, z, name }]
		this.edges = [];     // [{ a, b, controls, blocked }]
		this.width = 1.5;    // road width in meters
	}

	addNode(x, z, name = null) {
		this.nodes.push({ x, z, name });
		return this.nodes.length - 1;
	}

	addEdge(a, b, controls = []) {
		this.edges.push({ a, b, controls, blocked: false });
		return this.edges.length - 1;
	}

	blockEdge(idx)   { if (this.edges[idx]) this.edges[idx].blocked = true; }
	unblockEdge(idx) { if (this.edges[idx]) this.edges[idx].blocked = false; }

	// Sample one edge into N+1 points along its centerline. Returns
	// [{ x, z, tangent_x, tangent_z }] where the tangent is the unit
	// forward direction at that sample (used by the renderer to build
	// road quads and later by the color sensor).
	sampleEdge(idx, samples = 32) {
		const e = this.edges[idx];
		const knots = [this.nodes[e.a], ...e.controls, this.nodes[e.b]];
		const out = [];
		for (let i = 0; i <= samples; i++) {
			out.push(catmullRom(knots, i / samples));
		}
		// Tangents by forward difference; last sample reuses the previous tangent.
		for (let i = 0; i < out.length; i++) {
			const j = i < out.length - 1 ? i + 1 : i;
			const k = i < out.length - 1 ? i     : i - 1;
			let tx = out[j].x - out[k].x;
			let tz = out[j].z - out[k].z;
			const len = Math.hypot(tx, tz) || 1;
			out[i].tangent_x = tx / len;
			out[i].tangent_z = tz / len;
		}
		return out;
	}

	// Author the park. Eleven nodes, fourteen edges, three forks, two
	// dead-end spurs. Layout was hand-drawn — no procedural generation
	// here, the goal is "feels like a place" not "feels random".
	loadPark() {
		this.nodes = [];
		this.edges = [];

		// All park positions scale through this constant. Authoring is in
		// "park units"; the constant gives the park real-world size in m.
		// Bot is ~0.5 m wide and 1.5 m road = 3 bot-widths of corridor.
		const S = 5;

		// Nodes laid out in a roughly 110 m × 70 m park (after scaling).
		const A = this.addNode( -9 * S, -1 * S, 'west-entrance');
		const B = this.addNode( -3 * S, -3 * S, 'south-fork');
		const C = this.addNode(  4 * S, -4 * S, 'south-junction');
		const D = this.addNode(  9 * S, -2 * S, 'east-bend');
		const E = this.addNode( 10 * S,  4 * S, 'east-corner');
		const F = this.addNode(  5 * S,  5 * S, 'north-fork');
		const G = this.addNode( -2 * S,  4 * S, 'north-junction');
		const H = this.addNode( -7 * S,  2 * S, 'west-fork');
		const I = this.addNode(  1 * S,  0 * S, 'hub');
		const J = this.addNode( 12 * S, -5 * S, 'overlook');
		const K = this.addNode( -5 * S,  7 * S, 'grove');

		// Outer perimeter — the long lap. Control points keep each edge
		// from being a straight line; small offsets give organic curvature.
		this.addEdge(A, B, [{ x: -6   * S, z: -2.5 * S }]);
		this.addEdge(B, C, [{ x: -1   * S, z: -3.5 * S }, { x: 1 * S, z: -4.2 * S }]);
		this.addEdge(C, D, [{ x:  7   * S, z: -3.5 * S }]);
		this.addEdge(D, E, [{ x: 10.5 * S, z:  1   * S }]);
		this.addEdge(E, F, [{ x:  8   * S, z:  5   * S }]);
		this.addEdge(F, G, [{ x:  2   * S, z:  6   * S }]);
		this.addEdge(G, H, [{ x: -5   * S, z:  3.5 * S }]);
		this.addEdge(H, A, [{ x: -9   * S, z:  0   * S }]);

		// Cross-paths through the hub (I). These are the forks.
		this.addEdge(B, I, [{ x: -1 * S, z: -1   * S }]);
		this.addEdge(I, C, [{ x:  3 * S, z: -1.5 * S }]);
		this.addEdge(I, F, [{ x:  3 * S, z:  2   * S }, { x: 4 * S, z: 3.5 * S }]);
		this.addEdge(I, H, [{ x: -3 * S, z:  2   * S }]);

		// Spurs to dead-end features (overlook + grove).
		this.addEdge(C, J, [{ x:  8 * S, z: -5 * S }]);
		this.addEdge(G, K, [{ x: -3 * S, z:  6 * S }]);

		return this;
	}
}

// Uniform Catmull-Rom spline. `points` is the knot list including
// endpoints; t is 0..1 along the entire spline. Endpoints reuse the
// nearest interior knot as a phantom for the boundary tangents — kinks
// are fine here since edges meeting at a node may have intentionally
// different directions (path Y-junctions, real-life trail forks).
function catmullRom(points, t) {
	const n = points.length;
	if (n < 2) return { x: points[0].x, z: points[0].z };
	if (n === 2) {
		return {
			x: points[0].x + (points[1].x - points[0].x) * t,
			z: points[0].z + (points[1].z - points[0].z) * t,
		};
	}
	const segCount = n - 1;
	const segT     = Math.min(t * segCount, segCount - 1e-9);
	const seg      = Math.floor(segT);
	const u        = segT - seg;

	const p0 = points[Math.max(0, seg - 1)];
	const p1 = points[seg];
	const p2 = points[seg + 1];
	const p3 = points[Math.min(n - 1, seg + 2)];

	const u2 = u * u;
	const u3 = u2 * u;
	return {
		x: 0.5 * (
			(2 * p1.x) +
			(-p0.x + p2.x) * u +
			(2 * p0.x - 5 * p1.x + 4 * p2.x - p3.x) * u2 +
			(-p0.x + 3 * p1.x - 3 * p2.x + p3.x)     * u3
		),
		z: 0.5 * (
			(2 * p1.z) +
			(-p0.z + p2.z) * u +
			(2 * p0.z - 5 * p1.z + 4 * p2.z - p3.z) * u2 +
			(-p0.z + 3 * p1.z - 3 * p2.z + p3.z)     * u3
		),
	};
}
