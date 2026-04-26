// RoadCanvas — paints the road network into a 2D canvas. The canvas
// serves two purposes:
//   1) As the ground texture mapped onto the heightfield (the bot
//      visually drives on what looks like asphalt-on-grass).
//   2) As the data source for the color sensor — the sensor walks rays
//      pixel-by-pixel through `pixels` and stops at the first non-road
//      reading.
// Single source of truth: paint once, sample many.
//
// Coordinates: the canvas covers a square `worldSize` × `worldSize` patch
// of world centered at the origin. World (x, z) maps to canvas (cx, cy):
//
//     cx = (x + worldSize / 2) · pxPerMeter
//     cy = (worldSize / 2 - z) · pxPerMeter
//
// (Note: world +z is "north" on the canvas, drawn as small canvas y. The
// renderer applies `texture.flipY = false` so the texture sample matches
// the canvas pixel layout 1:1.)

export class RoadCanvas {
	constructor(network, opts = {}) {
		this.network    = network;
		this.worldSize  = opts.worldSize  ?? 250;
		this.pxPerMeter = opts.pxPerMeter ?? 8;
		this.W = this.worldSize * this.pxPerMeter;
		this.H = this.worldSize * this.pxPerMeter;

		this.canvas = document.createElement('canvas');
		this.canvas.width  = this.W;
		this.canvas.height = this.H;
		// willReadFrequently=true tells the browser to keep this canvas
		// in software-readable form. We snapshot the pixel buffer once
		// after draw() and the sensor only reads from `this.pixels`, so
		// strictly this is no longer hot per-frame — but the flag also
		// silences a (correct) console warning the browser emits any
		// time getImageData lands on a GPU-backed canvas.
		this.ctx = this.canvas.getContext('2d', { willReadFrequently: true });

		this.draw();
	}

	// Paint the canvas. Grass-green background, asphalt-brown road
	// network drawn as wide rounded polylines + filled discs at each
	// node (covers any seams). After drawing, snapshot the pixel buffer
	// for cheap per-pixel queries.
	draw() {
		const { ctx, W, H, pxPerMeter, network } = this;

		// Grass background — same color as the heightfield mesh material.
		// (When the texture is sampled at points the road doesn't cover,
		// the bot sees this.)
		ctx.fillStyle = '#5a7a3a';
		ctx.fillRect(0, 0, W, H);

		// Roads — thick rounded strokes between centerline samples.
		ctx.strokeStyle = '#3a2a20';
		ctx.fillStyle   = '#3a2a20';
		ctx.lineWidth   = network.width * pxPerMeter;
		ctx.lineCap     = 'round';
		ctx.lineJoin    = 'round';

		for (let ei = 0; ei < network.edges.length; ei++) {
			if (network.edges[ei].blocked) continue;
			const samples = network.sampleEdge(ei, 80);
			ctx.beginPath();
			const p0 = this.worldToCanvas(samples[0].x, samples[0].z);
			ctx.moveTo(p0.cx, p0.cy);
			for (let i = 1; i < samples.length; i++) {
				const p = this.worldToCanvas(samples[i].x, samples[i].z);
				ctx.lineTo(p.cx, p.cy);
			}
			ctx.stroke();
		}

		// Filled discs at each node. The stroke's lineCap='round' already
		// rounds line endings, but multiple edges meeting at a node still
		// need this to avoid visible seams.
		for (const node of network.nodes) {
			const p = this.worldToCanvas(node.x, node.z);
			ctx.beginPath();
			ctx.arc(p.cx, p.cy, network.width * 0.7 * pxPerMeter, 0, Math.PI * 2);
			ctx.fill();
		}

		// Snapshot the buffer so the sensor's pixel-by-pixel sampling is
		// just an array index, not a getImageData call per pixel.
		this.imageData = ctx.getImageData(0, 0, W, H);
		this.pixels    = this.imageData.data;
	}

	worldToCanvas(x, z) {
		const half = this.worldSize / 2;
		return {
			cx: (x + half) * this.pxPerMeter,
			cy: (half - z) * this.pxPerMeter,
		};
	}

	// Is the world point on a road pixel? Asphalt is dark; grass is
	// medium-saturated green. A simple darkness check is enough — the
	// boundary between the two has lots of contrast.
	isRoadAt(x, z) {
		const { cx, cy } = this.worldToCanvas(x, z);
		const px = Math.floor(cx);
		const py = Math.floor(cy);
		if (px < 0 || px >= this.W || py < 0 || py >= this.H) return false;
		const idx = (py * this.W + px) * 4;
		// Asphalt RGB ≈ (58, 42, 32). Grass ≈ (90, 122, 58). Road = "dark".
		return this.pixels[idx] < 80;
	}
}
