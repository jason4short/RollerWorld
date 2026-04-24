// Lean MLP with hand-coded backprop on typed arrays.
// Same math as Micrograd (tanh hidden, linear output, MSE loss, SGD+momentum)
// but runs batch updates without graph construction → orders of magnitude
// faster for training on thousands of samples. Swap to Micrograd for
// pedagogy; use this for speed.

export class MLP {
	constructor(inSize, hiddenSize, outSize) {
		this.inSize     = inSize;
		this.hiddenSize = hiddenSize;
		this.outSize    = outSize;

		// Xavier-ish init: scaled by 1/√fan_in
		const s1 = Math.sqrt(1 / inSize);
		const s2 = Math.sqrt(1 / hiddenSize);

		this.W1 = new Float64Array(hiddenSize * inSize);
		this.b1 = new Float64Array(hiddenSize);
		this.W2 = new Float64Array(outSize * hiddenSize);
		this.b2 = new Float64Array(outSize);
		for (let i = 0; i < this.W1.length; i++) this.W1[i] = (Math.random() * 2 - 1) * s1;
		for (let i = 0; i < this.W2.length; i++) this.W2[i] = (Math.random() * 2 - 1) * s2;

		// Momentum buffers (persist across epochs)
		this.mW1 = new Float64Array(this.W1.length);
		this.mb1 = new Float64Array(this.b1.length);
		this.mW2 = new Float64Array(this.W2.length);
		this.mb2 = new Float64Array(this.b2.length);
	}

	// Forward pass. Caches activations for backward.
	forward(x) {
		const H = this.hiddenSize, I = this.inSize, O = this.outSize;
		const h = new Float64Array(H);
		for (let i = 0; i < H; i++) {
			let s = this.b1[i];
			for (let j = 0; j < I; j++) s += this.W1[i * I + j] * x[j];
			h[i] = Math.tanh(s);
		}
		const out = new Float64Array(O);
		for (let i = 0; i < O; i++) {
			let s = this.b2[i];
			for (let j = 0; j < H; j++) s += this.W2[i * H + j] * h[j];
			out[i] = s;   // linear output
		}
		this._cache = { x, h, out };
		return out;
	}

	// Accumulate gradients for one sample into the provided buffers.
	backward(target, gW1, gb1, gW2, gb2) {
		const { x, h, out } = this._cache;
		const H = this.hiddenSize, I = this.inSize, O = this.outSize;

		// Output layer: ∂L/∂out = 2·(out − target)/O  (for mean-MSE)
		const dout = new Float64Array(O);
		for (let i = 0; i < O; i++) dout[i] = 2 * (out[i] - target[i]) / O;

		const dh = new Float64Array(H);
		for (let i = 0; i < O; i++) {
			gb2[i] += dout[i];
			const row = i * H;
			for (let j = 0; j < H; j++) {
				gW2[row + j] += dout[i] * h[j];
				dh[j]        += this.W2[row + j] * dout[i];
			}
		}

		// Hidden layer: tanh derivative is 1 − tanh²
		for (let i = 0; i < H; i++) {
			const dpre = dh[i] * (1 - h[i] * h[i]);
			gb1[i] += dpre;
			const row = i * I;
			for (let j = 0; j < I; j++) gW1[row + j] += dpre * x[j];
		}
	}

	// One training epoch over the full dataset. Returns mean MSE.
	trainEpoch(inputs, targets, lr, momentum) {
		const gW1 = new Float64Array(this.W1.length);
		const gb1 = new Float64Array(this.b1.length);
		const gW2 = new Float64Array(this.W2.length);
		const gb2 = new Float64Array(this.b2.length);
		const N = inputs.length;
		const O = this.outSize;

		let totalLoss = 0;
		for (let n = 0; n < N; n++) {
			const out = this.forward(inputs[n]);
			const tgt = targets[n];
			for (let i = 0; i < O; i++) {
				const e = out[i] - tgt[i];
				totalLoss += e * e;
			}
			this.backward(tgt, gW1, gb1, gW2, gb2);
		}

		// SGD with momentum. Average gradient by N so lr is comparable across
		// dataset sizes.
		const applySGD = (W, g, m) => {
			for (let i = 0; i < W.length; i++) {
				m[i] = momentum * m[i] - lr * (g[i] / N);
				W[i] += m[i];
			}
		};
		applySGD(this.W1, gW1, this.mW1);
		applySGD(this.b1, gb1, this.mb1);
		applySGD(this.W2, gW2, this.mW2);
		applySGD(this.b2, gb2, this.mb2);

		return totalLoss / N / O;
	}

	paramCount() {
		return this.W1.length + this.b1.length + this.W2.length + this.b2.length;
	}
}
