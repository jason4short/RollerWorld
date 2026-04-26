// Recurrent neural network on top of the in-tree micrograd. A learnable
// stateful function — like an MLP, but each call also carries forward a
// hidden state that becomes part of the next call's input. The hidden
// state is just a vector of Values that gets re-bound each step;
// backprop through time happens for free because the autodiff graph
// naturally extends across step() calls.
//
// Two execution paths:
//
//   step(x) / parameters() / loss.backward()
//     Slow autodiff path used during training. Builds a graph of `Value`
//     nodes through the whole sequence; loss.backward() walks the graph
//     and accumulates gradients into the shared weights (Value.grad uses
//     `+=` so the same weight participating in N steps gets its
//     contributions summed automatically — that's BPTT).
//
//   stepRaw(x) / resetRaw()
//     Fast inference path. Pure number arithmetic against the trained
//     weights — no Value objects, no graph allocation. Used at the
//     400 Hz inner loop where running autodiff would be ridiculous.
//     Maintains its own hidden state in a Float64Array.
//
// IMPORTANT: call reset() between training sequences. It rebuilds the
// hidden state with fresh Values that have NO parents — that's what
// disconnects the new sequence's autodiff graph from the previous one.
// resetRaw() is the equivalent for inference.

import { Value } from './micrograd/engine.js';
import { Layer } from './micrograd/neuron.js';

export class RNNCell {
	constructor(inputSize, hiddenSize) {
		// Identity activation on each layer — we sum the two contributions
		// THEN apply tanh, matching the standard RNN equation:
		//   h_next = tanh(W_input · x + W_hidden · h_current + b)
		this.W_input  = new Layer(inputSize,  hiddenSize, x => x);
		this.W_hidden = new Layer(hiddenSize, hiddenSize, x => x);
		this.inputSize  = inputSize;
		this.hiddenSize = hiddenSize;
		// Xavier-ish init on weights, zero biases. The base Neuron init
		// (`random()*0.01` weights, random bias) is fine for shallow MLPs
		// but leaves RNN weights too small to propagate signal through
		// time, and biases spread enough to saturate tanh on step 1.
		this._initLayer(this.W_input,  inputSize);
		this._initLayer(this.W_hidden, hiddenSize);
		this.reset();
		this.resetRaw();
	}

	_initLayer(layer, fanIn) {
		const scale = 1 / Math.sqrt(fanIn);
		for (const n of layer.neurons) {
			for (const w of n.weights) w.data = (Math.random() * 2 - 1) * scale;
			n.bias.data = 0;
		}
	}

	// --- Training path (autodiff via micrograd Values) ----------------------

	reset() {
		this.hidden = Array.from({ length: this.hiddenSize }, () => new Value(0));
	}

	step(x) {
		const xs = x.map(v => v instanceof Value ? v : new Value(v));
		let inputContrib  = this.W_input.forward(xs);
		let hiddenContrib = this.W_hidden.forward(this.hidden);
		if (!Array.isArray(inputContrib))  inputContrib  = [inputContrib];
		if (!Array.isArray(hiddenContrib)) hiddenContrib = [hiddenContrib];
		this.hidden = inputContrib.map((iv, i) =>
			iv.add(hiddenContrib[i]).tanh()
		);
		return this.hidden;
	}

	parameters() {
		return this.W_input.parameters().concat(this.W_hidden.parameters());
	}

	// --- Inference path (pure number arithmetic) ----------------------------

	resetRaw() {
		if (!this.hiddenRaw) this.hiddenRaw = new Float64Array(this.hiddenSize);
		else this.hiddenRaw.fill(0);
	}

	// x: array (or typed array) of input numbers. Returns the new hidden
	// state as a Float64Array (also stored on this.hiddenRaw).
	stepRaw(x) {
		const H = this.hiddenSize, I = this.inputSize;
		const wIn  = this.W_input.neurons;
		const wH   = this.W_hidden.neurons;
		const next = new Float64Array(H);

		// h_next[i] = tanh( Σ W_input[i][j] x[j]  +  Σ W_hidden[i][k] h[k]  +  b )
		for (let i = 0; i < H; i++) {
			let s = wIn[i].bias.data + wH[i].bias.data;
			const wi = wIn[i].weights, wh = wH[i].weights;
			for (let j = 0; j < I; j++) s += wi[j].data * x[j];
			for (let k = 0; k < H; k++) s += wh[k].data * this.hiddenRaw[k];
			next[i] = Math.tanh(s);
		}
		this.hiddenRaw.set(next);
		return this.hiddenRaw;
	}
}

export class RNN {
	constructor(inputSize, hiddenSize, outputSize) {
		this.cell        = new RNNCell(inputSize, hiddenSize);
		this.output      = new Layer(hiddenSize, outputSize, x => x);   // linear
		this.hiddenSize  = hiddenSize;
		this.outputSize  = outputSize;
		// Xavier-ish init on the output projection too.
		const scale = 1 / Math.sqrt(hiddenSize);
		for (const n of this.output.neurons) {
			for (const w of n.weights) w.data = (Math.random() * 2 - 1) * scale;
			n.bias.data = 0;
		}
	}

	// --- Training (autodiff) ------------------------------------------------

	step(x) {
		const h = this.cell.step(x);
		const y = this.output.forward(h);
		return Array.isArray(y) ? y : [y];
	}
	reset() { this.cell.reset(); }
	parameters() { return this.cell.parameters().concat(this.output.parameters()); }

	// --- Inference (numbers only) -------------------------------------------

	resetRaw() { this.cell.resetRaw(); }

	stepRaw(x) {
		const h = this.cell.stepRaw(x);
		const O = this.outputSize;
		const out = new Float64Array(O);
		for (let i = 0; i < O; i++) {
			const neuron = this.output.neurons[i];
			let s = neuron.bias.data;
			const ws = neuron.weights;
			for (let k = 0; k < this.hiddenSize; k++) s += ws[k].data * h[k];
			out[i] = s;
		}
		return out;
	}

	// Total parameter count, for stats display.
	paramCount() { return this.parameters().length; }
}
