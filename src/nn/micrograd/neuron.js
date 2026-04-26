import { Value } from './engine.js';

function random () {
    return Math.random() * 2 - 1;
}

class Neuron {
	/**
	 * Represents a neuron.
	 * @param {Number} inputCount - Number of inputs to the neuron.
	 * @param {Function} activationFn - Activation function to apply (e.g., tanh, relu, identity).
	 */
	constructor(inputCount, activationFn = x => x.tanh()) {
		this.weights = Array.from({ length: inputCount }, () => new Value(random()*.01));
		this.weights[0].label = 'w0';
		this.bias = new Value(random());
		this.bias.label = 'bias'
		this.activationFn = activationFn; // Store the activation function
	}	 

	/**
	 * Forward pass through the neuron.
	 * @param {Array} inputs - Input values to be dotted with the neuron's weights.
	 * @returns {Value} - Output of the neuron after applying the activation function.
	 */
	forward(inputs) {
		// Compute the weighted sum of inputs plus bias
		const weightedSum = this.weights.reduce((sum, weight, i) =>
			sum.add(weight.mul(inputs[i])),
			new Value(0.0)
		).add(this.bias);

		// Apply the configurable activation function
		return this.activationFn(weightedSum);
	}

    /**
     * Get all parameters in neuron
     * @returns {Array} all parameters in neuron
     */
    parameters() {
        return this.weights.concat([this.bias]);
    }
}

class Layer {

	/**
	 * Represents a layer of neurons.
	 * @param {Number} inputCount - Number of inputs to each neuron in the layer.
	 * @param {Number} outputCount - Number of neurons in the layer.
	 * @param {Function} activationFn - Activation function for the neurons in this layer.
	 */
	constructor(inputCount, outputCount, activationFn) {
		this.neurons = Array.from({ length: outputCount }, () => new Neuron(inputCount, activationFn));
	}

    /**
     * Forward pass through layer
     * @param {Array} x values to be dotted with weights in layer 
     * @returns {Array} outputs of layer 
     */
    forward (x) {
        let outs = [];
        for (let i = 0; i < this.neurons.length; i++) {
            outs.push(this.neurons[i].forward(x));
        }
        
        // If only one neuron, return it directly
        if (outs.length === 1) {
            return outs[0];
        }
        return outs;
    }

    /**
     * Get all parameters in layer
     * @returns {Array} all parameters in layer
     */
    parameters() {
        let params = [];
        this.neurons.forEach(neuron => {
            params = params.concat(neuron.parameters());
        });
        return params;
    }
}

class MLP {

	/**
	 * Represents a Multi-Layer Perceptron (MLP).
	 * @param {Number} inputSize - Number of inputs to the MLP.
	 * @param {Array} outputSizes - Array of sizes for each layer.
	 * @param {Function} hiddenActivationFn - Activation function for hidden layers.
	 * @param {Function} outputActivationFn - Activation function for the output layer.
	 */
	constructor(inputSize, 
				outputSizes, 
				hiddenActivationFn = x => x.tanh(), 
				outputActivationFn = x => x)
		{

		// Combine input size with the sizes of all layers
		const layerSizes = [inputSize, ...outputSizes];

		// Create layers with specified activation functions
		this.layers = [];

		for (let i = 0; i < outputSizes.length; i++) {
			const activationFn = i === outputSizes.length - 1 ? outputActivationFn : hiddenActivationFn;
			this.layers.push(new Layer(layerSizes[i], layerSizes[i + 1], activationFn));
		}
	}

    /**
     * Forward pass through MLP
     * @param {Array} x 
     * @returns {Array} output of MLP
     */
    forward (x) {
        this.layers.forEach(layer => {
            x = layer.forward(x);          
        });
        return x;
    }

    /**
     * Get all parameters in MLP
     * @returns {Array} all parameters in MLP
     */
    parameters() {
        let params = [];
        this.layers.forEach(layer => {
            params = params.concat(layer.parameters());
        });
        return params;
    }
    
    // Fast inference-only forward pass — pure arithmetic, no Value objects
    forwardRaw(x) {
        for (let li = 0; li < this.layers.length; li++) {
            const layer = this.layers[li];
            const isLastLayer = li === this.layers.length - 1;
            const out = new Array(layer.neurons.length);

            for (let ni = 0; ni < layer.neurons.length; ni++) {
                const neuron = layer.neurons[ni];
                let sum = neuron.bias.data;
                for (let wi = 0; wi < neuron.weights.length; wi++) {
                    sum += neuron.weights[wi].data * x[wi];
                }
                // Hidden layers use tanh, output layer is identity
                out[ni] = isLastLayer ? sum : Math.tanh(sum);
            }

            x = out;
        }
        return x;
    }

    // Fast inference using an external flat weight array (no internal neuron state needed)
    // Allows multiple cars to use different weights simultaneously
    forwardRawWithWeights(flatWeights, x) {
        let wi = 0;
        for (let li = 0; li < this.layers.length; li++) {
            const layer = this.layers[li];
            const isLastLayer = li === this.layers.length - 1;
            const inputSize = x.length;
            const out = new Array(layer.neurons.length);

            for (let ni = 0; ni < layer.neurons.length; ni++) {
                let sum = 0;
                for (let k = 0; k < inputSize; k++) {
                    sum += flatWeights[wi++] * x[k];
                }
                sum += flatWeights[wi++]; // bias
                out[ni] = isLastLayer ? sum : Math.tanh(sum);
            }

            x = out;
        }
        return x;
    }

    getWeights() {
        return this.layers.flatMap(layer =>
            layer.neurons.flatMap(neuron => [...neuron.weights.map(w => w.data), neuron.bias.data])
        );
    }
    
    randomize() {
        this.layers.forEach(layer => {
            layer.neurons.forEach(neuron => {
                neuron.weights.forEach(w => { w.data = (Math.random() * 2 - 1) * 0.01; });
                neuron.bias.data = Math.random() * 2 - 1;
            });
        });
    }

    setWeights(weights) {
        let index = 0;
        this.layers.forEach(layer => {
            layer.neurons.forEach(neuron => {
                neuron.weights.forEach((weight, i) => {
                    weight.data = weights[index++];
                });
                neuron.bias.data = weights[index++];
            });
        });

        if (index !== weights.length) {
            throw new Error("Mismatch between provided weights and network architecture.");
        }
    }
}

export { Neuron, Layer, MLP };