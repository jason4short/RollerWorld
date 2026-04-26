class Value {

    constructor(data, _children = [], _op = '', label = '') {
        this.data 		= data;
        this.grad 		= 0.0;
        this._backward 	= () => {return null;};
        this._parents 	= [...new Set(_children)];
        this._op 		= _op;
        this.label 		= label;
    }

    add(other) {
        if (!(other instanceof Value)) {other = new Value(other)}

        let out = new Value(this.data + other.data, [this, other], '+');
        out._backward = () => {
        	// apply the grad to the two parents
            this.grad  += 1.0 * out.grad  // Derivative of "this" with respect to the output
            other.grad += 1.0 * out.grad  // Derivative of "other" with respect to the output
        }
        return out
    }
    
	sub(other) {
        if (!(other instanceof Value)) {other = new Value(other)}

		// Create a new Value instance with the result of subtraction
		let out = new Value(this.data - other.data, [this, other], '-');

		// Define the backward pass
		out._backward = () => {
			this.grad  += 1.0 * out.grad;  // Derivative of "this" with respect to the output
			other.grad -= 1.0 * out.grad;  // Derivative of "other" with respect to the output
		};

		return out; // Return the resulting Value
	}

    mul(other) {
        if (!(other instanceof Value)) {other = new Value(other)}

        let out = new Value(this.data * other.data, [this, other], '*');
        out._backward = () => {
            this.grad += other.data * out.grad
            other.grad += this.data * out.grad
        }

        return out
    }

    pow(other) {
        if (typeof other !== 'number') {throw new Error('Can only raise to a number')}

        let out = new Value(this.data ** other, [this], `**${other}`);
        out._backward = () => {
            this.grad += other * (this.data ** (other - 1)) * out.grad
        }

        return out
    }


    truediv(other) {
        if (!(other instanceof Value)) {other = new Value(other)}

        let out = new Value(this.data * other.data ** -1, [this, other], '/');
        out._backward = () => {
            this.grad += 1.0 / other.data * out.grad
            other.grad += -this.data / (other.data ** 2) * out.grad
        }

        return out

    }

	tanh() {
		// Calculate tanh using the formula
		const x = this.data;
		// tanh output
		const t = (Math.exp(2 * x) - 1) / (Math.exp(2 * x) + 1);

		// Create the output Value with tanh result
		const out = new Value(t, [this], 'tanh');

		// Backward pass
		out._backward = () => {
			this.grad += (1 - out.data ** 2) * out.grad;
		};

		return out;
	}

	relu() {
		// Compute ReLU output: max(0, x)
		const x = this.data;
		const t = Math.max(0, x);

		// Create a new Value instance for the output
		const out = new Value(t, [this], 'relu');

		// Backward pass
		out._backward = () => {
			this.grad += (x > 0 ? 1 : 0) * out.grad; // Gradient is 1 if x > 0, else 0
		};

		return out;
	}

	leakyRelu(alpha = 0.01) {
		const x = this.data;
		const t = x > 0 ? x : alpha * x;

		const out = new Value(t, [this], 'leaky_relu');

		out._backward = () => {
			this.grad += (x > 0 ? 1 : alpha) * out.grad;
		};

		return out;
	}

	sigmoid() {
		// Compute sigmoid output: 1 / (1 + exp(-x))
		const x = this.data;
		const t = 1 / (1 + Math.exp(-x));

		// Create a new Value instance for the output
		const out = new Value(t, [this], 'sigmoid');

		// Backward pass
		out._backward = () => {
			const sigmoidOutput = out.data; // Use the computed sigmoid value
			this.grad += sigmoidOutput * (1 - sigmoidOutput) * out.grad;
		};

		return out;
	}

	elu(alpha = 1.0) {
		const x = this.data;
		const t = x > 0 ? x : alpha * (Math.exp(x) - 1);

		const out = new Value(t, [this], 'elu');

		out._backward = () => {
			this.grad += (x > 0 ? 1 : t + alpha) * out.grad;
		};

		return out;
	}

	swish() {
		const x = this.data;
		const sigmoidX = 1 / (1 + Math.exp(-x));
		const t = x * sigmoidX;

		const out = new Value(t, [this], 'swish');

		out._backward = () => {
			const sigmoidGrad = sigmoidX * (1 - sigmoidX);
			this.grad += (sigmoidX + x * sigmoidGrad) * out.grad;
		};

		return out;
	}

    exp() {

        let x = this.data
        let t = Math.exp(x)
        let out = new Value(t, [this], 'exp')

        out._backward = () => {
            this.grad += out.data * out.grad
        }

        return out
    }

    backward() {

        // topological order all of the children in the graph
        let topo = []
        let seen = new Set()

        let build_topo = (node) => {
            if (seen.has(node)) {
                return
            }

            seen.add(node)

            for (let child of node._parents) {
                build_topo(child)
            }

            topo.push(node)
        }

        build_topo(this)
        this.grad = 1.0

        for (let node of topo.reverse()) {
            node._backward()
        }
    }
}

export { Value }