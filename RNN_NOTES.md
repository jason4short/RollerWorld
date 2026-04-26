# RNN and adaptive learning — notes

Captured while adding the RNN pitch controller to Roller. Not a textbook — a reference that ties each idea back to something that's already in this codebase, so the concept has a hook.

Audience: future-Jason coming back to this, and the friend Roller is being built for. Vocabulary first, then the formal name in parentheses.

---

## What an RNN actually is

A neural net with **internal state that persists between calls**. Each step, the network sees:

- the current input
- its own previous hidden state

and emits:

- the current output
- a new hidden state

The hidden state is just a vector of numbers. It's stored as a member variable on the network object. Every call evolves it. Reset between sequences.

The whole equation:

```
h_next = tanh(W_input · x + W_hidden · h_current + b)
y      = W_output · h_next
```

That's it. The "memory" is the hidden state vector. The "learnable dynamics" are the weights that decide how it evolves.

**Hook into your world**: ArduBalance's auto-trim is hand-designed memory — `balance_offset` is a 1-dim hidden state with a hand-coded update rule (`+= K_I · pitch_err · dt`). An RNN generalizes this to "vector hidden state with learnable update rule." Same idea, more flexibility, but you pay for it (see "When RNNs earn their keep" below).

Code: `src/nn/rnn.js` — `RNNCell` and `RNN` on top of micrograd. `step()` is the autodiff training path; `stepRaw()` is the pure-numbers inference path used at the 400 Hz inner loop.

---

## Training: backprop through time (BPTT)

To train, you need gradients of the loss with respect to the weights. The loss might come at the end of a sequence ("bot fell over after 50 steps") or at each step ("motor force was wrong at step k").

Conceptually you "unroll" the RNN through time:

```
h_0 → step → h_1 → step → h_2 → ... → h_N
        ↑           ↑           ↑
       x_0         x_1         x_N
```

Each step uses the **same weights**. Backprop through this unrolled graph is **backprop through time** (BPTT). For shared weights, gradient contributions across all steps are **summed** (because `Value.grad += ...`).

In Roller, BPTT comes for free because we built on top of micrograd's scalar autodiff. Each `step()` call extends the autodiff graph; `loss.backward()` walks the whole thing. No special "unrolling" code.

**The catch**: gradients get multiplied by `W_hidden` once per step. If its eigenvalues are < 1 in magnitude, gradients **vanish**; > 1, they **explode**. Vanilla RNNs struggle to learn dependencies past ~10–30 steps. **LSTMs** and **GRUs** were invented for this — they have gates that selectively pass gradient through time, sidestepping the vanish/explode trap.

---

## Training gotchas (lessons from when it didn't work)

When we first tested the cell on a running-sum task, training diverged. Five things had to be right:

1. **Xavier-scaled weight init** (~1/√fan_in). Default `random()*0.01` was too tiny — signal couldn't propagate through W_hidden, hidden state stayed near zero.
2. **Zero biases**. Default random biases pushed tanh into saturation immediately.
3. **Per-element gradient clipping** (~±1). Shared-weight gradients accumulate over every step; one bad step will spike them and derail SGD.
4. **Mean loss across the sequence, not sum**. Otherwise lr has to be re-tuned every time you change sequence length.
5. **Pick a task that genuinely requires memory.** Running-sum had a shortcut ("output ≈ current input" gave okay loss for short sequences) AND a scale problem (cumulative sum grew past tanh's linear range). Delayed-copy worked first try.

---

## When RNNs earn their keep

The question that mattered: "Could I just hand-build the integrator and feed it as an extra input to the feedforward NN?"

**Yes. And you'd usually do it better than the RNN, with less data and faster training.**

ArduBalance's auto-trim already does exactly this. It's hand-engineered memory. If you know the right derived feature (running integral of pitch error), hand-engineering wins. Smaller, faster, more reliable, more interpretable.

**RNNs earn their keep when**:

1. **You don't know the right feature.** Sensor fusion, novel terrain detection, surface classification. The right derived signal isn't obvious; let the network find it.
2. **You need multiple temporal features at once.** Slow integral for trim + fast derivative for slip + autocorrelation for surface — one hidden state can encode several.
3. **The right time constant depends on context.** Hand integrator decays at a fixed rate; RNN's dynamics are learned and can be input-dependent.

**The deeper framing — same as the cascade NNs:**

> An RNN is *learnable feature engineering for time series.* If you know the function, hand-write it. Learn what you must.

---

## Frozen offline NN vs always-learning integrator

Two genuinely different paradigms, and the one we built (RNN) is the *narrower* one.

| | Your integrator | Trained RNN (as built) |
|-|-|-|
| When it learns                  | Always (every tick) | Once, offline           |
| What it adapts to               | Anything producing a steady error signal | Only what was in training data |
| Handles a NEW disturbance shape | Yes (adapts in real time)               | No (weights frozen)     |
| Runtime hidden state            | One number                              | 12-dim vector            |

The integrator is **online learning** — single-parameter, but real-time adaptation. The trained RNN is **offline learning** — batch training, then frozen.

The mismatch between training distribution and deployment is called **distribution shift**, and it's one of the central problems in ML.

### Three practical answers to distribution shift

1. **Train robustly.** Generate training data with every disturbance shape you can think of — vibration, gyro aliasing, motor faults, payload changes. The frozen network has been "vaccinated" against them. The formal name is **domain randomization** — the workhorse of modern sim-to-real RL.

2. **Hybrid: frozen NN + classical adaptive layer.** The NN does bulk pattern matching; a small online adaptive controller on top (like your integrator) handles drift and disturbances the NN didn't see. **This is how most real systems are actually built.** Tesla autopilot, Boston Dynamics locomotion, modern drones — all hybrid. *Use what you must, hand-engineer what you can.*

3. **Online learning.** Keep updating the network's weights at runtime as new data comes in. Theoretically clean. Practically brutal: who decides "right answer" right now? Online gradient steps can cause sudden behavior changes that crash the bot. Non-i.i.d. data breaks standard SGD assumptions. Mostly avoided in real systems.

**For a balance bot, hybrid is the right answer.** Train the RNN to be the bulk imitator. Keep the integrator running on top. RNN handles 95% in distribution; integrator catches the drift everything else missed.

---

## Bridge to RL

The "always learning, even when the bot changes" property you described is exactly what **reinforcement learning** is designed for. RL agents act, observe, get reward feedback, and update their policy continuously.

The catch: RL has the *same* distribution-shift problem in a different form. If the reward signal is wrong, sparse, or hackable, the agent learns garbage. RL training is *much* harder to debug than supervised + integrator. The hyperparameters are sharp; the failure modes are subtle.

In modern robotics, RL is a focused tool for specific subproblems where (a) there's a clean reward, (b) lots of sim experience is cheap, (c) classical control alone can't capture the policy. Locomotion, manipulation, RLHF for LLMs. Most everything else is still supervised learning + classical control + adaptive correction.

---

## One-sentence summary

> An RNN is a learnable, vector-valued integrator with input-dependent dynamics. It earns its keep when you don't know the right feature to extract by hand. Even when you use one, the right system architecture is usually **frozen learned model + hand-coded adaptive layer** — not pure online learning, and not pure imitation either.

---

## What's in Roller right now

- `src/nn/rnn.js` — RNN cell on top of micrograd. Both autodiff (training) and stepRaw (inference) paths.
- `src/nn/micrograd/` — vendored copy of the scalar-autodiff library, so Roller is self-contained.
- `src/controllers/stack/attitude.js` — three pitch modes: `rule` (PD + auto-trim), `nn` (feedforward MLP), `rnn` (recurrent net). Pick from the dropdown in the Attitude panel.
- `src/nn/trainer.js`'s `trainAttitudePitchRNN` — episodic BPTT against the rule controller. First-cut data has no temporal structure (independent random states per step), so the trained RNN matches the rule but the hidden state isn't doing useful work yet. The bias-drift demo (where memory actually matters) is unbuilt.
