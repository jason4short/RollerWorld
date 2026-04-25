# A tour of Roller

Welcome. Roller is a 2-wheel balance-bot simulator, but really it's a teaching tool — a place to develop intuition about how real balance bots think. The code is laid out so you can walk through the rooms in order.

If you're just here to play: open `index.html`, click **Start**, drag the on-screen joystick. If you're here to learn, keep reading.

---

## The big idea: a layered control stack

A real balance bot does not have one big controller that takes "where do I want to go?" and outputs "how much PWM should each motor get?" Anything that tries to do that is a mess to tune, a mess to debug, and impossible to teach.

Instead, the work is split into **layers**, each with a small job. Higher layers think in human terms (positions, headings); lower layers think in motor terms (forces, PWM). Each layer talks only to the layer above and the layer below. Like a building.

```
                          ┌──────────────────────────┐
   "go to that point"  →  │   Nav                    │  → vel_target_body, heading_target
                          ├──────────────────────────┤
                          │   Nav Mixer              │  → pitch_target, yaw_target
                          ├──────────────────────────┤
                          │   Attitude               │  → force_fwd, torque_yaw
                          ├──────────────────────────┤
                          │   Wheels                 │  → pwm_left, pwm_right
                          └──────────────────────────┘
                                       ↓
                                 [ the bot ]
                                       ↓
                                  sensors  ──→  feeds back into every layer
```

### Why these particular layers?

- **Nav** owns position and heading — it knows where the bot is and where it should go. Output: a body-frame velocity command and a target heading. It does *not* know what tilt or torque those mean, by design.

- **Nav Mixer** turns nav's velocity command into a body tilt. To go forward, a balance bot tips forward; to slow down, it tips backward. The mixer is the small loop that closes the velocity error using lean. (`pitch_target = Kvel · (vel_target − vel_actual)`.) Yaw passes through.

- **Attitude** is the angle controller. Given a target tilt and a target heading, it produces the *force* the chassis needs (longitudinal) and the *torque* the chassis needs (yaw). Pitch is PD with a slow auto-trim of the IMU zero. Yaw is a heading→rate→torque cascade.

- **Wheels** owns all motor math. It mixes `(force_fwd, torque_yaw)` into per-wheel forces, runs a closed-loop force tracker per wheel against the motor model, and emits PWM. Deadband, saturation, feed-forward live here and only here.

### Why layers, really?

Three reasons, each worth understanding.

1. **Locality of failure.** When the bot oscillates, the *layer* tells you where to look. If pitch overshoots, it's Attitude's pitch. If the bot weaves while driving straight, it's Wheels' yaw mix or per-wheel tracking. If the heading hunts, it's Attitude's yaw. You don't go fishing through one big function.

2. **Substitution.** Want to learn neural-net control? Replace one layer with an MLP. The layer's interface is tiny — a few numbers in, a few out — so the NN's job is well-defined. (`src/controllers/nn.js` is currently configured to swallow Mixer + Attitude + Wheels in one shot. The plan is to slot it into Attitude alone, where it belongs.)

3. **Rate hierarchy.** Higher layers run slowly (Nav at ~60 Hz — humans don't move that fast), lower layers run fast (Wheels at 400+ Hz — motors need to). A flat controller has to pick one rate and live with the compromise. A layered one matches each loop's rate to its job.

---

## Walking through the codebase

### Foyer — `main.js`, `src/app.js`

`main.js` boots `App`. `app.js` is the heartbeat — the `tick()` loop reads sensors, runs the controllers, steps the plant, draws the frame. Read `tick()` first; it's the table of contents.

### The body — `src/physics/`

This is the simulated robot itself. Read in this order:

- **`pendulum.js`** — the physics. Inverted pendulum on wheels with yaw, integrated with RK4. The state is `{x, z, vel_cart, pitch, pitch_rate, heading, yaw_rate, ...}`. Take time on the comment block at the top — it shows the equations of motion and the reaction-torque term that makes balance bots non-minimum-phase (lean back to accelerate forward).

- **`sensors.js`** — what the firmware actually sees. IMU with Gaussian noise, a tick-quantized wheel encoder, gyro-Z for yaw rate. Without this layer, controllers see god-mode state and tune too aggressively to survive on real hardware.

- **`motor.js`** — `F = Km · duty − Kv · v`. Two motors lumped into one horizontal force, plus a deadband (the PWM region where friction wins and nothing moves).

### The brain — `src/controllers/`

The cascade described above. (Modules being introduced layer by layer; see `PROJECT.md` for current status.)

For comparison, `ardubalance.js` is the original cascaded controller — kept as a reference baseline. Reading it next to the new layered stack is itself a lesson in how the same job can be expressed cleanly or messily.

### The senses — `src/render/`

- **`plotter.js`** — rolling time-series plot of any signal in `PLOT_SIGNALS`. The most-used debugging tool in the project.
- **`world2d.js`** / **`world3d.js`** — what you see in the canvas.

### The lab — `src/nn/`, `src/controllers/nn.js`, `src/recorder.js`

The neural-net controller and its trainer. The NN learns to mimic the cascade's output from the cascade's inputs. This is *behavior cloning* / *distillation* — a way to take a hand-tuned controller and freeze it into a feed-forward network you could ship on smaller hardware.

`trainer.js` has two modes — random-state sampling (covers the whole input envelope) and recorded trajectories (imitation learning). The random one is what works in practice; the recorded one is what students try first.

### The reference — `ArduBalance/`

Jason's original ArduPilot/ArduBalance firmware code, kept as reference. The whole point of the simulator is to develop intuition that transfers to that real hardware.

---

## How to play

1. **Watch it balance.** Open the page, hit Start. The bot is dropped at a small angle and the controller catches it.
2. **Drive it.** Switch *Pilot Mode* to **FBW** and use the on-screen joystick. Forward stick → forward velocity → forward lean.
3. **Break it.** Open the gain panel. Halve `bal_P`. Watch the pitch oscillate. Halve `wheel_D`. Watch the wheel velocity ring. *Each gain belongs to one layer; halving it teaches you what that layer does.*
4. **Click a target.** Switch *Pilot Mode* to **Auto** and click somewhere in the world. The nav layer activates. Watch the bot turn, then drive, then brake.
5. **Train an NN.** Open the NN panel, hit Train. The MLP is fit to the cascade's behavior. Switch the active controller to NN and see how well it imitates.

---

## Conventions

- **Tabs, not spaces.** Personal preference. The whole project is consistent.
- **Verbose names over Greek letters.** `pitch_rate`, not `θ̇`. `vel_cart`, not `v`. The codebase is read more than it's written, especially by someone learning.
- **Body frame vs world frame.** The single biggest source of bugs in this kind of code. See `CLAUDE.md` for the rule. Short version: pitch controllers see body-frame state; nav sees world-frame state; the mixer is the boundary.
- **Commit early, commit often.** `PROJECT.md` keeps the running log of decisions.

---

If you're standing at the front door for the first time, start with `app.js:tick()`, then read the four controller layers in order. The rest will make sense from there.
