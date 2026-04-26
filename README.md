# Roller

Browser-based 2-wheel balance-bot simulator. Vanilla ES modules, no build step — open `index.html` via a local server and the bot runs.

Roller is the playground version of *ArduBalance* — the firmware that won the SparkFun Autonomous Vehicle Competition on a 2-wheel balance bot, a famously hard platform for that race. Same cascade architecture, same physics intuitions, but you can swap controllers from a dropdown, watch a neural net train layer-by-layer, drive through a procedural park world, and see why each piece of a real robot stack exists by turning it off and watching things fail.

It's built as a teaching tool, not a product. Every layer exists to make a specific lesson visible.

## Quick start

```
python3 -m http.server
# then open http://localhost:8000
```

(Loading via `file://` mostly works, but ES module rules occasionally bite — a local server is safer.)

## What's in the world

- **Procedural park** — rolling hills (3-octave heightfield), hand-authored road network with three forks and two dead-end spurs, autumn-color tree clusters, water in valleys, atmospheric haze.
- **Four pilot modes** — Angle (raw tilt via arrow keys), FBW (fly-by-wire on-screen joystick), Auto (shift-click to drop waypoints, A\* path-plan around walls), and Road (reactive pilot using a forward-arc lidar or color sensor; toggle between weighted-sum and Follow-the-Gap algorithms).
- **Three pitch controllers** — hand-written PD with auto-trim, a feedforward NN trained to imitate the rule, and an RNN that maintains hidden state across ticks. Selectable from the cascade Attitude panel.
- **Sensors** — IMU with optional bias drift, encoder with optional dropout, 24-ray lidar, color sensor sampling the painted road texture, accumulating occupancy grid.
- **Slope physics** — hills are a force disturbance the motor has to overcome; the controller doesn't know they exist (because on a real bot, it shouldn't have to).

## The cascade

The bot runs a 4-layer controller stack at three rates:

```
	Nav      ( 60 Hz)   waypoint or pilot stick  →  velocity command
	Mixer    (100 Hz)   velocity error           →  pitch (tilt) target
	Attitude (100 Hz)   tilt target              →  force + yaw torque
	Wheels   (400 Hz)   force/torque             →  per-wheel PWM
```

Any layer can be swapped between rule-based and learned implementations from the UI. The legacy whole-stack ArduBalance and a single-NN baseline live alongside for comparison. See `TOUR.md` for the full walkthrough.

## Files

- `index.html`, `main.js`, `src/app.js` — UI shell + boot + top-level wiring
- `src/physics/` — pendulum, motor, sensors, lidar, road sensor
- `src/controllers/` — PID, ArduBalance, the cascade stack, road controller (WSum + FTG), nav, yaw
- `src/nn/` — typed-array MLP (fast), RNN on top of micrograd (autodiff for training + numbers-only inference), trainer
- `src/world/` — road network, road canvas, terrain, obstacles, occupancy grid
- `src/render/` — three.js world renderer, plotter

## Other docs

- `TOUR.md` — front-door walkthrough for human visitors
- `RNN_NOTES.md` — concept reference for RNNs, BPTT, and the frozen-vs-online-learning question
- `PROJECT.md` — running log of decisions and milestones (newest on top)
- `CLAUDE.md` — guidance for AI agents working on the codebase
- `ArduBalance/` — the original firmware reference

## Philosophy

Roller is a **palace, not a factory**. Comments explain the *lesson*, not the syntax. If you find a `// magic` note, it's working notes from someone figuring it out, not a TODO. Clarity outranks brevity.
