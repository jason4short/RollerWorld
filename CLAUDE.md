# Roller — Balance Bot Simulator

Browser-based 2-wheel balance-bot simulator. Vanilla JS ES modules, no build step — open `index.html` in a browser (or serve the dir).

## Layout

- `index.html` — UI shell + inline styles
- `main.js` — boots `App`
- `src/app.js` — top-level wiring
- `src/physics/` — `pendulum.js`, `motor.js`, `sensors.js`
- `src/controllers/` — `pid.js`, `nn.js`, `ardubalance.js`, `nav.js`, `yaw.js`, `pwm-table.js`
- `src/nn/` — `mlp.js`, `trainer.js` (neural-net controller training)
- `src/render/` — 2D + 3D world renderers, plotter
- `src/world/obstacles.js`
- `src/ui.js`, `experiment.js`, `presets.js`, `recorder.js`, `calibration.js`
- `ArduBalance/` — original ArduPilot/ArduBalance reference code Jason wrote

## Conventions

- **Indent with tabs** (user preference for this project).
- Vanilla ES modules, no bundler. Keep imports relative with `.js` extensions.
- No new dependencies without asking.

## Running

Static site — any local server works (`python3 -m http.server`, etc.). Just opening `index.html` via `file://` works for the 2D path but module loading rules may bite; prefer a local server.

## Progress / notes

See `PROJECT.md` for the running log of decisions, milestones, and open threads. Append new notes there (newest on top).

## Frames of reference

Mixing body-frame and world-frame state is the #1 source of subtle bugs here.

- **Body frame**: what physical sensors actually measure. Encoder → `sensors.v`, `sensors.x_body` (integrated body-forward distance). IMU → `sensors.th`, `sensors.w`. Gyro Z → `sensors.yawRate`. Pitch controllers (PID, ArduBalance, NN) MUST use these.
- **World frame**: `plantState.x`, `plantState.z`, `plantState.psi`. Exposed on `sensors` as `x`, `z`, `psi`. Only the nav layer (waypoint targeting) and the renderer should consume these.
- Rule of thumb: if a controller term would break when the bot yaws 90°, it's reading world-frame state where it should be reading body-frame.

## Working style

- Commit early, commit often — small checkpoints with one-line messages beat one giant commit.
