# Roller — Project Notes

Running log of decisions, milestones, and open threads. Newest on top. Keep entries short — a sentence or two. Link commits when relevant.

## Open threads

-

## Log

- 2026-04-25 — NN damping fix. The 4-input velocity-tracking NN rocked badly: only `pitch_rate` provided damping, and the trainer left ArduBalance's inner-loop D-state at zero so wheel_D was effectively absent from training targets. Added `vel_cart_prev` as a 5th input (NN learns its own d/dt) and the trainer now seeds `ab.last_vmeas` and `ab.speed_d_lpf` from the implied acceleration. Also renamed for readability: `v_target` → `vel_cart_target`, NN locals use `vel_cart`.
- 2026-04-25 — NN architecture: switched to a velocity-tracking NN. Inputs were `[pitch, pitch_rate, x, v, target_angle, dv]`; `x` was an unbounded body-frame integrator with no matching target-position input, so the only thing it could teach was "return to x=0," fighting nav. New inputs: `[pitch, pitch_rate, v, v_target]`. The teacher now combines nav's `tilt = clamp(Kvel*(v_target - v), tiltLimit)` step with the ArduBalance cascade, so the NN swallows both. App wires `nn.v_target = nav.v_desired_last`. Recorder format also changed (drops `x` and `target_angle`, adds `v_target`).
- 2026-04-24 — Fix: cart velocity spike on sharp nav input. Step in `v_desired` (clicked target, stick yank, arrival) propagated through `Kvel` into a step on `target_angle`, slamming ArduBalance's `vel_command` (= `bal_P * angle_err`) and saturating the inner speed loop. Now `v_desired` is rate-limited to `a_max` in both `update()` and `updateFbw()`.
- 2026-04-24 — Fix: PID/NN tipped over after a yaw turn while moving. `Kx * state.x` was using world-frame x, so post-yaw the body's forward axis no longer aligned with world-x and the term demanded runaway lean. Now `sensors.x_body` (encoder-integrated body-frame distance) feeds the position term; nav keeps world `x/z` for waypoint math.
- 2026-04-24 — Added FBW pilot mode + 2D on-screen joystick (`src/ui/joystick.js`, `nav.updateFbw`, Pilot panel in index.html). Stick → `v_desired`; centering relies on `Kvel*(0 - v_lpf)` for active braking. Mirrors how ArduPilot FBW was used to bring up the nav loop on real hardware before GPS scripting was ready. Raw-tilt (arrow keys) kept as a debug fallback via the Pilot Mode select.
- 2026-04-24 — Started PROJECT.md. Plan: commit more often (small checkpoints) so history isn't trapped in one initial commit.
