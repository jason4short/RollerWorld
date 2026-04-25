# Roller — Project Notes

Running log of decisions, milestones, and open threads. Newest on top. Keep entries short — a sentence or two. Link commits when relevant.

## Open threads

-

## Log

- 2026-04-24 — Fix: cart velocity spike on sharp nav input. Step in `v_desired` (clicked target, stick yank, arrival) propagated through `Kvel` into a step on `target_angle`, slamming ArduBalance's `vel_command` (= `bal_P * angle_err`) and saturating the inner speed loop. Now `v_desired` is rate-limited to `a_max` in both `update()` and `updateFbw()`.
- 2026-04-24 — Fix: PID/NN tipped over after a yaw turn while moving. `Kx * state.x` was using world-frame x, so post-yaw the body's forward axis no longer aligned with world-x and the term demanded runaway lean. Now `sensors.x_body` (encoder-integrated body-frame distance) feeds the position term; nav keeps world `x/z` for waypoint math.
- 2026-04-24 — Added FBW pilot mode + 2D on-screen joystick (`src/ui/joystick.js`, `nav.updateFbw`, Pilot panel in index.html). Stick → `v_desired`; centering relies on `Kvel*(0 - v_lpf)` for active braking. Mirrors how ArduPilot FBW was used to bring up the nav loop on real hardware before GPS scripting was ready. Raw-tilt (arrow keys) kept as a debug fallback via the Pilot Mode select.
- 2026-04-24 — Started PROJECT.md. Plan: commit more often (small checkpoints) so history isn't trapped in one initial commit.
