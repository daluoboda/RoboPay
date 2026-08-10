# LimX TRON 1 validation report

## Scope

Tier 1 simulator-only integration of the official `WF_TRON1A` model for the
fixed `navigate_obstacle_course` and `stop` skills. Payment price is `$0.001`
USDC on Base Sepolia.

## Requirement mapping

- **Simulator-only:** the submission invokes no physical robot API.
- **Approved simulators:** real MuJoCo and real Webots processes are executed.
- **No animation/trajectory replay:** commands are recomputed online by the
  shared route planner from measured simulator pose; the fixed waypoints are
  task goals, not timestamped frames.
- **No built-in demo motion:** both runtimes start from the paid registered
  skill and the profile-owned planner/controller path.
- **Measured task evidence:** waypoint completion, obstacle detection,
  clearance, collision, goal distance, pose, speed and tilt are emitted from
  simulator state.
- **Sim-to-Sim:** both simulators complete the same model/task/terminal
  contract. MuJoCo is actuator-level; Webots is explicitly task-level.

## Locally revalidated results (2026-08-10)

- MuJoCo 3.3.0: 10/10 waypoints, three obstacles detected, goal reached, no
  collision, `0.2016 m` minimum clearance and `5.3617 m` measured final x.
  The official LimX policy/encoder ONNX drives the pinned vendor MJCF.
- Webots R2025a: 10/10 waypoints, three obstacles detected, goal reached, no
  obstacle contact, `0.2395 m` minimum clearance and `5.3646 m` measured base
  displacement in `23.536 s`. A bounded task-level Supervisor adapter maps the
  online planner to chassis velocity on the converted dynamic model. Measured
  pose, velocity, orientation and contacts are terminal authority. It performs
  zero translation/rotation writes and uses no prerecorded trajectory.
- Sim-to-Sim contract: same model variant, course, waypoint count, obstacles,
  success boundary and measured terminal goal state.
- Real Zenoh: one valid correlated event executes once; replay publishes a
  terminal rejection and does not execute a second time.
- Durable restart: repeated action ID/idempotency key and repeated payment
  fingerprint are rejected after opening a fresh store instance.
- Real Go Tunnel/x402 middleware with a recording facilitator: a paid-shaped
  tampered signature receiving `isValid:false` returns HTTP 402, publishes zero
  ActionEvents, produces zero simulator state changes and makes zero settlement
  calls. Injected simulator failure and timeout remain unsettled; replay causes
  no second dispatch.

## Commands

```powershell
$env:PYTHONPATH = "$PWD/bridge"
py -3 -m pytest -q tests
py -3 bridge/run_mujoco_obstacle_course.py
py -3 bridge/run_sim2sim_validation.py
```

The x402 integration test requires the real Linux Tunnel built by `make build`
and runs in WSL/Linux with `TUNNEL_BIN=.../bin/tunnel` and
`LD_LIBRARY_PATH=.../.zenoh-c/lib`.

## Deliberate boundaries

MuJoCo uses the pinned LimX reinforcement-learning controller and joint torque
mapping. Webots deliberately validates at task level: it uses bounded root
velocity commands because the Isaac Gym policy is not a Webots controller.
Both execute the same online planner and measured terminal contract. Webots
does not write translation/rotation, reset physics or replay a trajectory, but
this evidence must not be represented as actuator-level equivalence. Identity
signing between the shared Gateway and robot WebSocket remains upstream, as
directed by maintainer review.
