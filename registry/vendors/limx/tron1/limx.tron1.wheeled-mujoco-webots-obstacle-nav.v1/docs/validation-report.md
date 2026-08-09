# LimX TRON 1 validation report

## Scope

Tier 1 simulator-only integration of the official `WF_TRON1A` model for the
fixed `navigate_obstacle_course` and `stop` skills. Payment price is `$0.001`
USDC on Base Sepolia.

## Locally revalidated results

- MuJoCo 3.3.0: 7/7 waypoints, three obstacles detected, goal reached, no
  contact, minimum clearance `0.0775 m`; official LimX policy/encoder ONNX.
- Webots R2025a: 7/7 waypoints, three obstacles detected, goal reached, no
  contact; official URDF and STL geometry converted by `urdf2webots`.
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

Webots uses its Supervisor-measured root pose as terminal authority while a
bounded profile controller drives the official converted robot through the
canonical route. Dynamic equivalence with the LimX reinforcement-learning
controller is asserted only for MuJoCo. Identity signing between the shared
Gateway and robot WebSocket remains upstream, as directed by maintainer review.
