# RoboPay — LimX TRON 1 Tier 1

This branch adds a simulator-only, payment-gated LimX TRON 1 obstacle-navigation
profile. It runs the official LimX `WF_TRON1A` model in real MuJoCo and Webots,
uses Zenoh for the local action/result boundary and settles x402 payment only
after a correlated successful simulator result.

Start here:

- [TRON 1 profile runbook](registry/vendors/limx/tron1/limx.tron1.wheeled-mujoco-webots-obstacle-nav.v1/README.md)
- [Validation report](registry/vendors/limx/tron1/limx.tron1.wheeled-mujoco-webots-obstacle-nav.v1/docs/validation-report.md)
- [Evidence manifest](registry/vendors/limx/tron1/limx.tron1.wheeled-mujoco-webots-obstacle-nav.v1/docs/evidence/evidence-manifest.yaml)
- [Tier 1 CI](.github/workflows/limx-tron1-tier1.yml)

## Demonstrated action

`navigate_obstacle_course` drives the wheeled-foot TRON 1 around three physical
barriers through seven state-driven waypoints. MuJoCo uses the official LimX
Isaac Gym policy and encoder ONNX files. Webots uses a generated PROTO from the
matching official URDF and STL meshes. Both report measured pose, waypoint,
obstacle, clearance, collision and goal metrics. `stop` retains zero velocity
and cannot fall through to navigation.

## Paid action flow

1. The payer discovers `limx-tron1-wf-sim-01` and its `$0.001` Base Sepolia
   USDC skills.
2. An unpaid action returns `402` and x402 requirements.
3. The real Go Tunnel rejects nil/invalid facilitator verification before
   `PostAction`; only a verified action is published to `robot/tunnel/action`.
4. The profile bridge validates the full correlation tuple and executes the
   official MuJoCo model/policy.
5. It publishes a terminal measured result on `robot/tunnel/result`.
6. The Tunnel settles only terminal success and exposes the correlated status
   and receipt. Failure, timeout, mismatch and replay remain unsettled.

The public response is immediate HTTP `202 accepted/pending`; execution is
asynchronous and the same `action_id` is used at the status endpoint.

## Security properties copied from approved PR 58

- Explicit registered-skill catalog and allowlist; missing configuration fails
  closed.
- `isValid:false` returns HTTP 402 with zero ActionEvents, zero actuation and
  zero settlement.
- Durable, payment-bound idempotency survives Tunnel restart.
- Unknown action/skill/parameter and foreign robot ID are rejected.
- Failure, timeout and correlation mismatch never settle.
- The positive E2E assembles WebSocket continuation frames, waits for bridge
  readiness and proves the first paid action without a warm-up action.
- x402 Python SDK `2.16.0` and `requests==2.33.0` are pinned.
- Secrets are accepted only from the runtime environment/GitHub Secrets and
  are never passed to the simulator bridge.

Robot identity-to-payee signing is explicitly tracked as a shared upstream
Fabric Tunnel/Gateway dependency, consistent with maintainer guidance; this
robot profile does not invent an incompatible local EIP handshake.

## Quick local validation

```powershell
$profile = 'registry/vendors/limx/tron1/limx.tron1.wheeled-mujoco-webots-obstacle-nav.v1'
py -3 -m pip install -r "$profile/bridge/requirements-dev.txt"
& "$profile/run-tests.ps1"
& "$profile/run-visual-mujoco.ps1"
& "$profile/run-visual-webots.ps1"
```

The Linux/CI authorization suite builds and runs the real Tunnel:

```bash
make build
make test
export PYTHONPATH=registry/vendors/limx/tron1/limx.tron1.wheeled-mujoco-webots-obstacle-nav.v1/bridge
export TUNNEL_BIN="$PWD/bin/tunnel"
export LD_LIBRARY_PATH="$PWD/.zenoh-c/lib"
python3 -m pytest -q registry/vendors/limx/tron1/limx.tron1.wheeled-mujoco-webots-obstacle-nav.v1/tests/test_x402_invalid_payment_gate.py
```

The checked-in `tunnel/config.json` is intentionally inert. Configure a stable
robot ID, non-zero payee, catalog, allowlist and idempotency-store path only in
an untracked environment or deployment secret manager.
