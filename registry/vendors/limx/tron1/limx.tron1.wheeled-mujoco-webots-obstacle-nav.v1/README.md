# LimX TRON 1 Tier 1 — paid MuJoCo + Webots obstacle navigation

This simulator-only profile runs a fixed obstacle course with the official LimX
`WF_TRON1A` wheeled-foot model. MuJoCo loads LimX's MJCF and official Isaac Gym
ONNX policy directly. Webots converts the matching official URDF and nine STL
meshes; it does not use a substitute robot.

## Pinned upstream identity

- [`limxdynamics/robot-description`](https://github.com/limxdynamics/robot-description), commit `469df8dbb56802b127ca8e2c5df23360c6c5488d`, Apache-2.0
- [`limxdynamics/tron1-rl-deploy-python`](https://github.com/limxdynamics/tron1-rl-deploy-python), commit `08b1f113444b85f2f24505a1a7a01a462cf77068`, Apache-2.0
- Variant: `WF_TRON1A`; URDF, MJCF, mesh and ONNX hashes are pinned in `robot.profile.yaml`

## What the action does

`navigate_obstacle_course` follows seven state-driven waypoints around three
physical obstacles and terminates only from measured simulator state. MuJoCo
uses the vendor policy to turn current base/joint observations and velocity
commands into torques. Webots runs the same bounded route against the converted
vendor geometry and reports measured root translation/rotation. Success means
all waypoints and the goal were reached, all obstacles were detected and no
obstacle contact occurred. `stop` is a separate zero-velocity terminal path.

No caller-controlled motion parameters are accepted.

The fixed planner bounds the episode to 70 seconds, linear velocity to
`0.65 m/s` and yaw rate to `0.5 rad/s`. Those limits are profile-owned and
cannot be raised by a paid request.

## Reproduce locally

```powershell
cd registry/vendors/limx/tron1/limx.tron1.wheeled-mujoco-webots-obstacle-nav.v1
py -3 -m pip install -r bridge/requirements-dev.txt
./run-tests.ps1
./run-visual-mujoco.ps1
./run-visual-webots.ps1
```

Set `WEBOTS_EXE` if Webots R2025a is not on `PATH`. The generated PROTO records
its vendor provenance and contains only repository-relative mesh paths.

### Zenoh session and message contract

For a standalone development session, start a Zenoh 1.9 router and point both
Tunnel and bridge at it:

```powershell
zenohd -l tcp/127.0.0.1:7447
$env:ZENOH_ENDPOINT = 'tcp/127.0.0.1:7447'
$env:PYTHONPATH = "$PWD/bridge"
py -3 -m limx_tron1_sim.bridge
```

The bridge subscribes to `robot/tunnel/action`, publishes terminal results to
`robot/tunnel/result`, metrics to `robot/limx_tron1/metrics`, and readiness to
`robot/limx_tron1/ready`. The bridge is the robot-control process: it validates
the correlated event, reserves it durably and invokes the real MuJoCo runtime.

The private input event preserves this tuple:

```json
{
  "action_id": "act-001",
  "robot_id": "limx-tron1-wf-sim-01",
  "skill_id": "navigate_obstacle_course",
  "idempotency_key": "act-001",
  "params_hash": "sha256:...",
  "payload": {"action": "navigate_obstacle_course", "params": {}},
  "transaction_details": {"payment_payload": {}, "payment_requirements": {}}
}
```

The result repeats `action_id`, `robot_id`, `skill_id`, `idempotency_key` and
`params_hash`, adds `status`, and places the measured simulator output under
`result`.

## Payment, authorization and replay contract

- Base Sepolia (`eip155:84532`), USDC, `$0.001` for either registered skill.
- The real Go Tunnel verifies x402 before `PostAction`. A nil or `isValid:false`
  facilitator response returns `402` and publishes zero ActionEvents.
- The Tunnel settles only after the correlated terminal result reports success.
  Failure, timeout and invalid payment remain unsettled.
- The same durable JSON replay implementation accepted in PR 58 protects the
  public Tunnel boundary. The profile additionally persists its private Zenoh
  execution reservation in SQLite, bound to action ID, idempotency key and
  payment fingerprint.
- Missing/unknown action, mismatched skill, foreign robot, changed parameters,
  uncorrelated Zenoh data and repeated payment evidence all fail closed.
- Robot WebSocket identity-to-payee signing remains an upstream shared
  Tunnel/Gateway dependency; this profile does not invent a local EIP protocol.

Runtime-only configuration:

| Variable | Purpose |
| --- | --- |
| `ROBOT_ID` | Must equal `limx-tron1-wf-sim-01` |
| `ROBO_PAYEE_ADDRESS` | Non-zero testnet payee wallet |
| `BASE_SEPOLIA_PRIVATE_KEY` | Test payer, visual runner only; never passed to the bridge |
| `SKILL_CATALOG_PATH` | Absolute path to `skill-catalog.json` |
| `ALLOWED_ACTIONS` | `navigate_obstacle_course,stop` |
| `ZENOH_CONFIG` or `ZENOH_ENDPOINT` | Explicit private Zenoh session |
| `TUNNEL_BIN` | Real catalog-aware Go Tunnel binary |

Never commit, print or record a private key. Use an untracked process
environment locally and GitHub Actions secrets in CI.

## CI and evidence

`.github/workflows/limx-tron1-tier1.yml` runs the same hardened Tunnel used by
PR 58, the adversarial `isValid:false` regression, failure/timeout
non-settlement, durable replay, real Zenoh, official-policy MuJoCo, real Webots
Sim-to-Sim, and a trusted push/workflow-dispatch Base Sepolia settlement job.
The live runner waits for an explicit bridge-ready event, sends the paid action
once after a clean start, assembles WebSocket continuation frames and uploads
the receipt/result JSON.

Expected success is HTTP `202`, followed by a status document with
`state: succeeded`, `settled: true`, the same `action_id`, measured course
metrics and a Base Sepolia transaction hash. Unknown skills/parameters return
a fail-closed error before Zenoh; injected execution failure returns terminal
`failed` with `settled: false`; a repeated request returns HTTP `409`.

### Troubleshooting

- **Bridge never ready:** verify the router is running and both processes use
  the same explicit `ZENOH_CONFIG`/endpoint.
- **Webots produces no JSON:** use R2025a and install `libsndio7.0` (or the
  distribution's compatible `libsndio` package).
- **Tunnel returns 503:** configure both the catalog/allowlist and a writable
  durable idempotency store; corrupt state deliberately fails closed.
- **Payment returns 402:** confirm Base Sepolia, payee, USDC asset and payer
  funds. Do not bypass verification.
- **Model hash test fails:** restore the pinned vendor asset; do not regenerate
  or hand-edit the official URDF/MJCF/ONNX files.

See [the validation report](docs/validation-report.md) and
[evidence manifest](docs/evidence/evidence-manifest.yaml).
