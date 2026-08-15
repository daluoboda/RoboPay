# Dobot CRA / CR3 — paid three-tag inspection (Tier 1, simulator-only)

This vendor-scoped profile executes a non-trivial, bounded CR3 arm task in real MuJoCo and real Webots. The `inspect_three_tags` action chooses the nearest unobserved tag from measured tool pose and computes online damped-least-squares IK; it does not replay a recorded trajectory.

## Model provenance

The vendor model is `cr3_robot.urdf` and its meshes from [Dobot-Arm/DOBOT_6Axis_ROS2_V4](https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4), pinned at `0f67ed938c0cec4ed0808af759ddbb608e573dbe` (MIT).

```
sha256:66df5ef6e26abf365fcf2e9cc2c87469329147283ca8757fa7d645bbe779c7e3
```

MuJoCo compiles that URDF with only a documented bounded actuator overlay. Webots R2025a converts the **same** pinned URDF at run time through `urdf2webots`; its generated PROTO is not vendor supplied.

## Course and state authority

- Three physically rendered markers (amber, cyan, violet) are fixed above a rendered workbench in the CR3 base frame.
- Success requires measured `Link6` / tool-center pose within `0.045 m` of each tag for eight samples, finite state, no unexpected contact, and tool height at least `0.15 m`.
- MuJoCo reads body pose, vendor-joint `qpos`, force, contacts, and dynamics. Webots reads generated CR3 motor position sensors and independently applies pinned-URDF kinematics.

`artifacts/sim2sim_result.json` is generated from the simulator run and ignored by Git: it is not a hand-written claim.

## Run locally

```powershell
cd registry/vendors/dobot/cra/dobot.cra.cr3-mujoco-webots-inspection.v1
py -3 -m pip install -r bridge/requirements.txt
$env:PYTHONPATH = "$PWD/bridge"
./run-visual-mujoco.ps1
./run-visual-webots.ps1
```

For a non-interactive real Sim-to-Sim validation:

```powershell
$env:WEBOTS_EXE = "$env:LOCALAPPDATA\Programs\Webots\msys64\mingw64\bin\webots.exe"
py -3 bridge/run_sim2sim_validation.py
```

The validator invokes real MuJoCo and a real Webots process, saves the measured outcome, and exits non-zero if either engine fails the canonical course.

## OBS-ready live Base Sepolia proof

`run-live-base-sepolia-visual.ps1` opens the real MuJoCo execution scene,
mirrors the real Tunnel logs, prints discovery, the unpaid `402`, the x402
payment requirement, the `202` accepted response, and the terminal settlement
status. On success it can open the transaction in BaseScan. The small local
HTTP/WebSocket proxy is only a visible transport stand-in for the hosted
gateway; it does not emulate the facilitator, payment, Zenoh, or simulator.
It subscribes to the bridge-ready event before starting the bridge and sends no
paid action until that event arrives. Its WebSocket reader reassembles
continuation frames, including a fragmented first response.

The launcher expects the **same catalog-aware hardened Tunnel contract used by
the Spot Tier 1 base**. Point it at that Tunnel binary and supply deployment
credentials through the current process or a secret manager; neither is
written into the repository:

```powershell
$env:TUNNEL_BIN = 'C:\path\to\hardened\tunnel'
$env:ROBO_PAYEE_ADDRESS = '<configured-payee>' # ROBOT_PAYEE_ADDRESS is accepted as a local alias.
# Set BASE_SEPOLIA_PRIVATE_KEY in this process via your secret manager.
./run-live-base-sepolia-visual.ps1 -OpenBaseScan
```

For an entirely non-spending connectivity check (no signature, no settlement,
no chain transaction), use:

```powershell
./run-live-base-sepolia-visual.ps1 -DryRun
```

For the final operator recording, the launcher prints the exact source commit
before any request, pauses after the paid viewer opens so the terminal and
MuJoCo window can be arranged side by side, holds the initial pose and each of
the three visible tag confirmations, then holds the final pose for three
seconds and closes the viewer automatically. The correlated result and Base
Sepolia settlement therefore appear without requiring the operator to close
the simulator manually.

## Tunnel, payment, and identity boundary

The profile bridge contains no private key, payee address, x402 verifier, or settlement code. It accepts only a **private post-verification Tunnel ActionEvent** carrying `action_id`, `robot_id`, `skill_id`, canonical `params_hash`, `idempotency_key`, payment payload, and payment requirements. It fails closed for absent/malformed evidence, wrong robot, missing/unknown skill, parameter drift, legacy uncorrelated events, and replay.

Before execution, SQLite persists both the idempotency key and a fingerprint of the verified payment payload at `artifacts/state/dobot_cr3_replay.sqlite3`. Reusing a payment or key after a restart cannot produce a second action. Tunnel owns execution-gated settlement and may settle only after it sees the same action ID return `status: success` on `robot/tunnel/result`.

The old base Tunnel format containing only `{payload:{action,...}}` is intentionally rejected because it cannot prove payment/action correlation. A live Base Sepolia run requires a hardened Tunnel that returns `402` before `PostAction` for absent/invalid verification, publishes the enriched event only after verification, persists payment-bound idempotency, and settles only after terminal success. Configure `ROBOT_PAYEE_ADDRESS` in that deployment—not in this profile.

Robot WebSocket identity-to-payee binding remains an upstream shared Tunnel/Gateway protocol responsibility; this profile does not invent a local EIP signing scheme.

## Registry contract and evidence

`robot.profile.yaml`, `skills.yaml`, `skill-catalog.json`, `payment-policy.yaml`, and `execution-mapping.yaml` declare the same two skills and `$0.001` Base Sepolia USDC policy. `stop` is explicit and cannot fall through to inspection.

`docs/evidence/evidence-manifest.yaml` separates real simulator evidence from Base Sepolia material that must be captured later against the required hardened Tunnel. No mocked facilitator or simulator is presented as live settlement evidence.
