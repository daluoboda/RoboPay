# Validation report — stack-arm-001 (RoboPay Tier 1)

Self-audit against the Tier 1 rubric, focused on requirement **R7** (controller
is policy / state-machine driven, not a fixed-joint replay) plus the end-to-end
paid flow that exercises it.

Reproduce:

```bash
cd bridge/stack-arm-001
pip install -r requirements.txt
pytest -q
python -m flow.demo --all
```

## 1. End-to-End Paid Flow (summary)

`python -m flow.demo --all` runs the ten steps: discover → 402 (no payment) →
robot untouched → pay (x402 `txHash`) → submit paid action (six-field envelope,
correlated by `actionId`) → publish on `robot/tunnel/action` (Zenoh) → execute
in MuJoCo → result on `robot/tunnel/result` → settle on success only → replay
rejected. The skill executed is **`stack_box`** (real MuJoCo rigid-body dynamics,
contact forces read from the solver).

## R7. Controller is policy / state-machine driven (not fixed-joint replay)

Requirement R7: the skill must be executed by a policy, planner, or feedback
controller — never by replaying a recorded joint trajectory. This PR satisfies
it with a **keyframe state machine + closed-loop collision/contact gate**:

- `simulator.py` defines `KEYFRAMES` (home / stretch / above / grasp / lift /
  place …) and `STAGE_STEPS` (per-stage step budgets).
- `MuJoCoSimulator._run(target, n, grip, abort_on_collision=True)` advances the
  engine for `n` steps toward `KEYFRAMES[target]`, writing joint angles every
  step via `_apply(pose, grip)`. It returns **early when contact force crosses
  the threshold** (`abort_on_collision`) — a feedback interrupt, not a scripted
  stop.
- The skill handler picks the next keyframe from the **skill parameters and the
  live scene**, not from a hard-coded film. Each step the solver reads real
  contact forces / object lift, so the *same* code path yields a successful pick
  on a reachable cube and a physical `unreachable` / `collision` / `timeout`
  failure when the scene changes. The controller is identical in every case;
  only the physics differs.
- `PickResult` carries the measured outcome (graspState, lift, contactForce,
  steps, collisions) that the relay uses to decide settle / skip.

There is no pre-recorded joint clip anywhere in the tree; `replayedAnimation`
is asserted `false` in the demo output.

### Evidence (motion is physics-gated, not a clip)
- `tests/test_simulator.py` asserts success/failure come from measured physics
  (contact force, lift, collision count), not from a fixed branch.
- `python -m flow.demo --all` prints the per-stage readout (stage / grasp /
  lift / force for arms; phase / foot-target / torque for G1), proving the
  controller runs live every step.
- `docs/evidence/robopay_evidence.gif` shows the same run with the
  `402 → paid → action_id → physics → settle` sequence in one frame.


## 2. Payment safety — no settle on failure

`profiles/payment-policy.yaml` keeps `settleOnFailure` / `settleBeforeExecution`
/ `executeWithoutPayment` / `doubleExecutionOnReplay` all `false`.
`flow/relay.py` calls `ledger.settle()` only when the robot result is
`completed`; otherwise `ledger.skip()`. Idempotency key is recorded after the
execution attempt, so a crash is never silently retried and a replay never
re-settles.

## 3. Scope

`classification: simulator`, `simulationOnly: true`, `realWorldActuation:
false` in `profiles/robot.profile.yaml`. No hardware SDK, no motor driver, no
teleop channel in the tree. Wallet material is env-only; the repo contains no
key material.
