# Validation report — door-arm-001 (RoboPay Tier 1)

Self-audit against the Tier 1 rubric, focused on requirement **R7** (controller
is policy / state-machine driven, not a fixed-joint replay) plus the end-to-end
paid flow that exercises it.

Reproduce:

```bash
cd bridge/door-arm-001
pip install -r requirements.txt
pytest -q
python -m flow.demo --all
```

## 1. End-to-End Paid Flow (summary)

`python -m flow.demo --all` runs the ten steps: discover → 402 (no payment) →
robot untouched → pay (x402 `txHash`) → submit paid action (six-field envelope,
correlated by `actionId`) → publish on `robot/tunnel/action` (Zenoh) → execute
in MuJoCo → result on `robot/tunnel/result` → settle on success only → replay
rejected. The skill executed is **`open_door`** (real MuJoCo rigid-body dynamics,
contact forces read from the solver).

## R7. Controller is policy / state-machine driven (not fixed-joint replay)

Requirement R7: the skill is executed by a policy / planner / feedback
controller, not a fixed joint playback. This PR builds on the arm keyframe
state machine and adds a **handle-tracking IK sequence**:

- The base pick/place path uses the same `KEYFRAMES` + `_run(target, n, grip,
  abort_on_collision=True)` closed-loop gate as the arm family (see below).
- `MuJoCoSimulator.open_door(params)` adds a **turn-handle → pull** sequence:
  an IK solve tracks the handle trajectory *each step*, so the door opens only
  when the handle is actually reached. The motion is feedback-gated by the
  measured handle contact, never a canned animation.
- The door state (latched / ajar / open) is read from the live simulation, and
  `open_door` aborts to a physical `blocked` failure if the handle cannot be
  reached — the same controller, different physics.

No recorded joint clip exists; `replayedAnimation` is asserted `false`.

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
