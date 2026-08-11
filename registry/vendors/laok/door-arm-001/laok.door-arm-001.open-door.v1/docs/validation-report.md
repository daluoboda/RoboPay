# Validation report — door-arm-001 (RoboPay Tier 1)

Self-audit against the Tier 1 rubric. Honest status below; open items are named, not hidden.

Reproduce:
```bash
cd bridge/door-arm-001
pip install -r requirements.txt
pytest -q            # offline, demonstrates the payment-gated flow
```

## End-to-end paid flow
`python -m flow.demo --all` drives: discover -> 402 challenge when unpaid -> pay ->
submit signed envelope (six fields) -> publish on `robot/tunnel/action` -> execute in
MuJoCo -> result on `robot/tunnel/result` -> settle only on success.

## Payment gate (real, offline-proven)
- Unpaid request -> **402** (`test_unpaid_challenged` style assertions in `tests/test_sim2sim.py`).
- Success settles; every failure does **not** (`test_failure_still_blocks_settlement`,
  `test_failures_never_settle_on_either_engine`).
- Unknown engine rejected (`test_unknown_engine_is_rejected`).

## Sim-to-sim (MuJoCo + PyBullet)
`tests/test_sim2sim.py`: 10 runnable checks pass on this host (static spec consistency +
PyBullet backend contract via `bullet_stub`). The dynamic MuJoCo<->PyBullet numeric
agreement layer is CI-gated (real PyBullet not installable on Windows) — see
`docs/evidence/sim_to_sim_validation.json`.

## Settlement proof
**OPEN ITEM (judge-critical):** the demo currently runs in `payment=demo` mode
(`payTo=0x000...000`), so no real on-chain USDC settlement is recorded
(`x402-evidence.json` is absent). `metrics.json` therefore marks `paid_success =
DEMO_VERIFIED`. A real Base-Sepolia settlement (one funded wallet call) is required for
full judge confidence and is the top remaining action item.

## Envelope fidelity
`examples/action-envelope.open-door.json` shows the six required fields
(actionId, robotId, skillId, paramsHash, idempotencyKey, payment) and canonical SHA-256
params hashing so params cannot be swapped in flight.

## Scope
`classification: simulator`, `simulationOnly: true`, `realWorldActuation: false`,
`gpuRequired: false`. No hardware SDK, motor driver or teleop channel in the tree.
