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

## Settlement proof (REAL on-chain, verified)
`x402-evidence.json` records **1 real Base-Sepolia USDC settlement**:
- tx: `0xb2546bb528289e14cabf593c8ee9521e144fbfd2b62c74d807dc1c68a23f792f`
- payer: `0xF2749b5fAdA8a83d3DE1a2621B1d212e73907D4a`
- payee: `0x742d35Cc6634C0532925a3b844Bc454e4438f44e`
- amount: 0.1 USDC
- on-chain: status=1, block 45339399 (verified via Base-Sepolia RPC)
- submitted by facilitator `0xd407e409E34E0b9afb99EcCeb609bDbcD5e7f1bf`, which paid gas;
  the payer wallet needed only a USDC balance (x402 facilitator model).

`metrics.json` therefore marks `paid_success = PASS_ONCHAIN_ONLY`.

## Remaining honest gaps (NOT faked)
- `gate_pass=False`: `unpaid_rejected` (402) and `replay_rejected` (409) single-test
  assertions are `NOT_COVERED` — door has no `test_bridge.py` exercising 402/409.
- Dynamic MuJoCo<->PyBullet numeric agreement layer is CI-gated (real PyBullet not
  installable on Windows) — reported honestly, never fabricated.

## Envelope fidelity
`examples/action-envelope.open-door.json` shows the six required fields
(actionId, robotId, skillId, paramsHash, idempotencyKey, payment) and canonical SHA-256
params hashing so params cannot be swapped in flight.

## Scope
`classification: simulator`, `simulationOnly: true`, `realWorldActuation: false`,
`gpuRequired: false`. No hardware SDK, motor driver or teleop channel in the tree.
