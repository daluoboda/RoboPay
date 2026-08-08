# Task traceability

This document ties every success-criterion item of the Tier 1 simulator track
to the file or evidence that proves it.

| # | Criterion | Evidence |
|---|----------|----------|
| 1 | Real 402 payment challenge | `docs/evidence/terminal/laok-sort-arm-402.png`, `bridge/laok_sort_arm_001_zenoh_bridge.py` (`do_GET`/`do_POST` 402 path) |
| 2 | Real x402 verify + settle on Base Sepolia | `x402-evidence.json` (3 hashes), first tx `0xea4f54c97c26762915615023cffeb942f858c51963ef1e2117de4de28b8f16d0` |
| 3 | On-chain settlement confirmed | basescan link above; payer `0xA0723A2d…3d13` 19.2 → 18.9 USDC across the three settlements; re-verified by `verify_settlement.py` |
| 4 | Simulator executes the skill | `docs/evidence/terminal/laok-sort-arm-mujoco.png`, `bridge/simulator.py` (`MuJoCoSimulator.pick_and_sort`) |
| 5 | Object physically picked, carried and sorted | metrics: `peakLift 0.1354 m` (carried clear of the table), `routed true`, `accuracy 0.0265 m` into bin `A`, `objectLifted 0.0014 m` (rests back down in the bin), `peakForce 6.5486 N`, `graspState released` |
| 6 | Async action/result over Zenoh | `docs/evidence/terminal/laok-sort-arm-async.png`, `bridge/laok_sort_arm_001_zenoh_bridge.py` (`ZenohTransport`) |
| 7 | actionId correlation | result envelope carries `actionId`; `execution-mapping.yaml` `correlationField: actionId` |
| 8 | Payment gates execution | `bridge/x402_client.py` (`PaymentGate.check`) |
| 9 | Idempotency / no double execution | `bridge/laok_sort_arm_001_zenoh_bridge.py` (`IdempotencyStore`), `tests/test_bridge.py` |
| 10 | Failure never settles | `bridge/laok_sort_arm_001_zenoh_bridge.py` (`_execute` settles only on success) |
| 11 | Reproducible by reviewer | `README.md`, `bridge/requirements.txt`, `tests/requirements.txt` |

## Privacy handling

- Wallet addresses and x402 signatures are withheld from public PNGs.
- Only the on-chain transaction hash is disclosed as the cross-reference.
- `ROBOT_PAYEE_ADDRESS` / `ROBOT_ID` / `ROBOT_PRIVATE_KEY` are read from
  environment variables at runtime and never committed.
