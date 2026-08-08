# Task traceability

This document ties every success-criterion item of the Tier 1 simulator track
to the file or evidence that proves it.

| # | Criterion | Evidence |
|---|----------|----------|
| 1 | Real 402 payment challenge | `docs/evidence/terminal/laok-stack-arm-402.png`, `bridge/laok_stack_arm_001_zenoh_bridge.py` (`do_GET`/`do_POST` 402 path) |
| 2 | Real x402 verify + settle on Base Sepolia | `x402-evidence.json` (3 hashes), first tx `0xad4c5669e1a3351a69256e5c5d507efc939ed9e7cfe0ac8b0be90603a796d1d6` |
| 3 | On-chain settlement confirmed | basescan link above; payer `0xA0723A2d…3d13` 19.5 → 19.2 USDC across the three settlements; re-verified by `verify_settlement.py` |
| 4 | Simulator executes the skill | `docs/evidence/terminal/laok-stack-arm-mujoco.png`, `bridge/simulator.py` (`MuJoCoSimulator.pick_and_stack`) |
| 5 | Object physically picked and stacked | metrics: `objectLifted 0.0502 m`, `a_z 0.0752` > `b_z 0.0250`, `stackStable true`, `stackOffsetXY 0.0121 m`, `peakForce 6.5486 N`, `graspState stacked` |
| 6 | Async action/result over Zenoh | `docs/evidence/terminal/laok-stack-arm-async.png`, `bridge/laok_stack_arm_001_zenoh_bridge.py` (`ZenohTransport`) |
| 7 | actionId correlation | result envelope carries `actionId`; `execution-mapping.yaml` `correlationField: actionId` |
| 8 | Payment gates execution | `bridge/x402_client.py` (`PaymentGate.check`) |
| 9 | Idempotency / no double execution | `bridge/laok_stack_arm_001_zenoh_bridge.py` (`IdempotencyStore`), `tests/test_bridge.py` |
| 10 | Failure never settles | `bridge/laok_stack_arm_001_zenoh_bridge.py` (`_execute` settles only on success) |
| 11 | Reproducible by reviewer | `README.md`, `bridge/requirements.txt`, `tests/requirements.txt` |

## Privacy handling

- Wallet addresses and x402 signatures are withheld from public PNGs.
- Only the on-chain transaction hash is disclosed as the cross-reference.
- `ROBOT_PAYEE_ADDRESS` / `ROBOT_ID` / `ROBOT_PRIVATE_KEY` are read from
  environment variables at runtime and never committed.
