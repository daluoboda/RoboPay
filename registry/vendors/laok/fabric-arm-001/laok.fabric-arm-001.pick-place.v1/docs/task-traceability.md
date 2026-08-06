# Task traceability

This document ties every success-criterion item of the Tier 1 simulator track
to the file or evidence that proves it.

| # | Criterion | Evidence |
|---|----------|----------|
| 1 | Real 402 payment challenge | `docs/evidence/terminal/laok-fabric-arm-402.png`, `bridge/laok_fabric_arm_zenoh_bridge.py` (`do_GET`/`do_POST` 402 path) |
| 2 | Real x402 verify + settle on Base Sepolia | `docs/evidence/terminal/laok-fabric-arm-settle.png`, tx `0xcf0222171e83fd6c0d3981cf202de984c1dd0cb10f06d81eef76da779a5fb6d2` |
| 3 | On-chain settlement confirmed | basescan link above; payer 19.8 → 19.7 USDC |
| 4 | Simulator executes the skill | `docs/evidence/terminal/laok-fabric-arm-mujoco.png`, `bridge/simulator.py` (`MuJoCoSimulator.pick_object`) |
| 5 | Object physically lifted | metrics: `objectLifted 0.1313 m`, `graspState attached` |
| 6 | Async action/result over Zenoh | `docs/evidence/terminal/laok-fabric-arm-async.png`, `bridge/laok_fabric_arm_zenoh_bridge.py` (`ZenohTransport`) |
| 7 | actionId correlation | result envelope carries `actionId`; `execution-mapping.yaml` `correlationField: actionId` |
| 8 | Payment gates execution | `bridge/x402_client.py` (`PaymentGate.check`) |
| 9 | Idempotency / no double execution | `bridge/laok_fabric_arm_zenoh_bridge.py` (`IdempotencyStore`), `tests/test_bridge.py` |
| 10 | Failure never settles | `bridge/laok_fabric_arm_zenoh_bridge.py` (`_execute` settles only on success) |
| 11 | Reproducible by reviewer | `README.md`, `bridge/requirements.txt`, `tests/requirements.txt` |

## Privacy handling

- Wallet addresses and x402 signatures are withheld from public PNGs.
- Only the on-chain transaction hash is disclosed as the cross-reference.
- `ROBOT_PAYEE_ADDRESS` / `ROBOT_ID` / `ROBOT_PRIVATE_KEY` are read from
  environment variables at runtime and never committed.
