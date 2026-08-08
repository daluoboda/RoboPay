# Task traceability

This document ties every success-criterion item of the Tier 1 simulator track
to the file or evidence that proves it.

| # | Criterion | Evidence |
|---|----------|----------|
| 1 | Real 402 payment challenge | `docs/evidence/terminal/laok-push-arm-402.png`, `bridge/laok_push_arm_001_zenoh_bridge.py` (`do_GET`/`do_POST` 402 path) |
| 2 | Real x402 verify + settle on Base Sepolia | `x402-evidence.json` (2 hashes), first tx `0xc660c41948a326909112c471f9553ddda204a71952e5ca9a5ec2bbc3fe9aaeba` |
| 3 | On-chain settlement confirmed | basescan link above; payer `0xA0723A2d…3d13` 18.9 → 18.7 USDC across the two settlements; re-verified by `verify_settlement.py` |
| 4 | Simulator executes the skill | `docs/evidence/terminal/laok-push-arm-mujoco.png`, `bridge/simulator.py` (`MuJoCoSimulator.push_object`) |
| 5 | Object physically displaced | metrics: `objectPushed 0.1087 m` horizontally, `objectLifted -0.0004 m` (stays on the table), `contactSamples 128`, `peakForce 6.10 N`, `graspState closed` |
| 6 | Async action/result over Zenoh | `docs/evidence/terminal/laok-push-arm-async.png`, `bridge/laok_push_arm_001_zenoh_bridge.py` (`ZenohTransport`) |
| 7 | actionId correlation | result envelope carries `actionId`; `execution-mapping.yaml` `correlationField: actionId` |
| 8 | Payment gates execution | `bridge/x402_client.py` (`PaymentGate.check`) |
| 9 | Idempotency / no double execution | `bridge/laok_push_arm_001_zenoh_bridge.py` (`IdempotencyStore`), `tests/test_bridge.py` |
| 10 | Failure never settles | `bridge/laok_push_arm_001_zenoh_bridge.py` (`_execute` settles only on success) |
| 11 | Reproducible by reviewer | `README.md`, `bridge/requirements.txt`, `tests/requirements.txt` |

## Privacy handling

- Wallet addresses and x402 signatures are withheld from public PNGs.
- Only the on-chain transaction hash is disclosed as the cross-reference.
- `ROBOT_PAYEE_ADDRESS` / `ROBOT_ID` / `ROBOT_PRIVATE_KEY` are read from
  environment variables at runtime and never committed.
