# Task traceability

This document ties every success-criterion item of the Tier 1 simulator track
to the file or evidence that proves it.

| # | Criterion | Evidence |
|---|----------|----------|
| 1 | Real 402 payment challenge | `docs/evidence/terminal/laok-fetch-arm-402.png`, `bridge/laok_fetch_001_zenoh_bridge.py` (`do_GET`/`do_POST` 402 path) |
| 2 | Real x402 verify + settle on Base Sepolia | `x402-evidence.json` (1 hash), tx `0xef567bf8a100cbb3fe8745606f13576d76e81e8fc64365e78ed57496f4321bcb` |
| 3 | On-chain settlement confirmed | basescan link above; payer `0xA0723A2d…3d13` 18.7 → 18.6 USDC; re-verified by `verify_settlement.py` |
| 4 | Simulator executes the skill | `docs/evidence/terminal/laok-fetch-arm-mujoco.png`, `bridge/simulator.py` (`MuJoCoSimulator.fetch_mobile_pick`) |
| 5 | Object physically fetched (mobile pick) | metrics: `graspState placed`, `success True`, `reason placed`; trace `approach_a → grip_a → lift_a → place → verify`; `bridge_async.log` |
| 6 | Async action/result over Zenoh | `docs/evidence/terminal/laok-fetch-arm-async.png`, `bridge/laok_fetch_001_zenoh_bridge.py` (`ZenohTransport`) |
| 7 | actionId correlation | result envelope carries `actionId`; `execution-mapping.yaml` `correlationField: actionId` |
| 8 | Payment gates execution | `bridge/x402_client.py` (`PaymentGate.check`) |
| 9 | Idempotency / no double execution | `bridge/laok_fetch_001_zenoh_bridge.py` (`IdempotencyStore`), `tests/test_bridge.py` |
| 10 | Failure never settles | `bridge/laok_fetch_001_zenoh_bridge.py` (`_execute` settles only on success) |
| 11 | Reproducible by reviewer | `README.md`, `bridge/requirements.txt`, `tests/requirements.txt` |

## Privacy handling

- Wallet addresses and x402 signatures are withheld from public PNGs.
- Only the on-chain transaction hash is disclosed as the cross-reference.
- `ROBOT_PAYEE_ADDRESS` / `ROBOT_ID` / `ROBOT_PRIVATE_KEY` are read from
  environment variables at runtime and never committed.
