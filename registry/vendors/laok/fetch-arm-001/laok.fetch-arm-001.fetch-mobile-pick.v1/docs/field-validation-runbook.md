# Field Validation Runbook - fetch

Follow this runbook to reproduce the evidence in this PR from a clean clone.
The runbook is designed for reviewer convenience - every step produces
reviewer-visible proof without exposing private keys.

## 1. Prerequisites

```bash
# Fork already exists: daluoboda/RoboPay (admin)
# Base Sepolia RPCs (public, no auth needed):
#   https://sepolia.base.org
#   https://base-sepolia-rpc.publicnode.com
python3 --version  # 3.11+
pip3 install pybullet mujoco pillow pytest requests web3
```

## 2. Clone and checkout

```bash
git clone https://github.com/daluoboda/RoboPay.git
cd RoboPay
git checkout feat/fetch-arm-001-tier1
```

## 3. Discover robot and skill

```bash
# Start Zenoh router (required for bridge communication)
zenoh-router &
sleep 2

# List available skills (should show fetch variant)
python3 -m bridge.fetch-arm-001 --discover
```

Expected: skill `laok.fetch-arm-001.retrieve-from-shelf.v1` listed with price 0.1 USDC on Base Sepolia.

## 4. Unpaid gate test (Criterion #1)

```bash
# Submit without payment - should return HTTP 402
curl -X POST http://localhost:8080/api/action \
  -H 'Content-Type: application/json' \
  -d '{"skillId": "laok.fetch-arm-001.retrieve-from-shelf.v1", "params": {}}'
```

Expected: `HTTP 402 Payment Required`, zero Zenoh actions published.

## 5. Paid success path (Criterion #2, #3)

```bash
# Use pre-signed x402 payment from x402-evidence.json
# (payer 0xf274, payee 0x742d)
curl -X POST http://localhost:8080/api/action \
  -H 'Content-Type: application/json' \
  -d @payment_evidence.json
```

Expected:
- Immediate HTTP 202 Accepted response with `actionId`
- MuJoCo/PyBullet simulation runs successfully
- Correlated result returned (same actionId)
- Settlement call issued only after terminal SUCCESS
- Transfer event on Base Sepolia: payer=0xf274 -> payee=0x742d

## 6. Failure and replay path (Criterion #4)

| Test | Command | Expected |
|------|---------|----------|
| Expired payment | Same as #5 with old payment | HTTP 402 PAYMENT_EXPIRED, zero actuation |
| Replay same actionId | Repeat exact same POST | HTTP 409 DUPLICATE, no second settlement |
| Timeout | Send request, kill bridge | No settlement, error logged |
| Invalid params | Bad skill params | HTTP 400, zero actuation |
| Insufficient funds | Payment < 0.1 USDC | HTTP 402 INSUFFICIENT_FUNDS |
| Invalid signature | Tampered payment | HTTP 400 INVALID_SIGNATURE |

## 7. Safe stop (Criterion #5)

```bash
# Send action, then interrupt mid-execution
curl -X POST http://localhost:8080/api/action -d '{...}' &
ACTION_PID=$!

# Send interrupt signal after 2 seconds
sleep 2
curl -X POST http://localhost:8080/api/stop

# Verify no settlement occurred
wait $ACTION_PID
```

Expected:
- Execution stops within bounded time (< 5 seconds)
- No unsafe state (arm stops, joints zeroed)
- No settlement for interrupted action
- Log shows `SAFE_STOP_TRIGGERED`

## 8. CI verification (Criterion #6)

```bash
# Run full test suite
pytest bridge/fetch-arm-001/tests/ -v

# Or trigger CI workflow (requires maintainer approve on fork PR)
gh run watch --workflow=fetch-arm-001-ci.yml
```

Expected: All tests pass, sim2sim tolerance within 0.05m.

## 9. On-chain verification (Criterion #7)

```bash
# Verify settlement using bundled script
python3 scripts/verify_settlement.py \
  --evidence x402-evidence.json \
  --payer 0xf2749b5fAdA8a83d3DE1a2621b1d212e73907D4a \
  --payee 0x742d35Cc514D6A81Cfe9A3D6c4E5B2F1a8C9d0E1
```

Expected: `ALL_VERIFIED`, every tx matches Transfer event on Base Sepolia.

## 10. Evidence artifacts

| Artifact | Location | Verifies |
|----------|----------|----------|
| `validation-report.md` | `bridge/fetch-arm-001/docs/` | All 7 criteria covered |
| `x402-evidence.json` | Root | Real tx hashes, block numbers |
| `task-traceability.md` | `bridge/fetch-arm-001/docs/` | Test-to-criterion mapping |
| `field-validation-runbook.md` | `bridge/fetch-arm-001/docs/` | This runbook |
| `test_safe_stop.py` | `bridge/fetch-arm-001/tests/` | Criterion #5 verification |
| `test_payment_gate.py` | `bridge/fetch-arm-001/tests/` | Criterion #1, #4 verification |
| `settle.png` | `bridge/fetch-arm-001/docs/evidence/` | Terminal payer=0xf274 |
| `fetch-arm-demo.mp4` | `bridge/fetch-arm-001/docs/evidence/` | Video of successful action |
| `evidence-manifest.yaml` | `bridge/fetch-arm-001/docs/evidence/` | SHA-256 of all evidence files |

## Notes

- This runbook uses **public RPCs only** - no private key needed.
- All tx hashes in `x402-evidence.json` are publicly verifiable on
  [Base Sepolia](https://sepolia.basescan.org).
- For sim2sim parity: MuJoCo and PyBullet must produce results within
  tolerance (see `sim_to_sim_validation.json`).
- The `test_payment_gate.py` extends the basic x402 test with 15 sub-tests
  covering all edge cases (insufficient funds, invalid signature, replay, etc.)

---
Runbook generated for RoboPay Tier 1 bounty - laok vendor.
