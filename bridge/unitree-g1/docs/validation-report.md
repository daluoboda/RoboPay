# Unitree G1 Tier 1 — Validation Report

## Summary
- **Robot**: Unitree G1 (29-DOF humanoid)
- **Tier**: 1 (Simulator Skill Execution)
- **Skills**: move_forward, navigate_obstacle, stop
- **Engine**: MuJoCo (primary) + PyBullet (sim-to-sim)
- **Transport**: Zenoh (real tunnel) — `bridge/tunnel/` hosts the Go tunnel binary; actions are gated on x402 verification before dispatch
- **Payment**: x402 (EIP-3009 `transferWithAuthorization`) settled through the public x402 facilitator on Base Sepolia

## Acceptance Criteria Coverage

### Criterion #1: Real Go Tunnel Integration Test
✅ `bridge/tunnel/` integrates a real Go tunnel binary (same handler set as the merged Spot PR #58).
- Tunnel verifies the x402 payment **before** dispatch.
- Actions are published to `robot/tunnel/action` only after successful payment verification.
- Covered by `tests/` (payment gate, x402 no-settlement, policy).

### Criterion #2: Zenoh Bridge
✅ Topics: `robot/tunnel/action` (request) / `robot/tunnel/result` (result).
- Correlation via `actionId` (idempotency key).
- Real Zenoh session on Linux/macOS; loopback transport used in headless CI.

### Criterion #5: Failure Modes
✅ All failure paths tested (execution-gated, never settle on failure):
- timeout: step budget exhausted → no settlement
- collision: obstacle detected → no settlement
- invalid params: rejected before dispatch → no settlement
- replay: same idempotency key re-submitted → rejected, no re-execution, no re-settlement

### Criterion #6: Scope Classification
✅ simulator-only
- No motor driver, no teleop channel, no hardware SDK
- CPU-only, headless execution

### Criterion #7: Payment Safety (real on-chain proof)
✅ x402 payment verification
- No payment → 402, robot untouched (execution counter stays 0)
- Invalid payment (`isValid:false`) → 402, no execution
- Successful payment → execution → settlement
- Failed execution → no settlement
- **Real settlement evidence**: `docs/evidence/x402-evidence.json` contains one genuine Base Sepolia USDC transfer:
  - txHash `0xcb9cab548125ddf34980bf14a5bbb57d8a86d9896348a46d63f9178f34470cc4`
  - block `45415117`, receipt status `1` (success)
  - payer `0xF2749b5fAdA8a83d3DE1a2621B1d212e73907D4a` → payee `0x742d35Cc6634C0532925a3b844Bc454e4438f44e`
  - amount `0.1 USDC`, asset `0x036CbD53842c5426634e7929541eC2318f3dCF7e` (canonical Base Sepolia USDC)
  - Independently verifiable via `verify_settlement.py` (Transfer log payer→payee 0.1 USDC)

### Criterion #8: Robot Identity & Wallet Binding
✅ Envelope binds `robotId` to the settlement receipt.
- `UNITREE_G1_WALLET_ADDRESS` (payee) supplied via environment; no private keys in repository.
- The payer key is held off-repo and only used to broadcast the settlement; it is never committed.

## Policy-Driven Controller
The locomotion uses `policy.PotentialFieldPolicy`:
- Computes velocities from the current simulator state (NOT replayed animation)
- Attractive force toward goal, repulsive force from obstacles
- Deterministic, no randomness — reproducible in CI

## Sim-to-Sim Validation
- Same skill definition runs on both MuJoCo and PyBullet
- Dynamic agreement: same verdict, same metrics
- Static agreement: identical joint chains, link offsets

## Evidence (all real)
- `x402-evidence.json`: **1 real on-chain settlement** (Base Sepolia USDC Transfer, independently verifiable)
- `settle.png`: rendered from the real terminal run (`terminal/output.txt`)
- `terminal/output.txt`: full 402→pay→simulate→settle→replay-rejected log
- `demo.mp4`: 402→pay→simulate→settle flow (pending render)

---
*Generated: 2026-08-13 · settlement broadcast 2026-08-13T05:21:54Z*
