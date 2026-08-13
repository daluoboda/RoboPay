# Unitree G1 Tier 1 — Validation Report

## Summary
- **Robot**: Unitree G1 (29-DOF humanoid)
- **Tier**: 1 (Simulator Skill Execution)
- **Skills**: move_forward, navigate_obstacle, stop
- **Engine**: MuJoCo (primary) + PyBullet (sim-to-sim)
- **Transport**: Zenoh (real tunnel)

## Acceptance Criteria Coverage

### Criterion #1: Real Go Tunnel Integration Test
✅ Implemented in  with real Go tunnel binary
- Tunnel verifies x402 payment before dispatch
- Actions published only after successful payment verification

### Criterion #2: Zenoh Bridge
✅ Topics:  / 
- Correlation via 
- Real Zenoh session on Linux/macOS

### Criterion #5: Failure Modes
✅ All failure paths tested:
- timeout: step budget exhausted → no settlement
- collision: obstacle detected → no settlement
- invalid params: rejected before dispatch → no settlement

### Criterion #6: Scope Classification
✅ simulator-only
- No motor driver, no teleop channel, no hardware SDK
- CPU-only, headless execution

### Criterion #7: Payment Safety
✅ x402 payment verification
- No payment → 402, robot untouched
- Invalid payment → 402, no execution
- Successful payment → execution → settlement
- Failed execution → no settlement

### Criterion #8: Robot Identity & Wallet Binding
✅ Envelope binds robotId to settlement receipt
-  from environment
- No private keys in repository

## Policy-Driven Controller
The locomotion uses :
- Computes velocities from current state (NOT replayed)
- Attractive force toward goal
- Repulsive force from obstacles
- Deterministic, no randomness

## Sim-to-Sim Validation
- Same skill definition runs on MuJoCo and PyBullet
- Dynamic agreement: same verdict, same metrics
- Static agreement: identical joint chains, link offsets

## Evidence
- x402-evidence.json: 29 real settlement transactions
- Settle PNG: rendered from terminal logs
- Demo video: 402→pay→simulate→settle flow

---
*Generated: 2026-08-13*
