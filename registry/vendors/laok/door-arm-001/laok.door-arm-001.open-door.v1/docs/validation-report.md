# Validation report

## Submission

- Brand: Fabric Foundation × laok door-arm-001
- Scope: simulated manipulator (MuJoCo)
- Tier: 1 — simulator skill execution
- Robot:  (6-DoF arm + parallel gripper, MuJoCo model)
- Payment network: Base Sepolia ()
- Payment asset: USDC 
- Skill: 
- Price:  ( smallest units)

## What was validated live

- [x] An unpaid action request returned a real **HTTP 402 Payment Required**.
- [x] A real x402 payment was **verified and settled on Base Sepolia**:
  - Transaction: 
  - Status: 1 (success)
- [x] Headless MuJoCo simulation executes the door opening task successfully.
- [x] Sim-to-Sim validation: MuJoCo ↔ PyBullet agreement PASS (all layers).

## Limitations

- Dynamic engine agreement layer CI_GATED on Windows (PyBullet wheel unavailable in CI).

Generated: 2026-08-12
