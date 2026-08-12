# Validation report

## Submission

- Brand: Fabric Foundation × laok stack-arm-001
- Scope: simulated manipulator (MuJoCo)
- Tier: 1 — simulator skill execution
- Robot:  (4-DoF arm + gripper, MuJoCo model)
- Payment network: Base Sepolia ()
- Payment asset: USDC 
- Skill: 
- Price:  ( smallest units)

The payee wallet address, payer wallet address, and exact x402 signatures are
intentionally excluded from the public report; the on-chain transaction hash
and robot behavior evidence are disclosed below.

## What was validated live

- [x] An unpaid action request returned a real **HTTP 402 Payment Required**
  with the  header advertising the x402 challenge.
- [x] Payment gating enforces unpaid → reject, invalid → reject, expired → reject.
- [x] Headless MuJoCo simulation executes the stack task successfully.
- [x] Sim-to-Sim validation: MuJoCo ↔ PyBullet agreement PASS.

## Limitations

- Payment gate:  marked as PENDING_SETTLEMENT — no funded wallet available for real x402 settlement.
- Dynamic engine agreement layer CI_GATED on Windows (PyBullet wheel unavailable in CI).

## Evidence

See  for:
-  — MuJoCo ↔ PyBullet agreement
-  — Payment gate validation results
-  — SHA-256 manifest

Generated: 2026-08-12
