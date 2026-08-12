# Validation report

## Submission

- Brand: Fabric Foundation × laok fetch-001
- Scope: simulated mobile manipulator (MuJoCo)
- Tier: 1 — simulator skill execution
- Robot:  (4-DoF arm + mobile base, MuJoCo model)
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
- [x] A real x402 payment was **verified and settled on Base Sepolia**:
  - Transaction: 
  - Status: 1 (success)
  - Payer:  → Payee: 
- [x] Headless MuJoCo simulation executes the fetch task successfully.
- [x] Sim-to-Sim validation: MuJoCo ↔ PyBullet agreement PASS.

## Limitations

- Dynamic engine agreement layer CI_GATED on Windows (PyBullet wheel unavailable in CI).

## Evidence

See  for:
-  — MuJoCo ↔ PyBullet agreement
-  — Payment gate validation results
-  — SHA-256 manifest

Generated: 2026-08-12
