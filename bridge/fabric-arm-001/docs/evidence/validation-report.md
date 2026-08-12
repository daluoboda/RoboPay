# Validation report

## Submission

- Brand: Fabric Foundation × laok fabric-arm-001
- Scope: simulated manipulator (MuJoCo)
- Tier: 1 — simulator skill execution
- Robot:  (4-DoF arm + gripper, MuJoCo model)
- Payment network: Base Sepolia ()
- Payment asset: USDC 
- Skill: 
- Price:  ( smallest units)

The payer/payee wallet addresses and on-chain transaction hashes are disclosed in the canonical report under `registry/…/docs/validation-report.md` and `x402-evidence.json` (payer `0xf2749b5fAdA8a83d3DE1a2621b1d212e73907D4a` → payee `0x742d35Cc6634C0532925a3b844Bc454e4438f44e`, settled on Base Sepolia).

## What was validated live

- [x] An unpaid action request returned a real **HTTP 402 Payment Required**
  with the  header advertising the x402 challenge.
- [x] A real x402 payment was **verified and settled on Base Sepolia**.
- [x] Headless MuJoCo simulation executes the pick task successfully.
- [x] Sim-to-Sim validation: MuJoCo ↔ PyBullet agreement PASS.

## Limitations

- Payment gate test_bridge.py coverage: incomplete (no 402/409 unit tests)
- Dynamic engine agreement layer CI_GATED on Windows (PyBullet wheel unavailable in CI)

## Evidence

See  for:
-  — MuJoCo ↔ PyBullet agreement
-  — Payment gate validation results
-  — SHA-256 manifest

Generated: 2026-08-12
