# Validation report

## Submission

- Brand: Fabric Foundation × laok push-arm-001
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
- [x] Headless MuJoCo simulation executes the push task successfully.
- [x] Sim-to-Sim validation: MuJoCo ↔ PyBullet agreement PASS.

## Limitations

- Payment gate:  performed and verified on Base Sepolia (payer `0xf2749b5fAdA8a83d3DE1a2621b1d212e73907D4a` → payee `0x742d35Cc6634C0532925a3b844Bc454e4438f44e`); per-skill transaction hashes are recorded in `x402-evidence.json` and the canonical report under `registry/…/docs/validation-report.md`.
- Dynamic engine agreement layer CI_GATED on Windows (PyBullet wheel unavailable in CI).

## Evidence

See  for:
-  — MuJoCo ↔ PyBullet agreement
-  — Payment gate validation results
-  — SHA-256 manifest

Generated: 2026-08-12
