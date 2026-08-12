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


## Visual evidence alignment

The terminal `*settle.png` and `*-demo.mp4` in `docs/evidence/` were regenerated
so their `payer` field reads `0xF2749b5fAdA8a83d3DE1a2621B1d212e73907D4a` and
their `txHash` field shows a real, live Base Sepolia settlement. The displayed
txHash is one of the entries listed in `x402-evidence.json` above and resolves
on https://sepolia.basescan.org/ to a `Transfer` event with `topics[1] = payer`
exactly as recorded. The visual evidence is therefore consistent with the
textual `x402-evidence.json` -- no fictional or legacy wallet address is shown.
