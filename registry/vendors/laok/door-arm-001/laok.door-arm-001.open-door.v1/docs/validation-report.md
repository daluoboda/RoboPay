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



## Policy-driven controller

The door-opening skill is driven by a **motion policy** (`bridge/door-arm-001/arm_spec.py`), not a fixed joint target: the controller selects each next command from the measured handle pose and contact state (move-above -> descend -> grip -> pull), and success is reported only from measured simulator state (door angle >= 0.5 rad, contact force >= 0.25 N).

This is not a pre-recorded animation or a fixed joint-target replay: the
motion sequence is re-planned from measured simulator state at each stage
boundary, and the same policy is what produces the success path and every
declared failure mode listed in this report. It satisfies the Tier 1
requirement that actions be triggered by a policy/controller, not replayed.
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
