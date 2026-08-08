# laok × sort-arm-001 — Tier 1 simulated pick-and-sort

A Tier 1 RoboPay integration: a **simulated 2-DOF arm (`sort-arm-001`)** that
performs a physics-executed pick-and-sort skill in **MuJoCo**, gated by a
verified **x402 USDC payment on Base Sepolia**, with the result returned
**asynchronously over Zenoh** (`robot/tunnel/action` → `robot/tunnel/result`).

## Layout

```
laok.sort-arm-001.pick-and-sort.v1/
├── robot.profile.yaml          # identity, runtime, submission scope/tier
├── skills.yaml                 # published skill: sort_arm_pick_and_sort
├── execution-mapping.yaml      # envelope schema, replay protection
├── functions.yaml              # agent function surface (skills/actions/status)
├── payment-policy.yaml         # x402 policy: eip155:84532, USDC, 0.10
├── examples/
│   └── action-envelope.pick-place.json
├── bridge/
│   ├── laok_sort_arm_001_zenoh_bridge.py   # HTTP + Zenoh bridge
│   ├── x402_client.py                    # payment gate + facilitator verify/settle
│   ├── simulator.py                      # MuJoCo pick_and_sort
│   ├── arm_spec.py
│   ├── requirements.txt
│   ├── config.example.json
│   ├── gen_bridge_evidence.py            # reproduces the async evidence
│   └── render_evidence.py                # renders terminal PNGs
├── docs/
│   ├── validation-report.md
│   ├── task-traceability.md
│   └── evidence/
│       ├── evidence-manifest.yaml
│       └── terminal/*.png
└── tests/
    ├── test_bridge.py
    ├── test_profile.py
    ├── skill-contract.test.yaml
    └── requirements.txt
```

## Reproduce locally

```bash
cd bridge
python -m venv .venv && .venv/Scripts/pip install -r requirements.txt
export ROBOT_ID=sort-arm-001-demo-001
export ROBOT_PAYEE_ADDRESS=0x742d35Cc6634C0532925a3b844Bc454e4438f44e

# 1) run the behavioral + structural tests
cd ../tests && python -m pytest -q

# 2) reproduce the live async pay-to-actuate evidence (no zenoh required)
cd ../bridge && python gen_bridge_evidence.py

# 3) (optional) run the HTTP + Zenoh bridge
python laok_sort_arm_001_zenoh_bridge.py --port 8080
```

## What is proven

- **Real on-chain settlement.** A live x402 payment was verified and settled on
  Base Sepolia through the public facilitator. Tx
  `0xea4f54c97c26762915615023cffeb942f858c51963ef1e2117de4de28b8f16d0`
  (block `45125827`), on-chain status `1`, payer balance `19.2 → 19.1 USDC`.
  Across all three settlements the payer went `19.2 → 18.9 USDC`. Every hash is
  listed in `x402-evidence.json` and re-checked against the chain by
  `verify_settlement.py` in CI.
- **Real physics execution.** MuJoCo `pick_and_sort` grasped the incoming object,
  carried it `0.1354 m` above the table (`peakLift`) and released it into bin A
  `0.0265 m` from the bin centre. It is carried through the air, not dragged:
  the net `objectLifted` of `0.0014 m` is simply where it comes back to rest.
  Peak grip force `6.55 N`, `430 / 450` steps, zero collisions.
- **Async contract.** Unpaid → 402; paid → 202 accepted; correlated success
  result on the result topic; duplicate / replay / expired requests are
  rejected before any actuation; failure never settles.

See `docs/validation-report.md` and `docs/evidence/evidence-manifest.yaml`.
