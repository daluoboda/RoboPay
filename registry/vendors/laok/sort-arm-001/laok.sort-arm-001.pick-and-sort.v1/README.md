# laok × sort-arm-001 — Tier 1 simulated pick-and-sort

A Tier 1 RoboPay integration: a **simulated 2-DOF arm (`sort-arm-001`)** that
performs a physics-executed pick-and-sort skill in **MuJoCo**, gated by a
verified **x402 USDC payment on Base Sepolia**, with the result returned
**asynchronously over Zenoh** (`robot/tunnel/action` → `robot/tunnel/result`).

## Layout

```
laok.sort-arm-001.pick-and-sort.v1/
├── robot.profile.yaml          # identity, runtime, submission scope/tier
├── skills.yaml                 # published skill: stack_arm_pick_and_stack
├── execution-mapping.yaml      # envelope schema, replay protection
├── functions.yaml              # agent function surface (skills/actions/status)
├── payment-policy.yaml         # x402 policy: eip155:84532, USDC, 0.10
├── examples/
│   └── action-envelope.pick-place.json
├── bridge/
│   ├── laok_stack_arm_001_zenoh_bridge.py   # HTTP + Zenoh bridge
│   ├── x402_client.py                    # payment gate + facilitator verify/settle
│   ├── simulator.py                      # MuJoCo pick_and_stack
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
python laok_stack_arm_001_zenoh_bridge.py --port 8080
```

## What is proven

- **Real on-chain settlement.** A live x402 payment was verified and settled on
  Base Sepolia through the public facilitator. Tx
  `0xcf0222171e83fd6c0d3981cf202de984c1dd0cb10f06d81eef76da779a5fb6d2`,
  on-chain status `1`, payer balance `19.8 → 19.7 USDC`.
- **Real physics execution.** MuJoCo `pick_and_stack` lifted the object `0.1313 m`
  with zero collisions.
- **Async contract.** Unpaid → 402; paid → 202 accepted; correlated success
  result on the result topic; duplicate / replay / expired requests are
  rejected before any actuation; failure never settles.

See `docs/validation-report.md` and `docs/evidence/evidence-manifest.yaml`.
