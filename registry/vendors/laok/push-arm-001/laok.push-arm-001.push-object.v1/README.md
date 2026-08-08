# laok × push-arm-001 — Tier 1 simulated push-object

A Tier 1 RoboPay integration: a **simulated 2-DOF arm (`push-arm-001`)** that
performs a physics-executed push-object skill in **MuJoCo**, gated by a
verified **x402 USDC payment on Base Sepolia**, with the result returned
**asynchronously over Zenoh** (`robot/tunnel/action` → `robot/tunnel/result`).

## Layout

```
laok.push-arm-001.push-object.v1/
├── robot.profile.yaml          # identity, runtime, submission scope/tier
├── skills.yaml                 # published skill: push_arm_push_object
├── execution-mapping.yaml      # envelope schema, replay protection
├── functions.yaml              # agent function surface (skills/actions/status)
├── payment-policy.yaml         # x402 policy: eip155:84532, USDC, 0.10
├── examples/
│   └── action-envelope.push-object.json
├── bridge/
│   ├── laok_push_arm_001_zenoh_bridge.py   # HTTP + Zenoh bridge
│   ├── x402_client.py                    # payment gate + facilitator verify/settle
│   ├── simulator.py                      # MuJoCo push_object
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
export ROBOT_ID=push-arm-001-demo-001
export ROBOT_PAYEE_ADDRESS=0x742d35Cc6634C0532925a3b844Bc454e4438f44e

# 1) run the behavioral + structural tests
cd ../tests && python -m pytest -q

# 2) reproduce the live async pay-to-actuate evidence (no zenoh required)
cd ../bridge && python gen_bridge_evidence.py

# 3) (optional) run the HTTP + Zenoh bridge
python laok_push_arm_001_zenoh_bridge.py --port 8080
```

## What is proven

- **Real on-chain settlement.** A live x402 payment was verified and settled on
  Base Sepolia through the public facilitator. Tx
  `0xc660c41948a326909112c471f9553ddda204a71952e5ca9a5ec2bbc3fe9aaeba`
  (block `45125835`), on-chain status `1`, payer balance `18.9 → 18.8 USDC`.
  A second paid action settled as
  `0x3d9cbff38cc74a836effaae040244b9c06b07b90dac774c8223cc480e296483e`
  (block `45125837`), taking the payer to `18.7 USDC`. Both hashes are listed in
  `x402-evidence.json` and re-checked against the chain by
  `verify_settlement.py` in CI.
- **Real physics execution.** MuJoCo `push_object` shoved the payload
  `0.1087 m` horizontally (x `0.350 → 0.459 m`) over `128` steps of measured
  blade contact, peak normal force `6.10 N`, zero collisions. Vertical
  displacement is `-0.0004 m`: the payload never leaves the table, which is
  the whole point of a push. Peak height during the stroke is `0.0354 m`,
  exactly the centre of mass of a 50 mm cube standing on its diagonal — it
  tumbles across the table, it is not launched.
- **Async contract.** Unpaid → 402; paid → 202 accepted; correlated success
  result on the result topic; duplicate / replay / expired requests are
  rejected before any actuation; failure never settles.

See `docs/validation-report.md` and `docs/evidence/evidence-manifest.yaml`.
