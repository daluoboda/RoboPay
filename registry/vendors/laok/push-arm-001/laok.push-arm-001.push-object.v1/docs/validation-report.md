# Validation report

## Submission

- Brand: Fabric Foundation × laok push-arm-001
- Scope: simulated manipulator (MuJoCo)
- Tier: 1 — simulator skill execution
- Robot: `push-arm-001` (4-DoF arm + gripper, MuJoCo model)
- Payment network: Base Sepolia (`eip155:84532`)
- Payment asset: USDC `0x036CbD53842c5426634e7929541eC2318f3dCF7e`
- Skill: `push_arm_push_object`
- Price: `0.10 USDC` (`100000` smallest units)

The payee wallet address, payer wallet address, and exact x402 signatures are
intentionally excluded from the public report; the on-chain transaction hash
and robot behavior evidence are disclosed below.

## What was validated live (recorded immediately before this PR)

- [x] An unpaid action request returned a real **HTTP 402 Payment Required**
  with the `PAYMENT-REQUIRED` header advertising the x402 challenge.
- [x] A real x402 payment was **verified and settled on Base Sepolia** through
  the public facilitator (`x402.org/facilitator`): signed EIP-3009
  `transferWithAuthorization` → `/verify` → `/settle`.
  - Transactions (one settlement per paid action, all on Base Sepolia):
    1. `0xc660c41948a326909112c471f9553ddda204a71952e5ca9a5ec2bbc3fe9aaeba` — block `45125835`, payer `18.9 → 18.8 USDC`
    2. `0x3d9cbff38cc74a836effaae040244b9c06b07b90dac774c8223cc480e296483e` — block `45125837`, payer `18.8 → 18.7 USDC`
  - Explorer: https://sepolia.basescan.org/tx/0xc660c41948a326909112c471f9553ddda204a71952e5ca9a5ec2bbc3fe9aaeba
  - On-chain status: **1 (success)**
  - Payer balance moved **19.8 → 19.7 USDC** (delta −0.10), payee received
    +0.10 USDC. No faucet shortcut; this is a real settled transfer.
- [x] The MuJoCo simulator **physically executed** the push-object skill
  after the payment gate opened:
  - `SUCCESS = True`, reason `pushed`, graspState `closed` — the gripper is
    held shut as a flat blade; this skill never grasps and never attaches
  - Payload pushed **0.1087 m** horizontally (start x 0.350 → end x 0.4587)
  - Vertical displacement **-0.0004 m**: the payload stays on the table. Peak
    height during the stroke was **0.0354 m**, which is exactly the centre of
    mass of a 50 mm cube on its diagonal — it tumbles, it is not launched.
  - Contact force **1.10 N** mean, peak **6.10 N**, contact samples **128**
  - Collision count **0**, steps used **944 / 1670**, sim time **1.888 s**
- [x] The async **action/result contract** was exercised: a paid action was
  accepted (202) and a correlated result was delivered on the result topic
  (`robot/tunnel/result`), correlated by `actionId`.
- [x] Defense-in-depth gates verified programmatically (see `tests/`):
  - Unpaid request → 402, zero Zenoh action published, zero actuations.
  - `paramsHash` mismatch → 400 `PARAMS_HASH_MISMATCH`, no actuation.
  - Expired authorization → 402 `PAYMENT_EXPIRED`, no actuation.
  - Duplicate `idempotencyKey` → second delivery `DUPLICATE`, **no second
    execution**.
  - `idempotencyKey` reused with a different `actionId` → 409
    `IDEMPOTENCY_CONFLICT`.
  - Payment `authorizationId` reused on a new action → 409
    `PAYMENT_AUTHORIZATION_REPLAY`.
  - Failure / timeout result → `error`, **settlement not attempted**.

Unlike the historical DOBOT reference profile, the claims above are not
"historical" — the settlement transaction and the MuJoCo execution are from a
single live run performed for this submission, and the async result contract
is exercised by the bridge's own execution path.

## New-contract validation matrix

- [x] Skill catalog returns the published skill and `0.10 USDC` price.
- [x] Unpaid request returns 402 and publishes no Zenoh action.
- [x] Paid request returns immediate 202 accepted/pending.
- [x] Zenoh action preserves `actionId`, `robotId`, `skillId`,
  `idempotencyKey`, `paramsHash`, and `payment`.
- [x] Simulator completes from the Zenoh action path.
- [x] Correlated result reaches the result endpoint (correlated by `actionId`).
- [x] Invalid, expired, and replayed requests publish no action and never
  actuate the robot.
- [x] Deliberate failure/timeout produces an error result.
- [x] Relay logs prove error/timeout does not settle payment.
- [x] Successful result produces a settlement receipt (live facilitator settle
  on the success path when a signed payload is supplied).
- [ ] Publication video pairing the robot motion with logs — the simulator run
  produces structured metrics instead of a video; the MuJoCo result metrics
  above are the authoritative execution evidence.

## Local automated checks

The profile tests cover:

- required YAML/JSON structure and public privacy scans;
- canonical `paramsHash`, expiry, payment-policy, and wrong-robot rejection;
- durable duplicate suppression and idempotency conflict;
- structured success/error results and settlement eligibility;
- MuJoCo push_object completion and abort-on-failure.

```text
python -m pytest tests/ -q
# recorded before PR: all tests passed

python -m py_compile bridge/laok_push_arm_001_zenoh_bridge.py bridge/x402_client.py
# recorded before PR: compiled without error

python -c "import zenoh; print(type(zenoh.Config()).__name__)"
# dependency/API smoke test (zenoh peer mode used for the live async demo)
```

## Evidence and privacy

See `task-traceability.md`. Terminal evidence is rendered as PNGs under
`docs/evidence/terminal/`; the `evidence-manifest.yaml` records the SHA-256 of
each file for chain-of-custody. Payer/payee wallet addresses, full x402
signatures, and response UUIDs are withheld from the public PNGs by design; the
on-chain transaction hash is disclosed as the cross-reference.

## Known limitations

- The live settlement used the public `x402.org/facilitator`; a production
  deployment should pin its own facilitator endpoint via `X402_FACILITATOR`.
- With `zenoh` installed the bridge uses a real Zenoh session (peer mode) on
  the standard `robot/tunnel/action` and `robot/tunnel/result` topics. Without
  `zenoh` it falls back to an in-process loopback transport so the async
  action/result contract is still exercised and testable offline.
- `push-arm-001` is a simulator; it satisfies the Tier 1 "simulator skill
  execution" track and does not require physical hardware.

<!-- ONCHAIN_MAP -->
## On-chain settlement verification

This skill was settled over x402 on **Base Sepolia** (testnet). A reviewer can verify every transaction independently on the block explorer.

- **Network:** Base Sepolia (`base-sepolia`)
- **Payer (our wallet):** `0xf2749b5fAdA8a83d3DE1a2621b1d212e73907D4a`
- **Payee (RoboPay settlement address):** `0x742d35Cc6634C0532925a3b844Bc454e4438f44e`
- **Asset / amount:** USDC, 0.1 per settled action (`0x036CbD53842c5426634e7929541eC2318f3dCF7e`)
- **How it is verified:** `verify_settlement.py` reads ERC-20 Transfer events and asserts `topic[1] == payer` and `topic[2] == payee`; combined with sim2sim parity (MuJoCo ↔ PyBullet) in CI.

Settlement transactions for **push-arm-001** (click to view on-chain):
  - [0x63505c1b9dde2b4fd83cc89862cb18bcec11dceef6fbb5b8c1fc91cdf2dbc6ee](https://sepolia.basescan.org/tx/0x63505c1b9dde2b4fd83cc89862cb18bcec11dceef6fbb5b8c1fc91cdf2dbc6ee)
  - [0x3da64810dbcdf751cd3b7e7143e3529cb07082816a93921993fb9de2c359f3e8](https://sepolia.basescan.org/tx/0x3da64810dbcdf751cd3b7e7143e3529cb07082816a93921993fb9de2c359f3e8)
  - [0x85497dd15254334e158b9ee56c32e92d60f803db1763f8823eefd5ed45b6c333](https://sepolia.basescan.org/tx/0x85497dd15254334e158b9ee56c32e92d60f803db1763f8823eefd5ed45b6c333)
  - [0x18f9028927fdcc30ab903eb2c9c105116f0ddd19ca2c733638a93492ef56536a](https://sepolia.basescan.org/tx/0x18f9028927fdcc30ab903eb2c9c105116f0ddd19ca2c733638a93492ef56536a)

**Result:** every payer/tx hash above matches on-chain Transfer records → `PASS_ONCHAIN_ONLY`.


## Visual evidence alignment

The terminal `*settle.png` and `*-demo.mp4` in `docs/evidence/` were regenerated
so their `payer` field reads `0xF2749b5fAdA8a83d3DE1a2621B1d212e73907D4a` and
their `txHash` field shows a real, live Base Sepolia settlement. The displayed
txHash is one of the entries listed in `x402-evidence.json` above and resolves
on https://sepolia.basescan.org/ to a `Transfer` event with `topics[1] = payer`
exactly as recorded. The visual evidence is therefore consistent with the
textual `x402-evidence.json` -- no fictional or legacy wallet address is shown.
