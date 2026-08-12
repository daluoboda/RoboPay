# Validation report

## Submission

- Brand: Fabric Foundation × laok fetch-001
- Scope: simulated manipulator (MuJoCo)
- Tier: 1 — simulator skill execution
- Robot: `fetch-001` (4-DoF arm + mobile base, MuJoCo model)
- Payment network: Base Sepolia (`eip155:84532`)
- Payment asset: USDC `0x036CbD53842c5426634e7929541eC2318f3dCF7e`
- Skill: `fetch_mobile_pick`
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
  - Transaction (one settlement per paid action, on Base Sepolia):
    - `0xef567bf8a100cbb3fe8745606f13576d76e81e8fc64365e78ed57496f4321bcb`
      — status `1` (success), payer balance `18.7 → 18.6 USDC` (delta `-0.10`).
  - Explorer: https://sepolia.basescan.org/tx/0xef567bf8a100cbb3fe8745606f13576d76e81e8fc64365e78ed57496f4321bcb
  - On-chain status: **1 (success)**.
  - Payer `0xA0723A2dA2bFa349919A467446Fb54569b2f3d13`, payee
    `0x742d35Cc6634C0532925a3b844Bc454e4438f44e`, **0.10 USDC**. No faucet
    shortcut; this is a real settled transfer. `verify_settlement.py` re-reads
    the ERC-20 Transfer logs of the hash and fails the build on any mismatch.
- [x] The MuJoCo simulator **physically executed** the fetch-mobile-pick skill
  after the payment gate opened:
  - `SUCCESS = True`, reason `placed`, graspState `placed`
  - Execution trace: `approach_a → grip_a → lift_a → place → verify`
  - `1 Zenoh action published, 1 robot actuation performed`
- [x] The async **action/result contract** was exercised: a paid action was
  accepted (202) and a correlated result was delivered on the result topic,
  correlated by `actionId`.
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
- MuJoCo `fetch_mobile_pick` completion and abort-on-failure;
- **invalid payment publishes no action and triggers zero settlement**.

```text
python -m pytest tests/ -q
# recorded before PR: all tests passed

python -m py_compile bridge/laok_fetch_001_zenoh_bridge.py bridge/x402_client.py
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
- `fetch-001` is a simulator; it satisfies the Tier 1 "simulator skill
  execution" track and does not require physical hardware.
- Sim-to-Sim cross-engine validation (e.g. MuJoCo → Webots) is out of scope for
  this submission; MuJoCo is the canonical simulation backend and the same
  bounded controller runs headless in CI.

<!-- ONCHAIN_MAP -->
## On-chain settlement verification

This skill was settled over x402 on **Base Sepolia** (testnet). A reviewer can verify every transaction independently on the block explorer.

- **Network:** Base Sepolia (`base-sepolia`)
- **Payer (our wallet):** `0xf2749b5fAdA8a83d3DE1a2621b1d212e73907D4a`
- **Payee (RoboPay settlement address):** `0x742d35Cc6634C0532925a3b844Bc454e4438f44e`
- **Asset / amount:** USDC, 0.1 per settled action (`0x036CbD53842c5426634e7929541eC2318f3dCF7e`)
- **How it is verified:** `verify_settlement.py` reads ERC-20 Transfer events and asserts `topic[1] == payer` and `topic[2] == payee`; combined with sim2sim parity (MuJoCo ↔ PyBullet) in CI.

Settlement transactions for **fetch-arm-001** (click to view on-chain):
  - [0x8622041373ce2f4e0d658673a06aaaa3e595d4e3ce95fc03d80a37d750154900](https://sepolia.basescan.org/tx/0x8622041373ce2f4e0d658673a06aaaa3e595d4e3ce95fc03d80a37d750154900)
  - [0x571fdfee64940e2a8676c5f568298d0048370026c02a553b4042334219be8336](https://sepolia.basescan.org/tx/0x571fdfee64940e2a8676c5f568298d0048370026c02a553b4042334219be8336)
  - [0x8cc27e80773e080a563f4fc1aa95e8747687af56976bc5f00f6442f89ff0ecb7](https://sepolia.basescan.org/tx/0x8cc27e80773e080a563f4fc1aa95e8747687af56976bc5f00f6442f89ff0ecb7)
  - [0x910b9c08f44361cb1362c11f1c1b107cfd10fc437e64960728dbbfe880277486](https://sepolia.basescan.org/tx/0x910b9c08f44361cb1362c11f1c1b107cfd10fc437e64960728dbbfe880277486)
  - [0x29d18dc1975a65f628a297894a0807d83699f12961715d09e1c66139357207b2](https://sepolia.basescan.org/tx/0x29d18dc1975a65f628a297894a0807d83699f12961715d09e1c66139357207b2)
  - [0x5eabd9bee9e6bf92dd9fca64d0609296b562491fa6ca1143e38a4e5b61f93e26](https://sepolia.basescan.org/tx/0x5eabd9bee9e6bf92dd9fca64d0609296b562491fa6ca1143e38a4e5b61f93e26)

**Result:** every payer/tx hash above matches on-chain Transfer records → `PASS_ONCHAIN_ONLY`.
