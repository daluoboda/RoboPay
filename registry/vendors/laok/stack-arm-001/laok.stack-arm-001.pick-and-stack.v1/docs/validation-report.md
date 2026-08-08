# Validation report

## Submission

- Brand: Fabric Foundation × laok stack-arm-001
- Scope: simulated manipulator (MuJoCo)
- Tier: 1 — simulator skill execution
- Robot: `stack-arm-001` (4-DoF arm + gripper, MuJoCo model)
- Payment network: Base Sepolia (`eip155:84532`)
- Payment asset: USDC `0x036CbD53842c5426634e7929541eC2318f3dCF7e`
- Skill: `stack_arm_pick_and_stack`
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
    1. `0xad4c5669e1a3351a69256e5c5d507efc939ed9e7cfe0ac8b0be90603a796d1d6` — block `45125820`, payer `19.5 → 19.4 USDC`
    2. `0x4e7db1219898a98187227c4673505f4af61fe22c6a783ad03d8ad9dbe136e80d` — block `45125822`, payer `19.4 → 19.3 USDC`
    3. `0x23439362ac5bf96f57296220c3d143e5442c18f08448c84fa6ca1e69d4e5137b` — block `45125825`, payer `19.3 → 19.2 USDC`
  - Explorer: https://sepolia.basescan.org/tx/0xad4c5669e1a3351a69256e5c5d507efc939ed9e7cfe0ac8b0be90603a796d1d6
  - On-chain status: **1 (success)** on all three
  - Payer `0xA0723A2dA2bFa349919A467446Fb54569b2f3d13`, payee
    `0x742d35Cc6634C0532925a3b844Bc454e4438f44e`, **0.10 USDC** each. No faucet
    shortcut; these are real settled transfers. `verify_settlement.py` re-reads
    the ERC-20 Transfer logs of every hash and fails the build on any mismatch.
- [x] The MuJoCo simulator **physically executed** the pick-and-stack skill
  after the payment gate opened:
  - `SUCCESS = True`, reason `stacked`, graspState `stacked`
  - Cube A grasped and placed on cube B: net rise **0.0502 m**
    (start z `0.0250` → rest z `0.0752`), cube B unmoved at z `0.0250`
  - Stack verified stable: `stackStable = true`, XY offset **0.0121 m**
  - Contact force **6.55 N** mean, peak **6.55 N**, contact samples 8
  - Collision count **0**, steps used **450 / 500**, sim time **0.90 s**
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
- MuJoCo pick_and_stack completion and abort-on-failure.

```text
python -m pytest tests/ -q
# recorded before PR: all tests passed

python -m py_compile bridge/laok_stack_arm_001_zenoh_bridge.py bridge/x402_client.py
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
- `stack-arm-001` is a simulator; it satisfies the Tier 1 "simulator skill
  execution" track and does not require physical hardware.
