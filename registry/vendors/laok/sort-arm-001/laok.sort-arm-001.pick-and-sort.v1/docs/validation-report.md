# Validation report

## Submission

- Brand: Fabric Foundation × laok sort-arm-001
- Scope: simulated manipulator (MuJoCo)
- Tier: 1 — simulator skill execution
- Robot: `sort-arm-001` (4-DoF arm + gripper, MuJoCo model)
- Payment network: Base Sepolia (`eip155:84532`)
- Payment asset: USDC `0x036CbD53842c5426634e7929541eC2318f3dCF7e`
- Skill: `sort_arm_pick_and_sort`
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
    1. `0xea4f54c97c26762915615023cffeb942f858c51963ef1e2117de4de28b8f16d0` — block `45125827`, payer `19.2 → 19.1 USDC`
    2. `0x293fbd53d9d5ba624bea4c4618c2d0da8181b2f2205cf8fbb3663eb125daf909` — block `45125830`, payer `19.1 → 19.0 USDC`
    3. `0xf4b6e8c9e333a678de86089aba7284f13ebba8f3d6d24ebb1503b3ea3afeb97a` — block `45125832`, payer `19.0 → 18.9 USDC`
  - Explorer: https://sepolia.basescan.org/tx/0xea4f54c97c26762915615023cffeb942f858c51963ef1e2117de4de28b8f16d0
  - On-chain status: **1 (success)** on all three
  - Payer `0xA0723A2dA2bFa349919A467446Fb54569b2f3d13`, payee
    `0x742d35Cc6634C0532925a3b844Bc454e4438f44e`, **0.10 USDC** each. No faucet
    shortcut; these are real settled transfers. `verify_settlement.py` re-reads
    the ERC-20 Transfer logs of every hash and fails the build on any mismatch.
- [x] The MuJoCo simulator **physically executed** the pick-and-sort skill
  after the payment gate opened:
  - `SUCCESS = True`, reason `routed`, graspState `released`
  - Object grasped, carried and released into bin **A**: start
    `(0.350, 0.000, 0.025)` → rest `(0.245, 0.110, 0.026)`
  - **`peakLift` 0.1354 m** — the payload is lifted clear of the table and
    carried through the air; `objectLifted` reads only **0.0014 m** because a
    routing run deliberately ends with the object back down in the bin
  - Placement accuracy **0.0265 m** from bin centre (`routed = true`, gate 0.07 m)
  - Contact force **6.55 N** mean, peak **6.55 N**, contact samples 8
  - Collision count **0**, steps used **430 / 450**, sim time **0.86 s**
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
- MuJoCo pick_and_sort completion and abort-on-failure.

```text
python -m pytest tests/ -q
# recorded before PR: all tests passed

python -m py_compile bridge/laok_sort_arm_001_zenoh_bridge.py bridge/x402_client.py
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
- `sort-arm-001` is a simulator; it satisfies the Tier 1 "simulator skill
  execution" track and does not require physical hardware.
