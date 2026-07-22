# Validation report — agibot.x2.aimdk-robopay.v1

Submission scope: physical robot, Tier 2 (vendor-provided built-in skill)

Brand: Fabric Foundation × AgiBot

Robot/control stack: physical AgiBot X2, AimDK_X2 1.0.0, ROS 2 Humble,
`/aimdk_5Fmsgs/srv/SetMcPresetMotion`

Zenoh topics: `robot/tunnel/action` and `robot/tunnel/result`

## Claim boundary

Three evidence records are kept separate:

1. **Video-correlated historical field run:** the candidate/reconstructed video
   shows transaction `0x35fa38…605a0a` together with the AimDK admission
   `RUNNING`, `task_id=8`, and the physical X2 right-hand wave.
2. **Independent historical paid success:** transaction `0x4f46…1798e` is a
   separate successful Base Sepolia paid request. It is not claimed to appear
   in, or correlate with, the `task_id=8` video sequence.
3. **Current strict profile:** adds full envelope validation, persistent replay
   protection, structured asynchronous results, and settlement eligibility
   semantics. It has automated offline coverage but has not yet been deployed
   for a new physical full-flow run.

The video-correlated historical flow settled payment before robot completion
and therefore does not prove the current no-settle-on-failure rule. Both legacy
transactions are supporting evidence, not substitutes for the required rerun.

## Historical physical validation

- [x] Physical AgiBot X2 identified and prepared in Stable Standing Mode.
- [x] `x2_right_wave` mapped to AimDK area `2`, motion `1002`.
- [x] Unpaid request returned HTTP 402.
- [x] Paid request returned HTTP 200/accepted on Base Sepolia.
- [x] Verified payment amount was 0.002 USDC.
- [x] Zenoh action reached the robot-side adapter.
- [x] AimDK accepted the action and returned `task_id=8` with `RUNNING`.
- [x] Physical right-hand wave was visibly observed and recorded.
- [ ] Historical logs contain an explicit AimDK terminal `SUCCESS` result.
- [ ] Historical flow proves no-settle-on-failure.

Public identifiers are privacy-masked:

| Field | Redacted value |
| --- | --- |
| Video-correlated transaction | `0x35fa38…605a0a` |
| Independent successful transaction | `0x4f46…1798e` |
| Historical payer | `0x8c0c…912F` |
| Historical payee | `0x3F5a…3987b` |
| Robot identity | `agibot-x2-demo-***` |

Full receipt material is retained outside Git for controlled reviewer
verification. No private key, payment signature, hostname, username, internal
IP, or robot serial number is included in this package.

## Historical evidence manifest

| Evidence ID | Artifact | SHA-256 | Size | What it proves | Publication status |
| --- | --- | --- | ---: | --- | --- |
| `AGX2-HISTORICAL-PHYSICAL-01` | [`docs/evidence/agibot-x2-historical-physical-evidence-redacted.mp4`](evidence/agibot-x2-historical-physical-evidence-redacted.mp4) | `242620d1982bbd1a80778319f6433f49e9ca434e39b83d96ff0268d6856fb70f` | 2,194,472 bytes | Privacy-redacted physical wave associated with task 8 and `0x35fa38...605a0a` | Included; video-only, no audio or source metadata |
| `AGX2-LEGACY-SOURCE-01` | `FABRIC-AgiBot-X2-English-Narrated-Demo.mp4` | `1221d214d22dddea1f82f8c0e89fcea546208628eee8605213ad5a580fbc44eb` | 10,442,891 bytes | Source evidence used to derive the public physical-action clip | Kept outside Git because it contains private terminal and user data |
| `AGX2-LEGACY-TX-01` | Base Sepolia receipt `0x35fa38…605a0a` | masked | n/a | Payment receipt correlated with the task 8 video evidence | Full value withheld from public tree |
| `AGX2-LEGACY-TX-02` | Base Sepolia receipt `0x4f46…1798e` | masked | n/a | Independent historical paid success only; no task 8 video correlation claimed | Full value withheld from public tree |

The public derivative is deliberately limited to physical motion and a
non-sensitive evidence label. Its machine-readable privacy and traceability
record is in [`docs/evidence/evidence-manifest.yaml`](evidence/evidence-manifest.yaml).
It is correlated only with `0x35fa38…605a0a`; the report does not infer any
video relationship for `0x4f46…1798e`.

## Current automated validation

Run from this profile directory:

```bash
python3 -m unittest discover -s tests -p 'test_*.py' -v
```

Local result on 2026-07-22: **26/26 tests passed** with Python 3.12.13.

- [x] Canonical `paramsHash` matches the committed example.
- [x] Valid explicit success produces a correlated structured result.
- [x] `RUNNING` remains pending and is not settlement-eligible.
- [x] Dry-run results cannot authorize settlement.
- [x] Duplicate idempotency/action/payment-authorization IDs do not execute twice.
- [x] Replay protection persists across bridge restart.
- [x] Wrong robot, unknown skill, invalid params, and tampered hash do not actuate.
- [x] Unverified, mismatched, settled, or expired payment evidence does not actuate.
- [x] Authorization TTL above 300 seconds is rejected before actuation.
- [x] Issuance beyond the 30-second future-clock allowance is rejected.
- [x] Unsafe TTL/skew configuration above the hard caps is rejected.
- [x] Vendor error produces `status=error` and `settlementEligible=false`.
- [x] Audit output omits payee and payment authorization identifiers.
- [x] AimDK state normalization handles both direct integers and ROS `.value` wrappers.

These tests use a fake executor. They validate the contract and fail-closed
routing behavior, not physical motion or on-chain settlement.

## Required strict physical rerun

- [ ] Robot and skill discovery show profile, price, and physical scope.
- [ ] Unpaid request returns 402 and produces no Zenoh action.
- [ ] Paid request returns immediately as accepted/pending with `actionId`.
- [ ] Action envelope preserves every required correlation/payment field.
- [ ] Authorization is fresh, no longer than 300 seconds, and within clock tolerance.
- [ ] Current adapter receives the action and invokes AimDK exactly once.
- [ ] Explicit terminal completion is correlated on `robot/tunnel/result`.
- [ ] Physical wave is visible in a newly redacted Fabric-branded video.
- [ ] Duplicate after adapter restart causes no second motion.
- [ ] Intentional robot-unavailable/error case proves no settlement.
- [ ] Success settles only after the terminal success result.
- [ ] Robot `robotsdk` identity handshake is bound to the configured payee.
- [ ] Final evidence manifest records hashes, capture time, and redaction review.

Follow [field-validation-runbook.md](field-validation-runbook.md) for the safe
capture sequence.

## Known limitations

- AimDK documents `RUNNING` as an accepted/in-progress response but does not
  document a task-ID completion query for this preset service. The adapter
  therefore does not upgrade `RUNNING` to success using a timer or video alone.
- The shared repository tunnel must consume `robot/tunnel/result` and gate
  settlement before the new end-to-end test can pass.
- Physical e-stop remains an operator responsibility; no paid remote stop is
  exposed.
- Public proof is intentionally privacy-masked. A reviewer needing an
  unredacted receipt must use a controlled verification channel.

## Acceptance decision

The project is correctly classified as **Tier 2** and the historical real-robot
task is traceable. The code package is ready for review and integration testing,
but the latest RoboPay success criteria should remain unchecked until the
strict physical rerun above is completed.
