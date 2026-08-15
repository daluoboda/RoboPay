# Dobot CRA / CR3 Tier 1 validation report

Scope: **simulator-only**. The profile runs a bounded, policy-driven three-tag
inspection task with the pinned Dobot CR3 vendor URDF in MuJoCo and Webots.
It does not claim physical-robot validation.

## Validated skills

- `inspect_three_tags` — paid, online nearest-target selection plus
  damped-least-squares IK from measured Link6/tool-center state.
- `stop` — paid, explicit safe-stop path; it cannot fall through to inspection.

## Automated validation

The mandatory automated validation covers:

- real Go Tunnel build and contract suite;
- fail-closed payment regression: a paid-shaped request rejected by the
  facilitator returns `402`, publishes zero ActionEvents, invokes zero
  simulator actions, and makes zero settlement calls;
- the submitted WebSocket reader reassembles continuation frames, while the
  live runner waits for the bridge-ready event before its first paid action;
- failure, timeout, idempotency-key replay, and payment replay tests: zero
  settlement on every unsuccessful path;
- real MuJoCo CR3 inspection plus a real in-process Zenoh action/result path;
- real Webots/MuJoCo Sim-to-Sim validation of the identical three-tag course.

## Current-HEAD live and visual evidence (2026-08-15)

The continuous split-screen recording linked from
[`evidence-manifest.yaml`](evidence/evidence-manifest.yaml) executes source
commit `39c1814edfe938c148b3fc59fd8ce593fb50dbe1`. It keeps the terminal and CR3
MuJoCo viewer simultaneously visible for action
`dobot-cr3-inspection-1786798884`: unpaid HTTP 402 precedes actuation, the
first paid request returns HTTP 202, the measured tool visits violet, amber and
cyan tags, and the viewer closes automatically after the documented final
hold. The same continuous recording then shows the correlated successful
result, settlement only after success, and the matching BaseScan transaction
`0x2ac0922fda4130fe1ac67834e038c84104220b729df4ac2c95d31fe1606a2baa`.

The corresponding trusted machine-readable result is versioned at
[`base_sepolia_result_1786798911.json`](evidence/base_sepolia_result_1786798911.json).
Its commit, action ID, transaction hash, JSON SHA-256 and recording SHA-256 are
bound in the evidence manifest.

The pre-registration visual runner uses a local HTTP/WebSocket proxy only to
expose the real Go Tunnel. The proxy does not verify or settle payments,
fabricate ActionEvents/results, or simulate the robot; payment remains in the
real Tunnel/public-facilitator path and execution crosses real Zenoh into the
measured MuJoCo bridge.

The local reproducible commands are in the [profile README](../README.md).

## Identity boundary

The bridge does not invent an EIP signing protocol. Private keys are supplied
only by environment/secret storage and never committed or logged. Signed
robot-to-payee binding remains a shared Tunnel/Gateway protocol dependency;
the current profile documents that limitation explicitly while requiring the
post-verification, payment-correlated Tunnel envelope.
