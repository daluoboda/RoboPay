# Dobot CRA / CR3 Tier 1 validation report

Scope: **simulator-only**. The profile runs a bounded, policy-driven three-tag
inspection task with the pinned Dobot CR3 vendor URDF in MuJoCo and Webots.
It does not claim physical-robot validation.

## Validated skills

- `inspect_three_tags` — paid, online nearest-target selection plus
  damped-least-squares IK from measured Link6/tool-center state.
- `stop` — paid, explicit safe-stop path; it cannot fall through to inspection.

## Automated validation

The Dobot Tier 1 GitHub Actions run for this revision passed all jobs:

- real Go Tunnel build and contract suite;
- fail-closed payment regression: a paid-shaped request rejected by the
  facilitator returns `402`, publishes zero ActionEvents, invokes zero
  simulator actions, and makes zero settlement calls;
- failure, timeout, idempotency-key replay, and payment replay tests: zero
  settlement on every unsuccessful path;
- real MuJoCo CR3 inspection plus a real in-process Zenoh action/result path;
- real Webots/MuJoCo Sim-to-Sim validation of the identical three-tag course;
- Base Sepolia x402 proof using repository Secrets, with the generated
  receipt/result JSON uploaded as a CI artifact.

The local reproducible commands are in the [profile README](../README.md).
The screen recording attached to the pull request shows discovery, unpaid
`402`, paid `202`, Zenoh-mediated execution in MuJoCo, terminal success, and
the Base Sepolia settlement transaction.

## Identity boundary

The bridge does not invent an EIP signing protocol. Private keys are supplied
only by environment/secret storage and never committed or logged. Signed
robot-to-payee binding remains a shared Tunnel/Gateway protocol dependency;
the current profile documents that limitation explicitly while requiring the
post-verification, payment-correlated Tunnel envelope.
