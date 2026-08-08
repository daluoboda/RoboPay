# Secrets & Credential Policy

This repository follows the RoboPay secret-handling standard. It mirrors the
fabric payment-policy.yaml `secrets` block so reviewers see the same commitment
across all five Tier-1 bounty PRs.

## Storage

- `storage: environment-variables-only` — every private key, wallet, and
  facilitator URL is read from a process environment variable; never from a
  committed file, never from a YAML / JSON value, never from a Docker layer.
- `committedToRepo: false` — no `private_key`, `mnemonic`, `sk-*`, `sk-or-*`,
  hex-64 private key, or any other long-lived secret is ever present in this
  repository, its history, or its build artefacts.

## Required Variables

| Variable                    | Required | Purpose                                        |
|-----------------------------|----------|------------------------------------------------|
| `<ROBOT>_PRIVATE_KEY`       | for live settlement only | Robot wallet signing key (never logged) |
| `<ROBOT>_WALLET_ADDRESS`    | yes      | Public address of the robot wallet             |
| `<ROBOT>_PAYTO_ADDRESS`     | yes      | The payee address the 402 challenge quotes     |
| `X402_FACILITATOR_URL`      | yes      | The x402 facilitator endpoint                  |

## Redaction

- `logs: true` — secrets are never echoed to stdout, stderr, or CI logs.
- `resultMetrics: true` — settlement metrics carry no payment material; the
  `payer` / `payee` addresses that appear in evidence JSON are public on-chain
  data, not private material.

## Rotation

Any credential ever committed or leaked must be treated as compromised from the
commit timestamp, not from the discovery timestamp — rotate at the provider
first, then purge from history, then update the source.

## CI Gate

The `.github/workflows/secret-scan.yml` workflow runs **gitleaks** on every
push and pull request. A leaked high-confidence secret fails the build before
it can be merged to `main`.