# Dobot CRA / CR3 profile documentation

The complete clean-checkout runbook, model provenance, Zenoh action/result
schemas, simulator commands, x402/Base Sepolia setup, and identity boundary
are documented in the [profile README](../README.md).

The public action examples live under [`../examples/`](../examples/). They
show the action body only: clients first request the same action without a
payment header to receive `402 PAYMENT-REQUIRED`, then submit the returned
x402 payment signature with the otherwise identical body.

Generated simulator and Base Sepolia artifacts are deliberately ignored by
Git. CI uploads them from the passing run, and the pull request carries the
screen recording for reviewer playback.

## Reproducibility and troubleshooting

From a clean checkout, install the profile bridge requirements, expose
`bridge/` on `PYTHONPATH`, and use the commands in the profile README to run
MuJoCo, Webots, or the non-interactive Sim-to-Sim validator. The bridge creates
the two local Zenoh endpoints itself: it publishes verified work to
`robot/tunnel/action`, and publishes its correlated terminal results to
`robot/tunnel/result`. A separate Zenoh router is optional for this local
single-host profile; when one is used, configure the Tunnel and bridge to join
the same router/session.

- **`ModuleNotFoundError: dobot_cra_sim`** — set `PYTHONPATH` to the profile's
  `bridge/` directory before invoking pytest or a bridge command.
- **Webots does not start or yields no result JSON** — set `WEBOTS_EXE` to the
  installed Webots executable, then rerun `run_sim2sim_validation.py`.
- **A live request stays pending** — keep the bridge running until its ready
  event is observed and retry only with the *same* idempotency key; do not make
  a second paid action while the first one is unresolved.
- **An action is rejected** — verify the exact configured robot ID, skill ID,
  canonical parameters, and that the request was first challenged with `402`
  before attaching its x402 payment header. Unknown or uncorrelated actions
  are intentionally rejected and are never published to Zenoh.
