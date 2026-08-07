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
