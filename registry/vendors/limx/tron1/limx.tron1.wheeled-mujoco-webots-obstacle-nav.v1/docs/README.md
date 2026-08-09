# Documentation index

- `../README.md` — setup, architecture and security contract
- `validation-report.md` — reproducible acceptance claims and commands
- `evidence/evidence-manifest.yaml` — evidence inventory and generation source
- `../examples/` — accepted public action envelopes

The validation report and manifest deliberately distinguish MuJoCo
actuator-level evidence from Webots task-level evidence. Neither document
claims that the Isaac Gym ONNX policy executes inside Webots.

Generated execution evidence belongs under `../artifacts/` and is uploaded by
CI. Private keys and other credentials must be provided only through the
runtime environment or GitHub Actions secrets.
