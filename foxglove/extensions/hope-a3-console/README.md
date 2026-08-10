# HOPE A3 Console

Foxglove extension implementing the operator-console design from
`Foxglove+Desktop+UI+improvements/design_handoff_a3_console` against the
native model_21800 Runner contract.

The panel uses one opt-in data source, `ws://<HDU-IP>:8766`. It never publishes
topics or mutates parameters. Buttons call only the explicit Trigger services
listed in `foxglove/v17/a3/bridge_params_v17_control.yaml`.

```bash
npm install
npm run lint
npm run package
```

Install the generated `.foxe` through Foxglove Desktop's Extensions screen,
then import `foxglove/layouts/v17_model21800_console.json`.

The recommended receiver sequence shown by the UI is Stand, Calibration
(`x_hit` refresh), then Ready. Calibration remains telemetry and is not a new
Runner admission gate. Ready to Serve and Serve remain disabled unless the
authoritative Runner reports a loaded serve controller and the appropriate
serve phase.
