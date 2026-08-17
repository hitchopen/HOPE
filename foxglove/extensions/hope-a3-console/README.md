# HOPE A3 Console

Foxglove extension implementing the attended operator console against the
native model_21800 Runner contract.

The panel uses one opt-in data source, `ws://<HDU-IP>:8766`. It never publishes
topics and cannot access generic parameters. Buttons call only the explicit
services listed in `foxglove/a3/bridge_params_control.yaml`. The
lifecycle configuration call uses a `SetParameters`-shaped request only so the
four named IPv4 strings can be transported; the dedicated server rejects every
other name/type and accepts confirmation only while stopped.

```bash
npm install
npm run lint
npm run package
```

Install the generated `.foxe` through Foxglove Desktop's Extensions screen,
then import `foxglove/layouts/model21800_console.json`.

The recommended receiver sequence shown by the UI is Stand, Calibration,
Refresh x_hit, then Ready. Calibration captures the ten physical P1 markers,
replaces the approved `P1 -> pelvis_link` receipt, stores the derived stationary
`world -> pelvis_link` audit snapshot in the same JSON, and waits for the base
relay to publish a fresh matching packet. It does not refresh Planner x_hit;
the separate `Refresh x_hit` button uses the existing atomic Planner contract.
Neither operation becomes a hidden Runner MOTION admission gate. Ready to Serve
and Serve remain disabled unless the authoritative Runner reports a loaded
serve controller and the appropriate serve phase.

The CPU card reports aggregate HDU load and the process with the largest CPU
delta. E-stop is assert-only: the panel follows the persistent HDU latch and
does not expose a reset operation.

The system-lifecycle card is backed by the separately installed HDU supervisor
documented in `docs/operations/foxglove_lifecycle.md`. `START SYSTEM`
replaces runbook STEP 0/1/2A/2B/4/5 and leaves the Runner in PASSIVE; it does
not replace physical robot support or access to the hardware E-stop.

`TIME CALIBRATION` is a separate attended maintenance action for runbook 10.4.
It is enabled only with confirmed network configuration, a stopped lifecycle,
and a fresh failing NTP gate. The fixed root coordinator stops MDU consumers
before HDU consumers, performs at most one hard-step per maintenance cycle,
restores in dependency order, and publishes persistent status after the expected temporary
8766 disconnect. It shares an exclusive operation interlock with `START
SYSTEM`; the panel exposes no clock parameters or shell input.
