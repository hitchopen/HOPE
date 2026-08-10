# HOPE A3 Console extension

Laptop-side Foxglove custom panel for the HOPE A3 match-day monitor. It
implements the visual design in the UI handoff without adding robot behavior.

The panel subscribes only to the monitor topics already documented in
`foxglove/README.md` and can call only the existing assert-only
`/hope/safety/trigger_estop` service. The sequence area is intentionally empty:
the proposed sequence services do not exist and are not exposed by the bridge.
The HDU and MDU tiles are visual `NO DATA` placeholders; the panel does not
subscribe to the proposed split-state topics. The proposed pelvis scene label
is also omitted, leaving the stock 3D configuration unchanged.

## Build and install locally

Node.js 18 or newer and npm are required. From this directory:

```bash
npm install
npm run build
npm run local-install
```

Restart or reload Foxglove Desktop, confirm that **HOPE A3 Console** appears in
Settings → Extensions, then import `foxglove/layouts/a3_monitor.json`.

Local extension installation may require a Foxglove developer seat. To create
a shareable artifact instead of installing directly:

```bash
npm run package
```

This generates a `.foxe` package in this directory. Build outputs, local
packages, and `node_modules/` are intentionally ignored by Git.

## Safety boundary

The E-stop button is disabled until `/hope/safety/estop_ready` is fresh and
true. It calls the existing service with `{}`, enforces a three-second UI
timeout, rejects re-entry, and treats every response without `success: true` as
failure. There is no reset or release control.
