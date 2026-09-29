# HOPE A3 Console

Download `hopeopen.hope-a3-console-1.8.9.foxe` from this directory and open it in
Foxglove Desktop's Extensions screen to install the console. No Node.js build is
needed for this prebuilt release. Import
[`model21800_console.json`](../../layouts/model21800_console.json) and connect to
`ws://<HDU-IP>:8766` after installing the matching HDU/Laptop services.

The public console supports Xbox A/B/X/Y mode actions, LT movement, LB+RB software
E-stop and software reset; Normal Play and Kernel Mode; and the public 24-sticker
calibration plus separate legacy V2/V3 profiles. It calls the dedicated services
in `bridge_params_control.yaml`; Runner remains the body-command owner.

Install the matching control-plane helpers, configs and service units from this
same checkout. See [Runtime operation](../../../docs/operations/runtime_xbox.md)
for the complete setup. A Foxglove panel installation alone does not install
robot-side services or start motion.

To rebuild the installer from the public source:

```bash
npm ci
npm run package
```

`npm run package` performs the production build and packages the JavaScript,
fonts, manifest and notices into the `.foxe`. `dist/` and `node_modules/` are
build inputs/outputs; the ready-to-install `.foxe` is committed as an ordinary
Git blob. When changing console source, regenerate the installer from that source.
When changing the release version, update the exact filename allowed by this
directory's `.gitignore` and the download links, and remove the superseded installer.
