# Public runtime files

A checkout of this branch includes the Foxglove installer and the inference and
playback assets needed by the public Runner. The published receive policy stays
at **model_21800**; private HOPE training changes and development checkpoints are
not included.

## Install the console

Download [hopeopen.hope-a3-console-1.8.8.foxe](extensions/hope-a3-console/hopeopen.hope-a3-console-1.8.8.foxe)
from this checkout, then install it through Foxglove Desktop's Extensions screen.
On GitHub, use the file's download button to retrieve the binary. The file is an
ordinary Git blob, so Git LFS is not needed for the console or inference ONNXs.

Import [model21800_console.json](layouts/model21800_console.json). The `.foxe`
already contains the JavaScript, fonts, package manifest and license notices;
you do not need Node.js, `node_modules/` or a local `dist/` to install it.

Install the matching Laptop/HDU/MDU helpers and services from the same checkout
using [Runtime operation](../docs/operations/runtime_xbox.md). The desktop panel
connects to the HDU control bridge at `ws://<HDU-IP>:8766`. Installing a panel does
not install the robot services or start the Runner.

## Included payloads

| Payload | Source in this repository | Consumer |
| --- | --- | --- |
| Console installer | `foxglove/extensions/hope-a3-console/hopeopen.hope-a3-console-1.8.8.foxe` | Foxglove Desktop |
| Layout | `foxglove/layouts/model21800_console.json` | Foxglove Desktop |
| Xbox input and helpers | `foxglove/laptop/`, `foxglove/helpers/` | Laptop ROS environment and lifecycle |
| Control-plane modules, units and configs | `foxglove/a3/` | HDU/MDU installation |
| Receive actor and parameters | `a3_deploy/a3_deploy_example/models/model_21800/policy/` | Native Runner |
| HumanLike actor, encoder and YAML | `a3_deploy/a3_deploy_example/assets/a3_runtime/teleop_humanlike/` | Native Xbox locomotion adapter |
| Six serve playback CSVs | `a3_deploy/a3_deploy_example/assets/a3_runtime/serve/motions/` | Serve controller and its probes |
| Gripper command definitions and bridge | `a3_deploy/a3_deploy_example/scripts/serve_gripper_presets.py`, `a3p_gripper_bridge.py` | Package tools |

The three HumanLike files are vendor-origin runtime assets and are stored apart
from HOPE's receive model. The native adapter opens exactly those three files;
the vendor YAML's unused `dataset_file` entry does not create a dependency on
`walk_data_all.csv` or on the vendor `motion_control` executable.

The default package builder now includes HumanLike automatically:

```bash
cd a3_deploy/a3_deploy_example
source /opt/ros/jazzy/setup.bash
bash scripts/build_a3_deploy_pkg.sh --arch x86_64
# With the documented Rockchip toolchain/sysroot available:
bash scripts/build_a3_deploy_pkg.sh --arch rockchip
```

The package contains `policy/`, `teleop_humanlike/`, `motions/`, `config/` and
`tools/`. The lifecycle passes the included `teleop_humanlike/` to Runner.
`--teleop-policy-dir PATH` selects another compatible bundle. `--without-teleop`
explicitly omits locomotion and removes stale teleop files when restaging.

## Verify a clone or source archive

From the repository root:

```bash
python3 foxglove/scripts/check_public_runtime.py
```

This standard-library-only check reads
[public-runtime-manifest.json](public-runtime-manifest.json), verifies file sizes
and SHA-256s, checks the installer's ZIP integrity and packaged font references,
and checks that its package metadata/readme/notices match the source. It also
checks the default model, HumanLike and serve files. It needs no ROS, npm or
hardware access and exits nonzero if a required file is missing or stale.
The `Public runtime files` GitHub Actions job runs this check on pull requests
and pushes to `main` or `public-runtime-xbox`.

The manifest records the public release asset set; it is not a hardware readiness
check or a new runtime admission condition. A successful file check does not
prove that services are installed, an Xbox is connected or physical motion is
safe. See [validation scope](../docs/operations/runtime_public_validation.md).

## Build dependencies and site-specific files

The repository supplies the source and runtime assets above. Native binaries
still need the documented x86 or ARM build environment: ROS 2 Jazzy, AimRT,
ONNX Runtime, Unitree SDK and the selected target sysroot. The setup/build scripts
fetch or consume these dependencies. AgiBot hardware transport/firmware comes
from the robot installation. These host/target dependencies are not bundled
inside the Foxglove extension.

A venue calibration receipt, network addresses and robot login settings must be
created for the deployment site. The public templates and calibration code are
included; private live receipts and credentials are not reusable installation
assets. Full Gate3 generates its own labelled simulation receipt as described in
[the Gate3 guide](../docs/MODEL_21800.md#what-the-current-gate-3-test-validates).

## Update a release

After modifying the console, update `package.json` and `package-lock.json` to the
same version, then run from the repository root:

```bash
npm --prefix foxglove/extensions/hope-a3-console ci
npm --prefix foxglove/extensions/hope-a3-console run package
python3 foxglove/scripts/check_public_runtime.py --write
python3 foxglove/scripts/check_public_runtime.py
```

Commit the new `.foxe`, source/lockfile changes, runtime asset changes and refreshed
manifest together. The `.gitignore` explicitly allows `.foxe` installers, including
when local Git excludes hide generic generated packages. Update the download
links when changing the release version. Do not upload an old private build in
place of the package built from this source.
