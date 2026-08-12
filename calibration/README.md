# Runtime calibration receipts

`p1_to_pelvis.json` is generated on the operator Laptop by the attended
ten-marker calibration workflow. It contains installation-specific geometry,
timestamps, sample counts, and audit evidence, so it is intentionally not
version-controlled.

Run the calibration procedure documented in
[`docs/operations/foxglove_first_hardware_test.md`](../docs/operations/foxglove_first_hardware_test.md)
before entering policy mode. Failed fits are written next to the active receipt
with a `.rejected.<UTC timestamp>.json` suffix for local diagnosis only.
