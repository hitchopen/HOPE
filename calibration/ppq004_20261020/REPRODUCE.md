# PPQ004 three-star ball: complete trajectories and calibration

The three original C3D exports, all three source-time replay CSVs, and the complete
extracted NPZ records are included as lossless gzip files. `manifest.json` records
both archive hashes and the byte-exact uncompressed hashes. The C3D hashes match
the original fit receipt. No installation backups, host processes or robot logs
are needed to use this dataset. The original Motive project ULB is not required.

Coordinates retain the original table-corner world frame; positions are metres,
time is seconds, and the recording rate is 300 Hz. Do not run the legacy
`extract_canonical.py` entrypoint: it uses a different table-centre convention.
PPQ004's `extract_ppq_c3d.py` imports only its rigid-pose/sphere helpers.
`trajectories/part*.csv.gz` contain all replay rows, including original gaps;
`fit/part*/raw_extracted.npz.gz` additionally preserve raw export/fit inputs.
The three takes have 65,535 / 65,535 / 15,861 frames. Files 1+2 train; file 3 is
held out: 246/27 flight arcs and 24 causally scoreable holdout arcs.

From the repository root, using Python with `requirements.txt` installed and g++:

```bash
python calibration/ppq004_20261020/verify_dataset.py --unpack /tmp/ppq004-reproduce
python calibration/ppq004_20261020/tools/score_holdout.py --fit /tmp/ppq004-reproduce/fit
# Re-extract from raw C3D when auditing input conversion:
python calibration/ppq004_20261020/tools/extract_ppq_c3d.py /tmp/ppq004-reproduce/raw/ppq_004_1.c3d --output /tmp/ppq004-fresh/part1
# Repeat extraction for parts 2 and 3, then refit (CPU-intensive):
python calibration/ppq004_20261020/tools/refit_ppq_multibounce.py --output /tmp/ppq004-fresh --baseline calibration/ppq004_20261020/fit/baseline_ppq002.json
PPQ004_FIT_DIR=/tmp/ppq004-fresh python calibration/ppq004_20261020/tools/validate_speed.py
PPQ004_FIT_DIR=/tmp/ppq004-fresh python calibration/ppq004_20261020/tools/speed_forward_check.py
```

`--unpack` requires a new destination to avoid replacing an existing experiment.
Archived JSON results and `validation.png` document the original run. Flight
holdout uses training-only coefficients; the deployable profile uses the final
all-data fit. The third file has only three qualified spin/contact holdouts;
the contact comparison is descriptive, not qualified. The fit script explicitly
prints that limitation. Speed-binned gains did not repeat reliably across time;
keep unified parameters. Hardware hitting and Gate3 remain unproven.

The portable replay CSVs are measured source trajectories, not an assertion of
network/policy/hardware performance. The historical 405 ms packetizer replay is
reported in the operations report and is separate from physical holdout scoring.
