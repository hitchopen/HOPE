# PPQ004 three-star ball: complete trajectories and calibration

The three original C3D exports, all three source-time replay CSVs, and the complete
extracted NPZ records are included as lossless gzip files. `manifest.json` records
both archive hashes and the byte-exact uncompressed hashes. The C3D hashes match
the original fit receipt. No installation backups, host processes or robot logs
are needed to use this dataset. The original Motive project ULB is not required.

Coordinates retain the original table-corner world frame; positions are metres,
time is seconds, and the recording rate is 300 Hz. The original table-corner
world frame is preserved; no table-centre transform is applied.
`trajectories/part*.csv.gz` contain all replay rows, including original gaps;
`fit/part*/raw_extracted.npz.gz` additionally preserve raw export/fit inputs.
The three takes have 65,535 / 65,535 / 15,861 frames. Files 1+2 train; file 3 is
held out: 246/27 flight arcs and 24 causally scoreable holdout arcs.

This PR contains model parameters, measured trajectories, raw recordings and
archived fit results only. Runtime selection, UI, fitting and verification tools
are submitted separately on `codex/ball-model-software-20261021`.
Each `.gz` can be decompressed with standard gzip; compare compressed and
uncompressed SHA-256 values against `manifest.json` before analysis.

Archived JSON results and `validation.png` document the original run. Flight
holdout uses training-only coefficients; the deployable profile uses the final
all-data fit. The third file has only three qualified spin/contact holdouts;
the contact comparison is descriptive, not qualified. Speed-binned gains did not repeat reliably across time;
keep unified parameters. Hardware hitting and Gate3 remain unproven.

The portable replay CSVs are measured source trajectories, not an assertion of
network/policy/hardware performance. The historical 405 ms packetizer replay is
reported in the operations report and is separate from physical holdout scoring.
