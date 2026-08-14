# HOPE reference motions

`preprocessed/hope_forehand.npz` and `preprocessed/hope_backhand.npz` are the
complete reference pair used by the published HOPE training recipe. They were
produced from HOPE project staged recordings and retargeted to the Agibot A3;
the source videos and license-gated human body-model inputs are not included.

The two processed motion artifacts and their sidecars are released by the HOPE
contributors under the repository's Apache-2.0 license. Their stable public
filenames deliberately carry no internal experiment or revision suffix.

Each adjacent YAML sidecar records the frames, joint order, body schema, phase
information, and racket convention. The file format and replacement workflow are documented in
[`../../docs/REPLACE_MOTIONS.md`](../../docs/REPLACE_MOTIONS.md).
