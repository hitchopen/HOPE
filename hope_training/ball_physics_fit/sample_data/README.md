# sample_data

One short, real ball-capture segment so the fitting pipeline runs out of the box.

## `sample_trajectory.csv`

A single captured ball trajectory in the **canonical** format the pipeline
consumes. 25 samples at roughly 45 Hz (~0.6 s). It shows a ball descending into
a **table bounce** near `t ≈ 0.07 s` and then a clean outgoing **flight arc**
rising to its apex and falling again — i.e. both physics ingredients the fit
cares about (drag in flight, restitution at the surface).

### Columns

| column | units | meaning |
|--------|-------|---------|
| `t` | seconds | time from the start of the segment (starts at 0) |
| `x` | metres  | world +x, forward toward the opponent |
| `y` | metres  | world +y, left |
| `z` | metres  | world +z, up; `z = 0` is (approximately) the table surface |

Frame and units match `configs/ball_physics.yaml` (SI; right-handed `+x`
forward, `+y` left, `+z` up). At the bounce the measured centroid dips a few cm
below `z = 0`; that is ordinary capture calibration/noise around the surface, not
a below-table position.

### Running on it

```bash
cd ..            # hope_training/ball_physics_fit
python stage1_segments.py     # detects the flight arc and the table bounce
python stage2_fits.py         # fits k_d on the flight arc
```

This one short segment is only a **format + smoke-test** example: it exercises
loading, segmentation, and the flight drag fit, but a single arc cannot pin down
`k_d`, and one edge-of-capture bounce is not enough to fit restitution. A real
re-fit needs **many** trajectories spanning a range of speeds (and racket
tracking for the paddle contact) — see the top-level `README.md`. Replace this
file with your own capture (or a folder of them via `BALLFIT_DATA_ROOT`).
