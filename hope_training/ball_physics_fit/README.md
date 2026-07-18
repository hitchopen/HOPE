# ball_physics_fit — no-spin ball-physics fitting

Fits the simplified, **no-spin** table-tennis ball model used by HOPE PingPong
from real ball-capture data, and writes the constants that go into
[`configs/ball_physics.yaml`](../../configs/ball_physics.yaml).

The model has two parts, both linear-velocity only (ball state `[x, y, z, vx, vy,
vz]`, no orientation / angular velocity / spin):

* **Flight** — quadratic aerodynamic drag: `a = g − k_d·|v|·v`.
* **Contact** — for the table top and the racket face, a normal restitution `e_n`
  plus a tangential grip/damping term (`a_t`, `b_t`, friction cap `mu`). See
  [`contact_model.py`](contact_model.py) for the exact equations.

The values shipped in `configs/ball_physics.yaml` came from a real capture of a
ball on a regulation table. **They are specific to that ball and table — re-fit
them for your own equipment** by running your capture through this pipeline and
copying the resulting numbers into the config.

## Data format

Everything downstream works on **canonical trajectory CSVs**: one file per
contiguous capture segment, with a `t,x,y,z` header, in SI units and the world
frame from the config (`+x` forward, `+y` left, `+z` up; `z = 0` at the table
surface). See [`sample_data/`](sample_data/) for a real example.

If your raw capture is in other units or is one long recording (e.g. absolute
timestamps and millimetres), convert it with `extract_canonical.py`, which
rebases time, scales to metres, splits on tracking gaps, and (optionally) shifts
`z` so the surface is at `z = 0`:

```bash
python extract_canonical.py RAW.csv sample_data/       # -> RAW_seg00.csv, ...
```

## Fit pipeline

Point `BALLFIT_DATA_ROOT` at a folder of canonical trajectory CSVs (it defaults
to the bundled `sample_data/`), then:

```bash
pip install -r requirements-ballfit.txt

python stage1_segments.py     # split trajectories into flights / bounces / strikes
python stage2_fits.py         # fit k_d, table e_n + tangential, paddle contact
```

`stage2_fits.py` writes `analysis/fits/stage2_fits.json`; copy the fitted values
into `configs/ball_physics.yaml`.

Optional checks (all read the stage outputs; PNGs are written only if matplotlib
is installed):

```bash
python flight_selfcheck.py           # front<->back flight self-consistency vs a noise floor
python predict_check.py              # forward flight-prediction accuracy vs horizon
python falsify/f1_kd_over_speed.py   # is k_d constant over launch speed?
python falsify/f3_table_e_vs_vn.py   # is table e_n constant over impact speed?
python falsify/f4_paddle_e_vs_un.py  # is paddle e constant over contact speed?
```

The `falsify/` checks return a PASS / KILL / INCONCLUSIVE verdict on the
constant-value assumptions, so you know when a single number is (or is not)
enough for your data.

## What each file does

| file | role |
|------|------|
| `paths.py`            | input (data root) and output (analysis) locations |
| `extract_canonical.py`| raw capture CSV → canonical `t,x,y,z` segment CSVs |
| `ballcore.py`         | load a trajectory, segment it into arcs/contacts, RK4 drag fit |
| `contact_model.py`    | the no-spin contact equations (table & racket) |
| `stage1_segments.py`  | trajectories → `flights.json`, `bounces.json`, `strikes.json` |
| `stage2_fits.py`      | ordered fits → `stage2_fits.json` (the values for the config) |
| `flight_selfcheck.py` | flight-model self-consistency vs a matched noise floor |
| `predict_check.py`    | forward flight-prediction accuracy vs horizon |
| `falsify/f1,f3,f4`    | constant-value sanity checks (drag, table e, paddle e) |

## Data requirements

* **Drag `k_d`** and **table restitution** (`e_n`, tangential) fit from **ball
  trajectory alone** — a ball-only capture with flight arcs and table bounces is
  enough. Use many arcs across a range of speeds; `k_d` is not identifiable from
  a single short arc.
* **Paddle restitution** needs the **racket** tracked too (its contact-point
  velocity and face normal at each strike): supply a paddle sidecar CSV
  `<name>.paddle.csv` (`t,x,y,z` and optionally `nx,ny,nz`) next to the ball
  file, and `stage1` will attach the paddle state to each strike. Without racket
  tracking, `stage2` fits flight + table and reports the paddle stage as skipped.

All internal state estimates use short windowed drag shooting fits rather than
raw finite differences, so they tolerate the position noise typical of optical
motion capture. Contact-detection thresholds in `ballcore.detect_contacts` may
need tuning for very slow or very noisy captures.
