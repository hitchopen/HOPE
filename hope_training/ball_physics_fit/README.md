# ball_physics_fit — venue ball-physics fitting pipeline

Fits the flight + contact model of `docs/ball_physics_low_speed_validation.md`
(Sony-Ace family: `a = g − k_d|v|v + k_m(ω×v)`, angle-dependent tangential-impulse
contacts) on the 2026-07-03 venue mocap dataset (Avatar Pro, 300 Hz, C3D exports:
15-marker ball rigid body + two 4-marker paddles + table markers). This replaces the
Mac-local-only `~/Desktop/Hope/Record` workspace as the fitting toolchain of record.

Deliverables produced by this pipeline live at:
- `configs/ball_physics_venue.yaml` — fitted constants with provenance + validity envelope
- `docs/ball_physics_fit_report.md` — full FIT_REPORT (stages, F1–F8 falsification verdicts,
  held-out metrics)

## Data layout

Point `BALLFIT_DATA_ROOT` at the venue recording folder (REQUIRED — the venue
recordings are not shipped with this repository; any `.tak` files in that
folder are raw Avatar projects the pipeline never reads). Expected tree:

```
$BALLFIT_DATA_ROOT/
  <take folders with .c3d exports>       # raw (not required once extracted)
  analysis/
    extracted/<take>.npz                 # extract_canonical.py output
    qa_stage0.json                       # qa_stage0.py output
    segments/{flights,bounces,strikes,meta}.json   # stage1_segments.py output
    fits/stage2_fits_{all,train}.json    # stage2_fits.py output
    fits/stage4_validation.json          # validate_stage4.py output
    falsification/F*_verdict.json + .png # F1–F8 battery
```

## Environment

This offline fitting tool uses an isolated host-Python virtual environment; it
does not run in the Isaac `grasping` or ROS `hope` Distrobox. On a new
workstation, complete the host prerequisites in
[`docs/DISTROBOX_SETUP.md`](../../docs/DISTROBOX_SETUP.md), then run:

```bash
cd "$HOME/workspace/HOPE/hope_training/ball_physics_fit"
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements-ballfit.txt
```

NumPy, SciPy and Matplotlib are always used. The `c3d` reader is needed only
for `extract_canonical.py`; everything downstream runs from the extracted NPZ.

## Pipeline (run in order)

```bash
python extract_canonical.py <take_dir_or_c3d> analysis/extracted   # per take (needs c3d)
python qa_stage0.py           # Stage 0: sampling/units/gravity gates — STOP if it fails
python stage1_segments.py     # Stage 1: flights / table bounces / racket strikes
python stage2_fits.py --split all     # Stage 2: k_d → k_m → table e → table tan → paddle
python stage3_falsify.py              # Stage 3: F1–F8 battery + adversarial verifiers (falsify/)
python stage2_fits.py --split train   # for held-out validation
python validate_stage4.py             # Stage 4 vs the train-split fit (test split)
python validate_stage4.py --yaml ../../configs/ball_physics_venue.yaml --paddle-e exp
                                      # Stage 4 vs THE SHIPPED YAML — the honest acceptance numbers
python predict_check.py --yaml ../../configs/ball_physics_venue.yaml --split all
                                      # two-horizon landing check: H0 at-contact through paddle /
                                      # H1 at-contact measured-out / H2 ~100 ms before landing
python predict_check.py --yaml ../../configs/ball_physics_venue.yaml --split all --full-state
                                      # FULL-STATE validation: position error vs horizon bins,
                                      # velocity/spin at 25/50/75% arc checkpoints, net-plane (x=0)
                                      # crossing state. Adds H1q = H1 velocity with spin re-estimated
                                      # from the FIRST 100 ms of flight quats (deploy-realistic spin
                                      # source; the strike w_out quat channel reads ~0.22x = junk).
                                      # Optional --magnus sat (saturating C_L form).
python falsify/f10_paddle_split.py    # per-paddle / per-face / blade-position splits of the racket
                                      # contact model (answers 正反面/拍位/双拍 questions; needs
                                      # extracted npz for face identity — pad_n in strikes.json is
                                      # flipped toward approach, so face comes from the raw body
                                      # normal channel). Venue verdicts: paddles DIFFERENT (p2's
                                      # e falls much faster with u_n), face/blade UNDERPOWERED.
python flight_selfcheck.py --yaml ../../configs/ball_physics_venue.yaml
                                      # front-window -> back-window (and reverse) self-consistency
                                      # on ballistic arcs + matched MC noise floor: decomposes
                                      # prediction error into variance (noise) vs model-form bias.
                                      # Venue: ~half/half in quadrature; excess 42-62 mm.
python test_oracle_present.py # loud-fail oracle check (never skips)
```

Landing ground truth everywhere is the OBSERVED first-bounce contact point `p_c`
from `bounces.json` under a continuity gate (measured post-strike state must land
within 0.30 m / 80 ms of the recorded bounce). The legacy label (integrating the
measured out-state through the same flight model) is reported separately as
`landing_reconstructed_label` — it shares the flight model with the prediction and
reads optimistic, so it is never the headline.

## Conventions

- Table frame: origin = table center, X = length (2.740), Y = width (1.525), Z = up;
  the playing surface sits at `meta.surface_z` (≈ −14 mm: corner markers stand ~14 mm
  proud of the surface). Ball-center at table contact = surface_z + 0.020.
- SI units throughout; spin = rad/s expressed in the table frame
  (`spin_from_quats(..., R_table=take["table_R"])` — quats are template→world).
- The venue ball is coated for mocap: m = 3.4 g (clean ITTF ball 2.70 g). Fitted k_d/k_m
  are acceleration coefficients for THIS ball; scale by m_taped/0.0027 for a clean ball.
- The 15 exported ball "markers" are Avatar-Pro solved-model points, NOT physical
  sphere-surface positions (max pairwise span 56 mm > ball diameter). Treat the centroid
  as a rigidly-attached virtual point; the true-center offset is handled dynamically
  (QA wobble check), not geometrically. KNOWN SYSTEMATIC (forensics
  `g1_wobble_delta_verdict`): the sphere-fit offset applied by the extractor is wrong by
  ~1.9 mm (35° direction error); the residual wobble is coherent but sits ~0.1 mm-rms
  under the 9 mm venue noise floor (k_d bias ≈ 0, k_m shift < 3%), so it is documented
  rather than re-extracted. On a cleaner rig, fold the correction
  (δ_common ≈ [−1.5, −0.4, −1.1] mm in the shared template frame) into extract_canonical.
- Never finite-difference accelerations for fitting — RK4 shooting fits only
  (`ballcore.fit_arcs_global`), g frozen at 9.81.

## Gotchas learned on this dataset

- Contacts are usually OCCLUDED (racket blocks cameras) → the contact falls between
  tracked runs. Pair arcs ACROSS gaps (extrapolate both sides to a meeting point);
  in-run contact detection alone finds almost nothing.
- Parabola-g arc gates must be wide ([5, 16]) or heavy-topspin arcs get systematically
  excluded (vertical Magnus biases apparent g) — which would blind the falsification tests.
- Quaternion spin is trustworthy below ~75 rev/s at 300 Hz; its SCALE was cross-validated
  aerodynamically (venue k_m 0.0044 inside the old rig's CI [0.0035, 0.0049]).
- Venue position noise ≈ 9 mm shooting-fit RMS (old OptiTrack rig: 0.4 mm) — widen
  outlier tolerances accordingly; contact windows are ≤25 frames with ±3/±5-frame
  exclusion zones at table/racket contacts.
