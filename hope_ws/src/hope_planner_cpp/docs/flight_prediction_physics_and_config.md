# Forward physics, effective configuration and uncertainty

Profile: `thin-shell-forward-v3-receipt-20261010`. Coefficients retain the public
2026-10-10 calibration receipt. Position-only reuse of these coefficients is a
provisional effective p/v model, not a separately validated spin-free calibration.

```text
p_dot = v
v_dot = g - kd ||v|| v + kM (omega × v)
u = (vx - R omega_y, vy + R omega_x, 0)
j = min(kt ||u||, mu (1+e) |vz_minus|)
dv_xy = -j u_xy / ||u||             (zero if ||u|| = 0)
v_plus = (v_minus_xy + dv_xy, -e vz_minus)
omega_plus = omega_minus + (dv_y, -dv_x, 0)/(kappa R)
```

| Parameter | Value | Units/provenance |
|---|---:|---|
| kd | 0.12861635416914574 | acceleration-law drag; no extra mass division |
| kM | 0.00379260504480258 | acceleration-law Magnus |
| e | 0.9313775047019153 | normal restitution |
| kt | 0.38744443855065164 | tangential slip gain, constrained [0,0.4] |
| mu | 0.16706204822993975 | Coulomb cap, nonnegative |
| R | 0.020 | m; fixed ball-radius prior |
| g | (0,0,−9.81) | m/s²; fixed prior |
| kappa | 2/3 | thin-shell inertia factor |

`POSITION_ONLY` uses gravity/drag and capped p/v tangential reduction with no
angular state and no Magnus. A raw zero vector means unavailable producer spin;
a zero produced by a trusted contact is usable. Constant free-flight angular
velocity is an explicit prior. It is labelled HELD after positive elapsed time.

All common/V2 validators require finite coefficients, positive radius,
nonnegative drag/Magnus/friction, restitution in [0,1], and kt in [0,0.4]. The
legacy inverse planner is unchanged and is not the owner of this contact law.

## Numerical and geometric contract

RK4 free flight and root-refined earliest event integration are shared by CKF
mean/sigma members, nominal mean/covariance, piecewise batch fitting, receiving
fallback and V2 replay. Table center contact is z=R on the actual top footprint;
edge ambiguity is separate. Net geometry is finite and radius-expanded. Collision
wins a quantized 1 ns tie; the remaining substep is propagated once after contact.
Apex is the vz root. Horizons, contacts, steps, raw buffers and pending work are
bounded; no initial below-table clamp or zero-duration impulse loop is allowed.

Independent DOP853 and analytic roots use 1/0.5/0.25 ms refinement with engineering
budgets 1 mm / 0.5 ms. These measure solver discrepancy, not real-ball error.
Independent energy, derivative, event covariance, PSD/symmetry and branch-quality
checks are distinct from solver agreement.

## Effective configuration identity and replay

`flight_model_config_id()` hashes the fixed-order, classic-locale hexadecimal
representation of the physical/CKF/geometry/shot settings and profile version.
Runtime and V2 replay share that function and parameter validation. Node
`config_id` additionally identifies plane policy/registration timing and output
cadence; moving-plane geometry has its own revision and source stamp. Forecast
and packet schema revision 3 exposes both the model ID and runtime configuration.
Packet restoration checks model identity and the CKF/noise fingerprint.

```bash
hope_flight_prediction_v2_replay --profile
hope_flight_prediction_v2_replay --profile --set physics.drag_k=0.15
hope_flight_prediction_v2_replay --observations -1 complete \
  --set integration_dt_s=0.001 --set horizon_s=1.5 --set x_hit=0.15 < observations.csv
```

Supported overrides include physics drag/Magnus/restitution/slip/friction/radius,
gravity XYZ, table length/width/Y/net geometry, integration/output dt, horizon,
receiving X and maximum source age. Unknown/nonfinite/unsafe overrides fail
explicitly. CSV uses exact signed int64 acquisition ns and raw XYZ/angular validity;
rejected angular NaNs are passed to the resolver without erasing valid XYZ.

`hope_planner_cpp_replay` is explicitly labelled **legacy planner replay**. It
retains command-related legacy behavior and is not a V2 forecaster. Its help now
matches its actual defaults. `hope_ball_flight_baseline_replay` instead exports
ball-only progressive/fixed-one-shot snapshots using the unchanged e73b312 batch,
incoming segmentation and fixed trigger, with shared forward model and separately
identified method configuration. It never invokes inverse racket planning.
One-shot exports only its real trigger epochs; replay does not manufacture timer
or intermediate predictions after a truncated input stream.

## Conditional uncertainty

The six-state covariance and seven-state event p/v/time covariance are conditional
local linearizations. Source clock uncertainty is carried separately. Missing
auxiliary spin/model, plane registration/motion and clock uncertainty are not a
calibrated full error distribution. Branch-sensitive or ill-conditioned events
are marked explicitly; unavailable covariance is NaN/labelled unavailable on ROS
and null in JSON. Do not use a small conditional covariance to prove legality.
Only a separate conservative complete bound may support hard predicted rejection.

Adoption requires independent registered recordings, separately assessed held,
contact-propagated and position-only histories, and held-out coverage/error tests.
No physical improvement or confidence calibration is established by numerical parity.
