# Physical assumptions and configuration

V2 uses one forward RK4 model for drag/gravity/optional Magnus and thin-shell capped table slip; event localization includes actual top footprint, radius-aware descending contacts, finite net, apex/plane/end ordering and bounded multiple contacts. Unknown spin stays unknown through contact. Independent numerical comparison covers 1 mm position / 0.5 ms event budgets; covariance is explicitly conditional, not a calibrated full uncertainty model.

Initial coefficients are unchanged: drag_k=0.12861635416914574; magnus_k=0.00379260504480258; restitution_v=0.9313775047019153; table_tangential_gain=0.38744443855065164 (bounded [0,0.4]); table_friction_cap_mu=0.16706204822993975; radius=0.02 m; gravity=(0,0,-9.81) m/s².

config/flight_prediction_v2_shadow.yaml has conservative full/packet output at 50 Hz. Compact output remains source-paced. Inspect `hope_flight_prediction_v2_replay --profile`; validated `--set` overrides change configuration identity. Every packet includes model/clock/frame/plane provenance. No tuning was performed during isolation.

Legacy Stage 2 keeps its original distinct Nakashima and venue contact laws, geometry, stepping, fit and fallback semantics. It is intentionally not numerically identical to the V2 kernel. Production adoption requires a separate reviewed migration.

Historical evaluation pins actual upstream code at 8d599e7. It uses original EstimatorConfig defaults (window .18 s, min span .08 s, min samples 12), fixed x_hit=.15, horizon cap 3 s, adaptive acquisition disabled, estimator/future spin and Magnus disabled. Effective BallPhysics coefficients are the upstream defaults listed above (plus restitution_h=.6125555614493483, Nakashima friction=.25). These are explicitly an upstream-default evaluation profile; deployed YAML overrides such as post-net zero-spin gain=.075 are not inferred from a filename. Raw producer omega is unsupported by the historical quaternion-only estimator and is not manufactured into an orientation. Legacy zero-prior contact behavior is labelled separately from V2 POSITION_ONLY.

The legacy API exposes a receiving-plane prediction, not bounce/apex trajectories. The adapter marks other events unsupported instead of generating them using V2 physics. Actual one-shot trigger epochs are sparse; intermediate predictions are never interpolated. Matched comparison currently rejects moving-plane sessions and accepts only a registered fixed .15 m X-plane. This limit is enforced before running binaries.
