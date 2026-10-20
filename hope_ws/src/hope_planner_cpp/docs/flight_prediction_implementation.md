# Isolated V2 architecture

`hope_v2_forward_physics` contains common_ball_physics and FlightSpinEstimator.
`hope_v2_forecast_core` contains CKF stepping/tracking, nominal events, checkpoints, physical history/config hashes, hit-plane manager and FlightPipeline/ShotContext.
`hope_v2_transport` encodes only six physical V2 messages.
The V2 node and replay do not link hope_planner_core. return_region contains required side/history enums and canonical SHA256/config identities; its original serialization IDs are retained.

The independent evaluation adapter links the original upstream batch estimator, spin estimator, predictor, incoming-flight segmentation and its physical adaptive-commit dependency. Adaptive commit is needed to link IncomingTrajectory even with adaptive acquisition disabled. The exporter calls no Stage 3 or command code; binary scope checks it separately.

Exactly five public states: INVALID, OPPONENT_RALLY, RALLY, OPPONENT_SERVE, SERVE. A predicted/authoritative SERVE route before observed striker B1 remains publicly provisional. Semantic revisions fence asynchronous solves without discarding same-shot samples; hard boundaries retire shot/spin/event generations.

Spin is MEASURED, HELD, CONTACT_PROPAGATED or POSITION_ONLY. Unknown or rejected angular metadata never synthesizes omega, destroys trusted same-shot spin, or loses valid XYZ. Once-only contacts and trusted checkpoint ledger survive transfer and window truncation. Source timestamps remain signed int64 nanoseconds; epoch zero is valid.

The registered table-frame X-plane is base_x + 0.58 m. FOLLOW_POSE follows fresh qualified pose; ON_REFRESH freezes the last qualified refresh. Pose loss withholds the plane event but preserves useful physical trajectories. No arbitrary-heading transform or live calibration is claimed.

The bounded worker owns one running and one latest pending job. Outputs are confined to /hope/shadow/flight_v2/{prediction,trajectory,packet}; command-topic remaps fail. Observation/context revisions are 2; forecast/packet revisions are 3. Old schema-2 forecast/packet data are rejected.

The OptiTrack relay adds a typed observation without changing legacy pose ordering, stamps, rate policy, placeholders or CSV columns. Orientation authority defaults false; an identity quaternion alone is not evidence. No clock-domain conversion is invented.
