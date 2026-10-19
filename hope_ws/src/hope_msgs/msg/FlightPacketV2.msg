# Immutable shot snapshot. No prediction/waypoint is admitted as an observation.
uint16 schema_version 2
string session_id
string producer_id
uint64 shot_generation
uint64 context_revision
string config_id
string payload_sha256
int64 boundary_time_ns
string history_continuity
string play_state
string forecast_route
geometry_msgs/Vector3 checkpoint_omega_rad_s
bool checkpoint_spin_available
string checkpoint_spin_mode
int64 checkpoint_measurement_time_ns
uint64 checkpoint_last_contact_id
FlightEventV2[] event_ledger
BallObservationV2[] observations
# Frozen six-state CKF checkpoint plus existing correlated measurement-bias nuisance moments.
int64 source_time_origin_ns
int64 state_source_time_ns
int64 last_measurement_time_ns
float64 first_measurement_time_s
float64 last_measurement_gap_s
uint64 observation_count
uint64 rejected_measurements
float64 innovation_squared
float64 innovation_rms_m
float64 innovation_max_m
float64 innovation_sum_squared
uint64 consecutive_rejections
string noise_fingerprint
float64[6] state_mean
float64[36] state_covariance
float64[3] bias_mean
float64[9] bias_covariance
float64[18] state_bias_covariance
string match_context
string classification_status
# Auxiliary model lineage at the CKF posterior, distinct from the authoritative trusted cache.
bool state_spin_available
geometry_msgs/Vector3 state_omega_rad_s
string state_spin_mode
int64 state_spin_measurement_time_ns
uint64 state_spin_last_contact_id
