int64 prediction_reference_time_ns
int64 arrival_dt_ns
int64 predicted_arrival_time_ns
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 omega_rad_s
bool spin_available
string spin_mode
float64[36] covariance
string linearization_quality
# measured_valid is true only at the actual measurement instant; usable includes held/model spin.
bool spin_measured_valid
int64 spin_measurement_time_ns
uint64 spin_last_contact_id
string spin_provenance
