string<=32 kind
int64 dt_ns
int64 absolute_time_ns
bool absolute_time_valid
float64[6] pre_state
float64[6] post_state
geometry_msgs/Vector3 incoming_direction
geometry_msgs/Vector3 surface_normal
float64 approach_speed_mps
# Joint [pre_state6,time_seconds] covariance, symmetric upper triangle.
# Post-reset state covariance is carried by subsequent fixed-time waypoints.
float64[28] covariance_upper
float64 time_std_s
string<=32 linearization_quality
bool physically_usable
