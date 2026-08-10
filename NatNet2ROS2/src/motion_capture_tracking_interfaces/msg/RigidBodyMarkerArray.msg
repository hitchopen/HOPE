# Atomic rigid-body pose, marker definition and per-frame marker observations.
# Keeping these in one message prevents calibration code from combining marker
# samples from different NatNet frames.
std_msgs/Header header
uint64 timestamp
int32 rigid_body_id
string rigid_body_name
geometry_msgs/Pose rigid_body_pose
float32 mean_marker_error_m
RigidBodyMarker[] markers
