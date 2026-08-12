# One marker belonging to a vendor rigid-body definition.
#
# model_position is the expected marker centre in the rigid-body local frame.
# position is the per-frame reconstructed marker centre in header.frame_id when
# has_live_sample is true. params is the raw NatNet labeled-marker bit field.
uint32 member_id
string name
geometry_msgs/Point model_position
int32 required_active_label
bool has_live_sample
geometry_msgs/Point position
float32 size_m
uint16 params
float32 residual_m
