# Causal context from an authoritative rally source. Not an actuator command.
uint8 BEGIN=0
uint8 RESET=1
uint8 VERIFIED_CONTACT=2
uint8 kind
int64 source_time_ns
uint64 rally_id
bool we_are_serving
# True only when acquisition/history continuity has been independently established.
bool history_complete
string<=128 frame_id
string<=16 contact_side
geometry_msgs/Point contact_position
string<=128 evidence_source_id
