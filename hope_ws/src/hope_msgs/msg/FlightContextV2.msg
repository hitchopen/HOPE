uint16 schema_version 2
string session_id
string authority
uint64 shot_generation
int64 source_time_ns
# START, RESET, CONTEXT, CONTACT, REFRESH_PLANE
string operation
# ACQUISITION, RACKET_CONTACT, FREE_FLIGHT_LAUNCH, RESTART, VERIFIED_CORRECTION
string boundary_evidence
int8 direction
string history_continuity
string match_context
string event_kind
int64 event_time_ns
int64 confirmed_at_ns
geometry_msgs/Point position
geometry_msgs/Vector3 pre_velocity
