uint8 CONTACT_RACKET=1
uint8 CONTACT_TABLE=2
uint8 CONTACT_NET=4

std_msgs/Header header
uint64 shot_id
bool active
geometry_msgs/Point position
geometry_msgs/Vector3 linear_velocity

# Contact bits describe the current MuJoCo step.  Counts are rising-edge counts
# accumulated by the 1 kHz plant and therefore cannot miss a short contact just
# because this message is published at a lower rate.
uint8 contact_bits
uint32 racket_contact_count
uint32 table_contact_count
uint32 net_contact_count
# Peak normal force accumulated since this shot was launched.
float64 racket_normal_force_n
