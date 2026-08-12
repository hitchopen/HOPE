# One source-time-causal OptiTrack sample inside an immutable BallFlightPacket.
# exposure_unix_stamp_ns is the original camera-exposure time converted to the
# Unix epoch by the Laptop NatNet adapter. It is never replaced by relay or
# receipt time.
int64 exposure_unix_stamp_ns
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
bool orientation_valid
