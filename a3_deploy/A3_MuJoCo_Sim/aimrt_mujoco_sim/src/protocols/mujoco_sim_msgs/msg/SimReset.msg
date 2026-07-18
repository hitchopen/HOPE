std_msgs/Header header

uint8 MODE_ABSOLUTE=0
uint8 MODE_KEYFRAME=1
uint8 mode
int32 keyframe_id

bool set_base
geometry_msgs/Pose pelvis_pose

bool set_base_twist
geometry_msgs/Twist pelvis_twist

bool set_joints
sensor_msgs/JointState joint_state

bool zero_all_velocities
bool clear_ctrl
