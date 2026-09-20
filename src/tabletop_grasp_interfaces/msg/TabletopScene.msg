# One observation; never combine boxes/plane from different timestamps.
std_msgs/Header header
bool valid
string reason
# Solid table box: its TOP face is the fitted plane, local Z points up.
geometry_msgs/Pose table_pose
geometry_msgs/Vector3 table_dimensions
uint32 table_point_count
float64 table_inlier_ratio
OrientedObject[] objects
int32 target_index
