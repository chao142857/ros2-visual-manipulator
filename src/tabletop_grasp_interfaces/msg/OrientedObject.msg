# Pose of box center. Local X=long, Y=short, Z=table normal (up).
string id
geometry_msgs/Pose pose
geometry_msgs/Vector3 dimensions
uint32 point_count
# Heuristic geometric support, NOT a calibrated probability.
float64 quality
bool yaw_observable
