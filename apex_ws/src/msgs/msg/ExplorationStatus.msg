std_msgs/Header header
# States: IDLE, EXPLORING, TARGET_FOUND, NAVIGATING_TO_TARGET, COMPLETED
string state
float32 explored_ratio
int32 frontiers_count
bool target_found
geometry_msgs/Point target_position
