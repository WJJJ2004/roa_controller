# Direct RSU manager -> HW iface, ROS/model actuator coordinates, N m.
# Fixed ordering: left upper(18), left lower(20), right upper(19), right lower(21).
# header is publication time; source headers preserve the epochs used in calculation.
std_msgs/Header header
std_msgs/Header feedback_header
std_msgs/Header target_header
uint32 seq
uint32 target_seq
float64[4] virtual_torque
float64[4] requested_torque
float64[4] torque
uint32 fault_code
bool valid
bool saturated
