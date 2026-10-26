std_msgs/Header header

string active_profile
string active_hardware_mode
string active_controller
string linkage_mode
string switch_state
string lifecycle_state
string detail

bool switching
bool ready
bool command_timed_out
bool degraded

string[] joint_names
float64[] measured_positions
float64[] reference_positions
float64[] commanded_velocities
