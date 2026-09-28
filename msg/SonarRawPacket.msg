# ROS receipt time; acquisition time is carried separately when available.
std_msgs/Header header
bool acquisition_time_valid
builtin_interfaces/Time acquisition_time
bool timing_uncertainty_known
float64 timing_uncertainty_sec
string provider
# Publisher acquisition-session identity; not a hardware boot ID.
string source_session_id
bool synthetic
string model
string packet_kind
string source_endpoint
string extrinsic_revision
uint64 sequence
uint8[] payload
