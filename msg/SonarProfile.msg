std_msgs/Header header
# Acquisition identity. Distinct profiles retain distinct IDs even if their
# intensities and raw packet bytes are identical; raw_packet_id is content-only.
string profile_id
string provider
# Publisher acquisition-session identity; not a hardware boot ID.
string source_session_id
string model
string raw_packet_id
string extrinsic_revision
bool synthetic
uint32 sequence
bool valid
string validity_reason
float64 angle_rad
uint16 angle_grad
bool auto_scan
uint16 start_angle_grad
uint16 stop_angle_grad
uint8 num_steps
uint8 gain_setting
uint16 transmit_duration_us
uint16 sample_period_ticks_25ns
uint16 transmit_frequency_khz
float32 sound_speed_mps
float32 sample_interval_m
float32 min_range_m
float32 max_range_m
uint8[] intensities
