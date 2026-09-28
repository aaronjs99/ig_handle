# Typed record of identified timing events and observed sensor-clock values.
# Source clocks remain unmapped until measured phase and transport behavior establish a mapping.
uint8 KIND_PPS_REFERENCE=1
uint8 KIND_CAMERA_EXPOSURE_MIDPOINT=2
uint8 KIND_IMU_SYNC_OUT=3
uint8 KIND_CAMERA_FRAME=4
uint8 KIND_LIDAR_CLOUD_HEADER=5
uint8 KIND_SENSOR_TRIGGER_COMMAND=6
uint8 KIND_MOCAP_FRAME=7
uint8 CORRELATION_UNRESOLVED=0
uint8 CORRELATION_MEASURED=1
uint8 CORRELATION_MISSING_EVENTS=2
uint8 CORRELATION_CONFLICT=3
uint8 ORDER_FIRST=0
uint8 ORDER_FORWARD=1
uint8 ORDER_GAP=2
uint8 ORDER_DUPLICATE=3
uint8 ORDER_OUT_OF_ORDER=4
uint8 ORDER_CLOCK_EPOCH=5
uint8 ORDER_SOURCE_RESTART_INFERRED=6
# Counter reset with a continuing raw clock, confirmed by subsequent advancing data.
uint8 ORDER_SEQUENCE_RESTART_INFERRED=7
# Raw clock or counter unavailable; ordering state is retained without advancement.
uint8 ORDER_UNAVAILABLE=8
uint8 IDENTITY_SCOPE_BRIDGE_INSTANCE=1
uint8 SEQUENCE_COUNTER_SOURCE=1
# source_sequence is the ROS Header.seq value when this origin is selected.
uint8 SEQUENCE_COUNTER_ROS_HEADER=2
# header.stamp is the source ROS publication/receipt stamp. source_sequence preserves its declared width.
std_msgs/Header header
uint8 kind
uint8 identity_scope
string event_id
uint64 bridge_event_sequence
string source_key
string source_id
string source_instance_id
bool source_sequence_valid
uint64 source_sequence
uint8 source_sequence_origin
string bridge_instance_id
string firmware_build_id
bool source_boot_id_valid
string source_boot_id
uint64 observed_source_generation
bool source_generation_inferred
uint32 source_clock_epoch
bool source_clock_epoch_valid
uint8 source_sequence_bits
string source_clock_domain
# Fit scope includes the acquisition adapter instance and observed reset generation.
# Hardware boot identity remains unknown unless supplied by the device.
string clock_mapping_source_instance_id
string clock_mapping_source_epoch_id
bool raw_source_time_valid
builtin_interfaces/Time raw_source_time
# Preserve an original floating timestamp and unconverted device ticks when supplied.
float64 raw_source_time_seconds
string[] auxiliary_clock_names
uint64[] auxiliary_clock_ticks
float64[] auxiliary_clock_frequency_hz
builtin_interfaces/Time receipt_ros_time
uint64 receipt_monotonic_ns
string receipt_host_boot_id
uint64 ros_time_epoch
uint8 order_disposition
uint64 missing_source_sequences
# Teensy timing outputs carry a source-local trigger counter shared by the
# command and feedback records. It is scoped to the firmware clock epoch and
# is not a durable cross-restart capture ID.
bool correlation_sequence_valid
string correlation_source_id
uint32 correlation_sequence
# Exact measured links; a shared trigger may belong to four camera captures.
string[] correlated_capture_ids
string[] correlated_event_ids
uint8 correlation_status
uint64 correlation_revision
# A derived revision preserves this original immutable physical event.
bool derived_revision
string original_event_id
uint64 mapped_continuity_epoch
bool sensor_frame_counter_valid
uint64 sensor_frame_counter
bool mapped_acquisition_time_valid
builtin_interfaces/Time mapped_acquisition_time
uint64 clock_mapping_revision
float64 timing_uncertainty_sec
bool clock_mapping_calibrated
