# Primitive-only firmware timing payload. It deliberately has no std_msgs/Header:
# rosserial rewrites Header.seq, while source_sequence must survive unchanged.
uint8 KIND_PPS_REFERENCE=1
uint8 KIND_CAMERA_EXPOSURE_MIDPOINT=2
uint8 KIND_IMU_SYNC_OUT=3
uint8 KIND_SENSOR_TRIGGER_COMMAND=4
uint8 kind
uint8 source_index
uint32 source_sequence
uint8 source_sequence_bits
uint32 source_clock_epoch
bool source_clock_epoch_valid
bool correlation_sequence_valid
uint32 correlation_sequence
uint32 raw_time_sec
uint32 raw_time_nsec
