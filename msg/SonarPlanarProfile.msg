std_msgs/Header header
# Header stamp uses timestamp_semantics; it is not silently treated as acquisition time.
uint8 TIMESTAMP_SOURCE=0
uint8 TIMESTAMP_RECEIPT=1
uint8 TIMESTAMP_OFFSET_CORRECTED_ESTIMATE=2
uint8 timestamp_semantics
builtin_interfaces/Time receipt_time
# Offset applied to receipt time for this product; not a transport-delay measurement.
float64 timestamp_offset_from_receipt_sec
# Stable only when source_identity_complete is true.
string source_measurement_id
string capture_id
string source_session_id
uint64 source_sequence
bool source_identity_complete
string provider
string model
string extrinsic_revision
string processing_profile
bool synthetic
# Physical timing uncertainty has not yet been characterized for these decoded profiles.
bool timing_uncertainty_known
float64 timing_uncertainty_sec
# A valid zero-return DT100 ping is different from no message; it is not by itself free-space evidence.
uint8 SAMPLES_ARE_RETURNS=0
uint8 SAMPLES_ARE_INTENSITY_BINS=1
uint8 sample_semantics
bool profile_complete
uint32 nominal_sample_count
# Returns are scan-plane samples; this contract carries no elevation observation.
uint32[] sample_indices
float64[] ranges_m
float64[] azimuth_rad
float32[] intensity
