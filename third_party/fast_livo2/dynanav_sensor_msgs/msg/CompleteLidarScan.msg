# Explicit producer attestation for one atomic, unchanged PointCloud2.
# Version 1 only: frozen OS1 Rev6 32ch 10Hz 1024res, full accumulated output.
# [source_start_ns,source_end_ns) certifies scan completion, NEVER free space.
uint32 contract_version
string<=64 session_id
uint64 sequence
uint64 native_frame_id
string<=64 sensor_id
string<=160 config_id
uint64 source_start_ns
uint64 source_end_ns
# Empirically audited coordinate-frame sampling phase, NOT scan start.
uint64 coordinate_frame_start_ns
uint64 coordinate_frame_end_ns
bool accumulate_outputs
bool scan_complete
sensor_msgs/PointCloud2 cloud
