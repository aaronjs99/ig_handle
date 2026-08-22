# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| JkBmsDetails.msg | Carries JK-specific device identity, provenance, cell-balance, cycle, alarm, and switch details alongside the standard `sensor_msgs/BatteryState` `/sense_ighandle` surface. | std_msgs/Header | scripts/power/jk_bms_node.py, recording and power diagnostics |
| Ping360RawPacket.msg | Defines timestamped raw Ping360 packet transport with provider-specific extrinsic-revision provenance. | std_msgs/Header | scripts/sonar/ping360_provider.py, ig_handle/CMakeLists.txt |
| SonarDiagnostics.msg | Reports provider identity, endpoint, read-only state, protocol and firmware versions, packet counters, errors, timeouts, and configuration hash. | std_msgs/Header | scripts/sonar/ping360_provider.py, IG Handle readiness and recording consumers |
| SonarProfile.msg | Defines provider-neutral sonar profile samples with explicit physical-versus-synthetic, acquisition-frame, and extrinsic-revision provenance. | std_msgs/Header | scripts/sonar/ping360_provider.py, mariner sonar imaging, heron_simulator Ping360 profiles, range_aid marker frontend, ig_handle/CMakeLists.txt |
| SonarRawPacket.msg | Preserves a sonar vendor datagram with ROS receipt time, provider identity, source endpoint, packet kind, extrinsic revision, and sequence provenance. | std_msgs/Header | scripts/sonar/receiver.py, heron_simulator multibeam simulation, mariner DT100 decoder, ig_handle/CMakeLists.txt |
