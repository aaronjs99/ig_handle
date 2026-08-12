# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| Ping360RawPacket.msg | Defines timestamped raw Ping360 packet transport with provider-specific extrinsic-revision provenance. | std_msgs/Header | scripts/sonar/include/ping360_provider.py, ig_handle/CMakeLists.txt |
| SonarDiagnostics.msg | Reports provider identity, endpoint, read-only state, protocol and firmware versions, packet counters, errors, timeouts, and configuration hash. | std_msgs/Header | scripts/sonar/include/ping360_provider.py, IG Handle readiness and recording consumers |
| SonarProfile.msg | Defines provider-neutral sonar profile samples with explicit physical-versus-synthetic, acquisition-frame, and extrinsic-revision provenance. Its 2026-08-11 ROS 1 MD5 is `c60a9cd87d90490ea37c2ae5164e2b76`; older bags require an explicit migration bridge before replay. | std_msgs/Header | scripts/sonar/include/ping360_provider.py, mariner sonar imaging, heron_simulator Ping360 profiles, range_aid marker frontend, ig_handle/CMakeLists.txt |
| SonarRawPacket.msg | Preserves a sonar vendor datagram with ROS receipt time, provider identity, source endpoint, packet kind, extrinsic revision, and sequence provenance. | std_msgs/Header | scripts/sonar/include/receiver.py, heron_simulator multibeam simulation, mariner DT100 decoder, ig_handle/CMakeLists.txt |
