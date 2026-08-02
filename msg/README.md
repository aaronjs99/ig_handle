# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| Ping360RawPacket.msg | Defines timestamped raw Ping360 packet transport. | None | scripts/sonar/include/ping360_provider.py, ig_handle/CMakeLists.txt |
| SonarDiagnostics.msg | Defines the SonarDiagnostics ROS message fields. | None | ig_handle/CMakeLists.txt |
| SonarProfile.msg | Defines provider-neutral sonar profile samples. | None | scripts/sonar/include/ping360_provider.py, mariner sonar imaging, heron_simulator Ping360 profiles, ig_handle/CMakeLists.txt |
