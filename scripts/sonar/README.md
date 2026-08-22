# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| Linux_DeltaT_v1023_x86_64 | Pinned vendor DeltaT executable retained for the optional commissioning-only x86-64 provider. | Linux x86-64, unverified DeltaT hardware path | deltat_runner.py |
| __init__.py | Marks the standard `sonar` provider package. | Python import system | provider.py |
| deltat_runner.py | Materializes one validated DeltaT runtime INI and supervises the pinned vendor executable. | profiles.py, Linux_DeltaT_v1023_x86_64 | provider.py |
| provider.py | Provides the single command entrypoint for raw receive, DeltaT, and Ping360 providers. | receiver.py, deltat_runner.py, ping360_provider.py | CMake installation and sonar launch files |
| ping360_provider.py | Implements UDP Ping Protocol discovery, read-only identity reporting, and explicitly authorized physical Ping360 profiles that publish only after checksum-valid device identity and source-device-ID binding. | rospy, ig_handle sonar messages, sensors.parameters, ping_protocol.py | provider.py, launch/sensors/start_sonar.launch |
| ping_protocol.py | Implements checksum-validated Ping Protocol framing, profile-source identity admission, and Ping360 identity/device/profile payload codecs. | Python binary-struct support | ping360_provider.py and GRANDE sonar contracts |
| profiles.py | Loads and validates named DeltaT acquisition profiles together with canonical network endpoints. | PyYAML, config/sensors/sonar/profiles.yaml, sensors.network | deltat_runner.py |
| receiver.py | Byte-preserves admitted sonar UDP datagrams with receipt, provider, frame, revision, sequence, and actual source endpoint; DT100-labelled ingress requires strict-boolean hardware commissioning plus a configured expected source IP and optionally port. | rospy, ig_handle/SonarRawPacket, sensors.network, sensors.parameters | provider.py, MARINER strict 83P decoder |
