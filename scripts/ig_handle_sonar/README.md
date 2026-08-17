# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| __init__.py | Marks the standard `ig_handle_sonar` provider package. | Python import system | ../sonar/sonar.py |
| deltat_runner.py | Materializes one validated DeltaT runtime INI and supervises the pinned vendor executable. | profiles.py, Linux_DeltaT_v1023_x86_64 | ../sonar/sonar.py |
| ping360_provider.py | Implements UDP Ping Protocol discovery, read-only identity reporting, and explicitly authorized physical Ping360 profiles that publish only after checksum-valid device identity and source-device-ID binding. | rospy, ig_handle sonar messages, ig_handle_runtime.parameters, ping_protocol.py | ../sonar/sonar.py, launch/sensors/start_sonar.launch |
| ping_protocol.py | Implements checksum-validated Ping Protocol framing, profile-source identity admission, and Ping360 identity/device/profile payload codecs. | Python binary-struct support | ping360_provider.py, GRANDE sonar contract tests |
| profiles.py | Loads and validates named DeltaT acquisition profiles together with canonical network endpoints. | PyYAML, config/sensors/sonar/profiles.yaml, ig_handle_runtime.network_config | deltat_runner.py |
| receiver.py | Byte-preserves admitted sonar UDP datagrams with receipt, provider, frame, revision, sequence, and actual source endpoint; DT100-labelled ingress requires strict-boolean hardware commissioning plus a configured expected source IP and optionally port. | rospy, ig_handle/SonarRawPacket, ig_handle_runtime.network_config, ig_handle_runtime.parameters | ../sonar/sonar.py, MARINER strict 83P decoder |
