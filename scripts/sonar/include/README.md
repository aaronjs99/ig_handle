# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| __init__.py | Marks the sonar provider implementation as an importable Python package. | Python import system | ../sonar.py |
| deltat_runner.py | Materializes one validated DeltaT runtime INI and supervises the pinned vendor executable. | profiles.py, Linux_DeltaT_v1023_x86_64 | ../sonar.py |
| ping360_provider.py | Implements UDP Ping Protocol discovery, read-only identity reporting, and explicitly authorized Ping360 scan/profile publication. | rospy, ig_handle sonar messages, ping_protocol.py | ../sonar.py, launch/sensors/start_sonar.launch |
| ping_protocol.py | Implements checksum-validated Ping Protocol framing plus Ping360 identity, device-data, and profile payload codecs. | Python binary-struct support | ping360_provider.py |
| profiles.py | Loads and validates named DeltaT acquisition profiles together with canonical network endpoints. | PyYAML, config/sensors/sonar/profiles.yaml, network_config.py | deltat_runner.py |
| receiver.py | Byte-preserves admitted sonar UDP datagrams with receipt, provider, frame, revision, sequence, and actual source endpoint; DT100-labelled ingress requires explicit hardware commissioning plus a configured expected source IP and optionally port. | rospy, ig_handle/SonarRawPacket, network_config.py | ../sonar.py, MARINER strict 83P decoder |
