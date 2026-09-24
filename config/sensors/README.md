# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| README.md | Explains the sensor configuration modules and their ownership. | platform/, cameras/, power/, sonar/ | IG Handle operators and launch maintainers |
| platform/sensor_contract.yaml | Owns deployed sensor identities, bindings, acquisition topics, and launch policy. | network/sensor_network.yaml, cameras/, sonar/ | sensor contract runtime, sensor bringup, external sensor service, GRANDE launch |
| platform/sensor_frames.yaml | Owns sensor frame names, static extrinsics, verification metadata, and measurement references. | Physical measurement evidence | TF export/broadcast, GRANDE launch, Heron Simulator |
| platform/sensor_models.yaml | Records reference sensor-family capabilities separately from deployed device identity. | Manufacturer specifications | Hardware and simulator review |
| platform/sensor_frames.html | Interactive review of the configured sensor origins and frame evidence. | platform/sensor_frames.yaml, Plotly | Hardware, TF, and simulator review |
| cameras/README.md | Documents serial-bound camera profiles and calibration status. | cameras/*.yaml, sensor contract | Camera launch and calibration work |
| power/README.md | Documents battery registry and JK BMS configuration ownership. | power/battery_registry.yaml, power/jk_bms.yaml | Battery launch and passive power telemetry |
| sonar/README.md | Documents sonar profiles and their deployment assumptions. | sonar/profiles.yaml, sensor contract | Sonar launch and processing |

The directory is split by responsibility: platform metadata and extrinsics are
under `platform/`, while camera, power, and sonar inputs remain in their
modality-specific modules. Runtime code refers to these paths explicitly; no
second copy is maintained.
