# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| platform.md | Defines the physical sensor platform, network, supervision, recording, and device-identity boundaries. | runtime_surface.yaml, sensor_contract.yaml, sensor_frames.yaml, sensor_network.yaml | Sensor integration and field preparation |
| platform.pdf | Rendered sensor-platform reference. | platform.md | Review and offline sharing |
| sensor_timing.md | Defines fail-closed Teensy reference qualification, trigger/feedback electrical contracts, and bench acceptance gates. | ../config/teensy/firmware_config.h, ../main/sensor_sync.h, official device manuals | Firmware integration and commissioning |
| sensor_timing.pdf | Rendered sensor-timing firmware reference. | sensor_timing.md | Review and offline sharing |
| telescope.md | Defines telescoping-arm hardware, homing, sensing, packaging, and open measurements. | ../config/telescope/hardware.yaml, ../config/teensy/firmware_config.h, ../config/runtime_surface.yaml | Arm assembly and embedded-control work |
| telescope.pdf | Rendered telescope reference. | telescope.md | Review and offline sharing |
