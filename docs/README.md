# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| imu_characterization.md | Defines the non-actuating Xsens stationary-analysis workflow, candidate-only covariance boundary, physical follow-up gates, and DLiO uncertainty qualification requirements. | ../scripts/imu/analyze_stationary_bag.py, ../scripts/imu/validate_stationary_candidate.py, ../config/sensors/sensor_contract.yaml | IMU calibration review, future experiment execution, and covariance commissioning decisions |
| platform.md | Defines the physical sensor platform, network, supervision, recording, and device-identity boundaries. | runtime_surface.yaml, sensor_contract.yaml, sensor_frames.yaml, sensor_network.yaml | Sensor integration and field preparation |
| platform.pdf | Rendered sensor-platform reference. | platform.md | Review and offline sharing |
| sensor_timing.md | Defines fail-closed Teensy reference and field-valid qualification, hardware-owned VLP PPS, bounded inverted status-V NMEA with per-unit qualifier/lock readback, camera/MTi trigger-feedback contracts, asynchronous sonar safety boundary, and bench acceptance gates. | ../config/teensy/firmware_config.h, ../main/sensor_sync.h, official device manuals | Firmware integration and commissioning |
| sensor_timing.pdf | Rendered sensor-timing firmware reference. | sensor_timing.md | Review and offline sharing |
| timing_circuit.md | Source-of-truth V6 circuit architecture, subsystem rationale, pin/connector contract, grounding, delays, safety, and release gates. | V6 KiCad design, ../config/teensy/firmware_config.h, sensor and interface manuals | Electrical review, firmware alignment, PCB release, and future V6 changes |
| timing_circuit.pdf | Rendered V6 circuit explanation and diagrams. | timing_circuit.md | Lab review and offline sharing |
| telescope.md | Defines the one-switch telescoping-arm hardware, open-means-trip behavior, homing, encoder/field-valid supervision, asynchronous field-invalid stop, packaging, and open measurements. | ../config/telescope/hardware.yaml, ../config/teensy/firmware_config.h, ../config/runtime_surface.yaml | Arm assembly and embedded-control work |
| telescope.pdf | Rendered telescope reference. | telescope.md | Review and offline sharing |
