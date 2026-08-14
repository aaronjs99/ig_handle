# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| firmware_pin_contract.h | Validates whole-firmware Teensy pin uniqueness, RTC-I2C exclusions, and telescope PWM/ADC capabilities before enabled I/O initialization. | firmware_config.h, Teensy 4.1 pin contract | main.ino |
| firmware_build_identity.h | Fail-closed identity used by direct IDE builds; the pinned build script replaces it only in its temporary staged sketch with the exact source SHA-256. | None | main.ino, scripts/teensy/build_firmware.sh |
| main.ino | Teensy 4.1 entrypoint for reference-qualified sensor timing, source-bound build diagnostics, observed camera/IMU event stamps, ROS serial, and fail-closed telescope operation. | Arduino, rosserial, RTClib, firmware_config.h, firmware_build_identity.h, firmware_pin_contract.h, sensor_sync_runtime.h, telescope_runtime.h | Firmware build and flash |
| sensor_sync.h | Side-effect-free reference qualification, continuous rollover-safe relative-epoch mapping, trigger scheduling, feedback tracking, and bounded ISR-mailbox primitives. | stdint.h | sensor_sync_runtime.h, main.ino, ephemeral host validation |
| sensor_sync_runtime.h | Single-owner Teensy timer adapter for four Forge trigger/ExposureActive channels, two VLP-16 PPS paths, one MTi event-marker pair, and timing diagnostics. | Arduino, firmware_config.h, sensor_sync.h | main.ino |
| telescope_control.h | Pure calibrated encoder/length conversion helpers that reject provisional zero calibration. | stdint.h, firmware_config.h | telescope_runtime.h, ephemeral host validation |
| telescope_runtime.h | Telescope hardware/state adapter with stable complementary-limit qualification, request-driven homing, timeout/stall/overcurrent fatal stops, and break-before-make motor reversal. | Arduino.h, math.h, telescope_control.h, firmware_config.h | main.ino |
