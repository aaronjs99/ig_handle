# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| firmware_pin_contract.h | Validates whole-firmware Teensy pin uniqueness, RTC-I2C exclusions, and telescope PWM/ADC capabilities before enabled I/O initialization. | ../config/teensy/firmware_config.h, Teensy 4.1 pin contract | main.ino |
| main.ino | Teensy 4.1 entrypoint for reference-qualified sensor timing, observed camera/IMU event stamps, diagnostics, ROS serial, and fail-closed telescope operation. | Arduino, rosserial, RTClib, ../config/teensy/firmware_config.h, firmware_pin_contract.h, sensor_sync_runtime.h, telescope_runtime.h | Firmware build and flash |
| sensor_sync.h | Side-effect-free reference qualification, continuous rollover-safe relative-epoch mapping, trigger scheduling, feedback tracking, and bounded ISR-mailbox primitives. | stdint.h | sensor_sync_runtime.h, main.ino, ephemeral host validation |
| sensor_sync_runtime.h | Single-owner Teensy timer adapter for four Forge trigger/ExposureActive channels, two VLP-16 PPS paths, one MTi event-marker pair, and timing diagnostics. | Arduino, ../config/teensy/firmware_config.h, sensor_sync.h | main.ino |
| telescope_control.h | Pure calibrated encoder/length conversion helpers that reject provisional zero calibration. | stdint.h, ../config/teensy/firmware_config.h | telescope_runtime.h, ephemeral host validation |
| telescope_runtime.h | Telescope hardware/state adapter with stable complementary-limit qualification, request-driven homing, timeout/stall/overcurrent fatal stops, and break-before-make motor reversal. | Arduino.h, math.h, telescope_control.h, ../config/teensy/firmware_config.h | main.ino |
