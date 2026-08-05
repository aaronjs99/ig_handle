# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| main.ino | Teensy 4.1 timing, trigger, ROS serial, and fail-closed telescope firmware entrypoint. | Arduino, rosserial, ../config/teensy/firmware_config.h, telescope_runtime.h | Firmware build and flash |
| telescope_control.h | Pure telescope position, limit, current, direction, and duty-cycle control logic. | stdint.h, ../config/teensy/firmware_config.h | main.ino, telescope_runtime.h |
| telescope_runtime.h | Hardware I/O and state adapter for the telescope controller. | Arduino.h, math.h, telescope_control.h, ../config/teensy/firmware_config.h | main.ino |
