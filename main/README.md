# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| main.ino | Implements the Teensy firmware entry point. | None | None |
| telescope_control.h | Defines the telescope control C++ interface or constants. | stdint.h, ../config/teensy/firmware_config.h | ig_handle/main/main.ino, ig_handle/main/telescope_runtime.h |
| telescope_runtime.h | Defines the telescope runtime C++ interface or constants. | Arduino.h, math.h, telescope_control.h | ig_handle/main/main.ino |
