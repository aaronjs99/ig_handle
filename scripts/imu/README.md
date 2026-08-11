# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| force_wakeup.py | Sends the Xsens bootloader wake-up sequence for supervised recovery without replacing normal driver bringup. | PyUSB, physical Xsens USB connection | CMake installation and manual IMU recovery |
