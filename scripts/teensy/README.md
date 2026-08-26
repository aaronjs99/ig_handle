# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| build_firmware.sh | Stages the checked-in Teensy source and configuration, verifies the installed content-pinned ros_lib snapshot and its clean rosserial source commit, pins the Teensy 4.1 toolchain/libraries, embeds a portable source-bound build ID, and emits hashed ELF/HEX artifacts without flashing. | Arduino CLI 1.3.0, Teensy core 1.60.0, RTClib 2.1.4, Adafruit BusIO 1.17.4, rosserial c169ae2, ros_lib 2abba46 | Source-checkout firmware build and commissioning record |
| teensy_rosserial_launcher.py | Bounds device startup, supervises a uniquely named rosserial child, and fail-stops on child exit, device loss, or missing, stale, or mismatched commissioned firmware identity. | rospy, rosserial_python, std_msgs, config/udev/99-ig-handle.rules | CMake installation, launch/core/start_rosserial.launch |
