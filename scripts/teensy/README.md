# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| teensy_rosserial_launcher.py | Waits for a late Teensy device, supervises rosserial with canonical diagnostic topics, and restarts after bridge exit without fabricating availability. | rospy, rosserial_python, config/udev/99-ig-handle.rules | CMake installation, launch/core/start_rosserial.launch |
