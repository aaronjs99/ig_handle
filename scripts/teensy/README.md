# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| teensy_rosserial_launcher.py | Waits for the stable Teensy device, starts rosserial with canonical timing topics, and exits cleanly when optional hardware is absent. | rospy, rosserial_python, config/udev/99-ig-handle.rules | CMake installation, launch/core/start_rosserial.launch |
