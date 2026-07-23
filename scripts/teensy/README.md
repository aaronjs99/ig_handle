# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| teensy_rosserial_launcher.py | Launch rosserial only when the optional configured Teensy is present. | os, subprocess, rospy | ig_handle/CMakeLists.txt, ig_handle/launch/core/start_rosserial.launch |
