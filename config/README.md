# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| runtime_surface.yaml | Owns the canonical IG Handle mocap, telescope, timing, raw sonar, decoded sonar, and Ping360 topic names. | ROS interface contract | GRANDE launch, dashboard, recording, and sensor integration; IG Handle launch; config/teensy/firmware_config.h |
