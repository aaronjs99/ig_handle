# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| runtime_surface.yaml | Owns the canonical Heron/IG Handle power, mocap, telescope, raw and typed timing, raw sonar, decoded sonar, and Ping360 topic names. | ROS interface contract | GRANDE launch, dashboard, recording, and sensor integration; IG Handle launch; config/teensy/firmware_config.h; scripts/teensy/timing_event_adapter.py |
