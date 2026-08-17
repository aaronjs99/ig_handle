# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| __init__.py | Marks the standard `ig_handle_mocap_udp` transport package. | Python import system | ../mocap/mocap.py |
| datacollect.py | Receives source-filtered Motive-side JSON, validates its schema, strict tracking-validity type, and finite geometry, and emits stale-state status when packets stop. | socket, rospy, ig_handle_runtime.parameters | ../mocap/mocap.py |
