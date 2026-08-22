# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| __init__.py | Marks the standard `mocap.udp` transport package. | Python import system | ../bridge.py |
| datacollect.py | Receives source-filtered Motive-side JSON, validates its schema, strict tracking-validity type, and finite geometry, and emits stale-state status when packets stop. | socket, rospy, sensors.parameters | ../bridge.py |
