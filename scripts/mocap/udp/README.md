# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| __init__.py | Exposes the DataCollect UDP transport as an importable mocap subpackage. | Python import system | ../mocap.py |
| datacollect.py | Receives source-filtered Motive-side JSON, validates its schema and finite geometry, and emits stale-state status when packets stop. | socket, rospy | ../mocap.py |
