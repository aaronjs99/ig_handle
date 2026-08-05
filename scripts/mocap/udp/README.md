# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| __init__.py | UDP mocap transport helpers. | None | Python package imports |
| datacollect.py | Receives, validates, and decodes Motive-side Heron mocap JSON over UDP. | json, math, socket, rospy | ../mocap.py |
