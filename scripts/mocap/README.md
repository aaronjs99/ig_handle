# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| __init__.py | Declares the motion-capture transport package. | Python | IG Handle motion-capture imports |
| bridge.py | Selects packaged NatNet or validated DataCollect UDP transport, strictly admits runtime flags, and publishes canonical pose, marker cloud, object, status, and optional TF outputs. | rospy, mocap.natnet, mocap.udp, sensors.network, sensors.parameters | CMake installation, launch/core/natnet_bridge.launch |
