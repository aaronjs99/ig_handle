# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| mocap.py | Selects NatNet or validated DataCollect UDP transport and publishes canonical pose, marker cloud, object, status, and optional TF outputs. | rospy, natnet package, udp/datacollect.py, network_config.py | CMake installation, launch/core/natnet_bridge.launch |
