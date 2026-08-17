# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| mocap.py | Selects NatNet or validated DataCollect UDP transport, strictly admits runtime flags, and publishes canonical pose, marker cloud, object, status, and optional TF outputs. | rospy, natnet package, udp/datacollect.py, ig_handle_runtime.network_config, ig_handle_runtime.parameters | CMake installation, launch/core/natnet_bridge.launch |
