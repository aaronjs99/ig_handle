# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| mocap.py | Selects packaged NatNet or validated DataCollect UDP transport, strictly admits runtime flags, and publishes canonical pose, marker cloud, object, status, and optional TF outputs without modifying Python search paths. | rospy, ig_handle_mocap_natnet, ig_handle_mocap_udp, ig_handle_runtime.network_config, ig_handle_runtime.parameters | CMake installation, launch/core/natnet_bridge.launch |
