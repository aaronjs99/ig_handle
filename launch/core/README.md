# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| natnet_bridge.launch | Selects NatNet or DataCollect UDP mocap transport, resolves configured endpoints, and publishes canonical mocap topics with stale-data handling. | config/network/sensor_network.yaml, config/runtime_surface.yaml, scripts/mocap/mocap.py, MARINER config evaluator | GRANDE bringup and mocap-assisted state review |
| start_rosserial.launch | Starts the conditional fail-stop Teensy launcher with bounded device/identity deadlines and canonical PPS, camera-time, IMU-time, and diagnostic-status topics. | /dev/teensy, commissioned firmware build ID, scripts/teensy/teensy_rosserial_launcher.py | GRANDE bringup and embedded timing acquisition |
