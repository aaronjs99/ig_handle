# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| natnet_bridge.launch | Selects NatNet or DataCollect UDP mocap transport through explicit launch arguments and publishes canonical mocap topics with stale-data handling. | scripts/mocap/bridge.py | GRANDE bringup and mocap-assisted state review |
| start_power.launch | Establishes the IG Handle-owned `/sense_heron` ingress and starts the read-only JK provider only when its complete commissioned identity also resolves through the physical battery registry. | scripts/power/, config/sensors/jk_bms.yaml, config/sensors/battery_registry.yaml, config/runtime_surface.yaml | GRANDE bringup |
| start_rosserial.launch | Starts the conditional fail-stop Teensy launcher with bounded device/identity deadlines and canonical PPS, camera-time, IMU-time, and diagnostic-status topics. | /dev/teensy, commissioned firmware build ID, scripts/teensy/teensy_rosserial_launcher.py | GRANDE bringup and embedded timing acquisition |
