# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| jk_bms.yaml | Defines the exact address/name/model/hardware/software/serial/date, read-only JK02-32S BLE contract and physical plausibility bounds for IG Handle's own battery telemetry; the package-level node gate remains false until GRANDE real bringup explicitly enables it, which is the standard default. | Physical BMS app Device Info evidence plus live read-only frame capture | launch/core/start_power.launch, scripts/power/jk_bms_node.py |
| sensor_contract.yaml | Defines canonical deployed sensor identity, lifecycle ownership, expected publishers, roles, topics, launch policy, reachability, readiness thresholds, and guarded sonar provider state. | sensor_network.yaml, cameras/, sonar/ | sensor contract runtime, external sensor service, sensor_bringup.py, sensors.launch |
| sensor_frames.html | Interactive reference for configured static sensor origins; labels partial translation evidence, unverified orientation, and omitted dynamic navigation transforms. | sensor_frames.yaml, Plotly | Hardware, TF, and simulator review |
| sensor_frames.yaml | Owns sensor frame names, parent-child transforms, verification metadata, and measurement references. | Physical measurement evidence | scripts/frames/export.py, GRANDE TF launch and dashboard, Heron Simulator sensor profile |
| sensor_models.yaml | Records reference sensor-family capabilities separately from deployed device identity. | Manufacturer specifications | Hardware and simulator review |
