# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| sensor_contract.yaml | Defines the canonical deployed sensor inventory, roles, topics, launch policy, reachability, provider state, and readiness thresholds. | sensor_network.yaml, cameras/, sonar/ | scripts/sensor_contract.py, scripts/sensor_bringup.py, launch/sensors.launch |
| sensor_frames.html | Interactive reference for configured static sensor origins; labels partial translation evidence, unverified orientation, and omitted dynamic navigation transforms. | sensor_frames.yaml, Plotly | Hardware, TF, and simulator review |
| sensor_frames.yaml | Owns sensor frame names, parent-child transforms, verification metadata, and measurement references. | Physical measurement evidence | scripts/frames/export.py, GRANDE TF launch and dashboard, Heron Simulator sensor profile |
| sensor_models.yaml | Records reference sensor-family capabilities separately from deployed device identity. | Manufacturer specifications | Hardware and simulator review |
