# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| sensor_contract.yaml | Canonical deployed sensor identities, bindings, topics, and acquisition policy. | cameras/, sonar/, network/sensor_network.yaml | sensor_bringup.py, contracts.py, launch/sensors.launch |
| sensor_frames.yaml | Canonical frames, static extrinsics, verification metadata, and measurement references. | Physical calibration evidence | frames/export.py, frames/broadcast.py, GRANDE and simulator launches |
| sensor_models.yaml | Reference capability and model metadata. | Manufacturer specifications | Hardware and simulator review |
| sensor_frames.html | Interactive frame and measurement review. | sensor_frames.yaml, Plotly | Operators and calibration review |
