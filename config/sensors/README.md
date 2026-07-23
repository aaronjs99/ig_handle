# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| sensor_contract.yaml | Provides configuration values for sensor contract. | None | README.md, contrib/shreyam/README.md, launch/sensors.launch |
| sensor_frames.html | Provides the sensor_frames.html artifact used by this folder. | None | None |
| sensor_frames.yaml | Provides configuration values for sensor frames. | None | README.md, config/sensors/sensor_models.yaml, scripts/frames/export.py |
| sensor_models.yaml | Provides configuration values for sensor models. | sensor_frames.yaml | None |
