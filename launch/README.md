# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| sensors.launch | Starts the contract-driven sensor supervisor with explicit extra, disabled, and reachability selections. | config/sensors/sensor_contract.yaml, scripts/sensor_bringup.py | GRANDE bringup and direct IG Handle sensor operation |
