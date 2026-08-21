# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| sensors.launch | Starts the contract-driven sensor supervisor, which launches internally owned providers and observes externally owned providers with explicit selection and reachability state. | config/sensors/sensor_contract.yaml, scripts/sensor_bringup.py | GRANDE bringup and direct IG Handle sensor health observation |
