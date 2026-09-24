# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| battery_registry.yaml | Commissioned battery identities, roles, labels, and evidence state. | Physical label commissioning | Battery registry and fleet telemetry |
| jk_bms.yaml | Read-only JK02-32S BLE device contract, polling, stale handling, and plausibility bounds. | BMS Device Info and frame capture | battery.launch, start_power.launch, jk_bms_node.py |
