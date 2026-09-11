# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| battery.launch | Launches standalone JK BMS acquisition without a logger. | jk_bms_node.py, battery registry | Battery monitoring service and launch |
| sensors.launch | Starts the required sensor supervisor; explicit use_external_sensor_provider also owns the required selected fixed-profile Xsens provider for a single test. Default false preserves standalone service composition. | sensor contract, sensor_bringup.py, external_sensor_provider.py | GRANDE bringup and standalone sensor observation |
