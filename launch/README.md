# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| battery.launch.py | Launches standalone JK BMS acquisition without a logger. | jk_bms_node.py, battery registry | Battery monitoring service and launch |
| battery.launch | ROS 1 compatibility launch for the standalone read-only JK BMS telemetry node. | jk_bms_node.py, battery registry | Legacy launch callers during migration |
| battery.launch.py | Launches standalone JK BMS acquisition without a logger. | jk_bms_node.py, battery registry | ROS 2 battery monitoring |
| sensors.launch | ROS 1 sensor composition retained for legacy callers while the sensor lifecycle owner is ported. | ROS 1 sensor drivers, sensor_bringup.py | Legacy sensor bringup |
