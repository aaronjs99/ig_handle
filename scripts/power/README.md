# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| heron_sense_ingress.py | Admits only the configured raw Heron MCU publisher and republishes finite electrical/RC telemetry on the IG Handle-owned `/sense_heron` boundary without changing the Heron. | rospy, heron_msgs/Sense | launch/core/start_power.launch, GRANDE runtime consumers |
| jk_bms_node.py | Holds one commissioned, read-only JK BLE connection, admits the exact MAC/name/model/hardware/software/serial/date identity, and publishes standard `/sense_ighandle` battery state plus provenance-complete `/sense_ighandle/details`; publishes explicit unavailable state on identity, protocol, or freshness failure. | rospy, sensor_msgs/BatteryState, ig_handle/JkBmsDetails, ig_handle_power | launch/core/start_power.launch |
