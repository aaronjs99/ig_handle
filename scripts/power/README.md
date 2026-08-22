# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| __init__.py | Declares the installed IG Handle power package. | Python | catkin Python packaging |
| bluez_ble.py | Owns exact-address/name/service discovery, staged device-info then telemetry read queries, notification delivery, and reconnect behavior for the configured but not yet commissioned JK BMS candidate. | BlueZ D-Bus, GLib | scripts/power/jk_bms_node.py |
| heron_sense_ingress.py | Republishes the admitted raw Heron MCU `Sense` stream on the canonical IG Handle topic after exact publisher and finite-value checks. | rospy, heron_msgs/Sense | GRANDE telemetry and actuator evidence paths |
| jk_bms_node.py | Publishes read-only JK battery telemetry only from the configured device and protocol contract. | rospy, sensor_msgs/BatteryState, ig_handle/JkBmsDetails, bluez_ble.py, jk_bms_protocol.py | IG Handle power launch and GRANDE telemetry |
| jk_bms_protocol.py | Strictly assembles and decodes the explicitly selected JK02 24S/32S read-only frame layout with checksum, identity, cell-count, named alarm/status, and physical-plausibility checks. | Python stdlib | scripts/power/jk_bms_node.py and GRANDE power contracts |
