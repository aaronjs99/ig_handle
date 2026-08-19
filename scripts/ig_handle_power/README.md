# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| __init__.py | Declares the installed IG Handle power package. | Python | catkin Python packaging |
| bluez_ble.py | Owns exact-address/name/service discovery, staged device-info then telemetry read queries, notification delivery, and reconnect behavior for one commissioned JK BMS. | BlueZ D-Bus, GLib | scripts/power/jk_bms_node.py |
| jk_bms_protocol.py | Strictly assembles and decodes the explicitly selected JK02 24S/32S read-only frame layout with checksum, identity, cell-count, named alarm/status, and physical-plausibility checks. | Python stdlib | scripts/power/jk_bms_node.py, GRANDE power contract test |
