# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| 01-netplan.yaml | Host netplan template for the deployed `192.168.131.10` Heron interface, sensor subnet, and canonical 192.168.2.0/24 sonar interface. | Ubuntu netplan, sensor_network.yaml | Manual host network configuration |
| sensor_network.yaml | Canonical runtime endpoints, ROS graph addresses, mocap endpoints, and local interface roles. | Reviewed deployment addresses | scripts/ig_handle_runtime/network_config.py, launch evaluation, GRANDE runner and bringup |
