# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| 01-netplan.yaml | Host netplan template for the deployed `192.168.131.10` Heron interface, sensor subnet, and canonical 192.168.2.0/24 sonar interface. | Ubuntu netplan, sensor_network.yaml | Manual host network configuration |
| heron_ighandle_client.launch | Boat-side hardware-only composition for joining IGHandle's master without stock localization, model TF, or ROS command publishers. | Heron Kinetic hardware drivers, robot_upstart | `/etc/ros/kinetic/ros.d/base.launch` on Heron |
| sensor_network.yaml | Canonical runtime endpoints, the physical IGHandle-owned graph, isolated simulation graph, mocap endpoints, and local interface roles. | Reviewed deployment addresses | scripts/sensors/network.py, persistent sensing services, GRANDE runner, and bringup |
