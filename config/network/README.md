# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| 01-netplan.yaml | Host networkd template retaining all four sensor/boat subnets without carrier, including the `192.168.131.10` physical master. | Ubuntu netplan, sensor_network.yaml | Manual host network configuration |
| heron_ighandle_client.launch | Boat-side hardware-only composition for joining IGHandle's master without stock localization, model TF, or ROS command publishers. | Heron Kinetic hardware drivers, robot_upstart | `/etc/ros/kinetic/ros.d/base.launch` on Heron |
| deploy_heron_ighandle_client.sh | Prepares and validates the hardware launch, shared master-client module, existing startup script, and service restart override before installation; backs up and restores only its managed files without restarting ROS. | Heron Kinetic robot_upstart job, root maintenance window | Deliberate boat-side install and rollback |
| sensor_network.yaml | Canonical runtime endpoints, the physical IGHandle-owned graph, isolated simulation graph, mocap endpoints, and local interface roles. | Reviewed deployment addresses | scripts/sensors/network.py, persistent sensing services, GRANDE runner, and bringup |
