# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| 01-netplan.yaml | Host netplan template for the Heron, sensor, and sonar interfaces; its sonar subnet must be reconciled before deployment. | Ubuntu netplan, sensor_network.yaml | Manual host network configuration |
| sensor_network.yaml | Canonical runtime endpoints, ROS graph addresses, mocap endpoints, and local interface roles. | Reviewed deployment addresses | scripts/network_config.py, sensor-contract launch evaluation, GRANDE runner and bringup |
