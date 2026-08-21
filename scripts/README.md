# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| external_sensor_provider.py | Supervises one contract-declared external sensor across device hotplug and physical ROS-master replacement while enforcing exact device, publisher, frame, stamp, and topic freshness contracts. | rospy, rospkg, sensor_msgs, ig_handle_runtime, roslaunch | systemd/ig-handle-xsens.service |
| network_config.py | Provides the `rosrun` command-line entrypoint for printing one canonical network-contract value. | ig_handle_runtime.network_config | operators, diagnostics |
| network_launch_eval.py | Exposes selected network-contract values to roslaunch substitutions without source-path injection or duplicated addresses. | ig_handle_runtime.network_config, config/network/sensor_network.yaml | GRANDE bringup, launch/core/natnet_bridge.launch |
| sensor_bringup.py | Selects, checks, starts, supervises, and reports contract-enabled physical sensor providers while observing externally owned providers without opening their devices. | rospy, rospkg, rosgraph, ig_handle_runtime | CMake installation, launch/sensors.launch |
| sensor_contract_launch_eval.py | Resolves sensor fields, endpoints, reachability, and provider options for safe roslaunch substitution. | ig_handle_runtime.sensor_contract, config/sensors/sensor_contract.yaml | GRANDE bringup, launch/sensors/start_sonar.launch |
| ig_handle_power/ | Installs the strict JK frame decoder and exact-device BlueZ read-query transport shared by the BMS node and tests. | Python stdlib, BlueZ D-Bus, GLib | power/jk_bms_node.py, catkin Python packaging |
| power/ | Owns the raw-Heron telemetry ingress and read-only IG Handle BMS acquisition boundary. | heron_msgs, sensor_msgs, BlueZ D-Bus, ig_handle_power | launch/core/start_power.launch, GRANDE bringup |
