# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| network_config.py | Resolves the active source/install package and loads named ROS, mocap, sensor, and host-interface values from its canonical network contract as both a small CLI and an importable sibling module. | PyYAML, rospkg, config/network/sensor_network.yaml | CMake devel/install module layout, network_launch_eval.py, sensor runtime modules |
| network_launch_eval.py | Exposes selected network-contract values to roslaunch substitutions without duplicating addresses in launch XML. | network_config.py, config/network/sensor_network.yaml | GRANDE bringup, launch/core/natnet_bridge.launch |
| sensor_bringup.py | Selects, checks, starts, supervises, and reports contract-enabled physical sensor providers using installed sibling helpers instead of a source-checkout import bypass. | rospy, rospkg, network_config.py, sensor_contract.py | CMake installation, launch/sensors.launch |
| sensor_contract.py | Loads and validates deployed sensor records, bindings, roles, topics, provider selections, and launch arguments as an importable sibling module. | PyYAML, network_config.py, config/sensors/sensor_contract.yaml | CMake devel/install module layout, sensor_bringup.py, launch-time contract evaluation |
| sensor_contract_launch_eval.py | Resolves sensor fields, endpoints, reachability, and provider options for safe roslaunch substitution. | sensor_contract.py, config/sensors/sensor_contract.yaml | GRANDE bringup, launch/sensors/start_sonar.launch |
