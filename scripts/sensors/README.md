# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| __init__.py | Declares the installed sensor-support package. | Python standard library | Python packaging |
| contracts.py | Loads, validates, and queries the canonical sensor lifecycle contract. | PyYAML | sensor_bringup.py, external_sensor_provider.py, and launch integration |
| external_sensor_provider.py | Owns serial-qualified Xsens under fixed integrated, standalone or isolated ground graph profiles; accepts roslaunch remappings and exits on master/device loss. | Sensor contract, ROS | Xsens service profiles and per-test integrated sensors.launch |
| network.py | Loads canonical host, interface, and sensor-network values. | PyYAML, rospkg | Sensor providers, mocap, sonar, and network_config.py |
| network_config.py | Prints one endpoint from the canonical network contract. | sensors.network | Services and deployment tools |
| parameters.py | Rejects ambiguous string and numeric stand-ins for boolean runtime parameters. | Python standard library | Sensor and transport nodes |
| ros_master.py | Binds registrations to a bounded master PID lease, optionally through a specified source address; missing or replaced masters require a fresh test. | Python stdlib XML-RPC | sensor_bringup.py, jk_bms_node.py, ground_capture.py, grande/run.py |
| sensor_bringup.py | Supervises contract-enabled sensors with exclusive physical ownership; permits restart when the expected local publisher's old registration refuses connections. Master loss/replacement exits 75 after tracked child cleanup. | Sensor contract, ROS, sensors.ros_master | Sensor launches and services |
