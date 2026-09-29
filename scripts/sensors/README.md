# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| __init__.py | Declares the installed sensor-support package. | Python standard library | Python packaging |
| contracts.py | Loads, validates, and queries the canonical sensor lifecycle contract. | PyYAML | sensor_bringup.py, external_sensor_provider.py, and launch integration |
| external_sensor_provider.py | Owns serial-qualified Xsens on IGHandle's configured physical master; accepts roslaunch remappings and exits on master/device loss. | Sensor contract, ROS | Xsens services and explicitly owned sensors.launch |
| network.py | Loads canonical host, interface, and sensor-network values, and configures an explicitly requested Heron ROS endpoint. | PyYAML, rospkg, Python networking | Sensor providers, telemetry, mocap, sonar, and network_config.py |
| network_config.py | Prints one endpoint from the canonical network contract. | sensors.network | Services and deployment tools |
| parameters.py | Rejects ambiguous string and numeric stand-ins for boolean runtime parameters. | Python standard library | Sensor and transport nodes |
| process_identity.py | Reads immutable Linux process identity and parentage fields for safe process ownership checks. | pathlib | sensor_bringup.py and external_sensor_provider.py |
| ros_master.py | Binds registrations to a bounded PID lease with optional `/run_id` replacement detection. Its standalone client waits for the fixed master and exits for the existing Heron service to restart only its owned processes. | Python stdlib XML-RPC, Python 3.5 or later | Sensor and battery providers, ground_capture.py, grande/run.py, Heron startup job |
| sensor_bringup.py | Supervises contract-enabled sensors with exclusive physical ownership; permits restart when the expected local publisher's old registration refuses connections. Master loss/replacement exits 75 after tracked child cleanup. | Sensor contract, ROS, sensors.ros_master | Sensor launches and services |
