# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| external_sensor_provider.py | Supervises one contract-declared external sensor across device hotplug and physical ROS-master replacement while enforcing exact device, publisher, frame, stamp, and topic freshness contracts. | rospy, rospkg, sensor_msgs, sensors, roslaunch | Exactly one documented system or user Xsens service template |
| network_config.py | Provides the `rosrun` command-line entrypoint for printing one canonical network-contract value. | sensors.network | operators, diagnostics |
| sensor_bringup.py | Selects, checks, starts, supervises, and reports contract-enabled physical sensor providers while observing externally owned providers without opening their devices. | rospy, rospkg, rosgraph, sensors | CMake installation, launch/sensors.launch |
