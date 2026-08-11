# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| broadcast.py | Publishes only the selected configured static sensor TF edges and rejects unknown transform names. | rospy, tf2_ros, geometry_msgs, config/sensors/sensor_frames.yaml | GRANDE static-TF launch, CMake installation |
| export.py | Validates and exports canonical sensor extrinsics as JSON or launch-ready transform arguments. | PyYAML, config/sensors/sensor_frames.yaml | CMake installation, Heron Simulator sensor URDF generation |
