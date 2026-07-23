# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| broadcast.py | Publish selected Heron sensor TF edges from the shared YAML config. | math, rospy, tf2_ros, geometry_msgs | grande/grande/launch/include/tf_static.launch, ig_handle/CMakeLists.txt |
| export.py | Export canonical Heron sensor extrinsics for launch-time consumers. | argparse, json, math, os | ig_handle/CMakeLists.txt, heron_simulator/urdf/sensors.urdf.xacro |
