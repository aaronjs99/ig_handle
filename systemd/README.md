# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| ig-handle-roscore-user.service | Owns the independent local ROS master and validates the configured network interface. | ROS Noetic, sensor_network.yaml | Standalone battery and optional standalone sensor services |
| ig-handle-sensor-bringup-user.service | Supervises the standalone sensor profile; mutually exclusive with integrated physical sensing. | Independent core and Xsens user services | Explicit standalone deployment |
| ig-handle-xsens-user.service | Runs the serial-qualified Xsens provider in the fixed standalone profile under the persistent `ig-handle` user manager. | ig-handle-roscore-user.service, systemd user manager with linger, ROS Noetic, dialout membership | Reboot-persistent standalone physical sensing |
| ig-handle-xsens.service | Runs the serial-qualified Xsens provider in the explicit integrated profile against the physical Heron ROS master. | systemd, ROS Noetic, IG Handle install or workspace overlay, dialout group | Manual integrated GRANDE sensing |
