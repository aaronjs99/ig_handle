# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| ig-handle-roscore-user.service | Owns the sole physical ROS master at `192.168.131.10:11311`; dependent sensor and GRANDE services are bound to its lifecycle and enabled in its `.wants/` directory for recovery after replacement. Unexpected clean exits restart; an explicit service stop remains stopped. | ROS Noetic, sensor_network.yaml | Persistent sensors and GRANDE infrastructure |
| ig-handle-sensor-bringup-user.service | Supervises physical sensor drivers against the IGHandle master; absent devices remain unavailable. Restarts after an unexpected roslaunch exit, including required-child failures reported as success. | Core and Xsens user services | Reboot-persistent acquisition |
| ig-handle-xsens-user.service | Runs the serial-qualified Xsens provider on the physical graph under the persistent `ig-handle` user manager. | ig-handle-roscore-user.service, systemd user manager with linger, ROS Noetic, dialout membership | Reboot-persistent IMU acquisition |
| ig-handle-xsens.service | System-level alternative for the same physical Xsens provider, selected deliberately after stopping the user alternative. A shared same-user provider lock prevents duplicate ownership; user-manager Conflicts cannot stop a system-manager unit. | systemd, ROS Noetic, IGHandle install, dialout group | Deliberate system-level deployment |
