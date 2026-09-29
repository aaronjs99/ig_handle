# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| ig-handle-roscore-user.service | Owns the sole physical ROS master at `192.168.131.10:11311`; dependent sensor and GRANDE services are bound to its lifecycle and restart only their owned processes after replacement. | ROS Noetic, sensor_network.yaml | Persistent sensors and GRANDE infrastructure |
| ig-handle-sensor-bringup-user.service | Supervises physical sensor drivers against the IGHandle master; absent devices remain unavailable while other sensors continue. | Core and Xsens user services | Reboot-persistent acquisition |
| ig-handle-xsens-user.service | Runs the serial-qualified Xsens provider on the physical graph under the persistent `ig-handle` user manager. | ig-handle-roscore-user.service, systemd user manager with linger, ROS Noetic, dialout membership | Reboot-persistent IMU acquisition |
| ig-handle-xsens.service | System-level alternative for the same physical Xsens provider. It conflicts with the user service so only one can own the serial device. | systemd, ROS Noetic, IGHandle install, dialout group | Deliberate system-level deployment |
