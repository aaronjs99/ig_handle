# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| ig-handle-xsens.service | Runs the serial-qualified external Xsens provider as the single reboot-persistent owner of sensor contract ID 1. | systemd, ROS Noetic, IG Handle install or workspace overlay, dialout group | IG Handle host boot and GRANDE physical sensor readiness |
| ig-handle-xsens-user.service | Provides the same single-owner contract through the persistent `ig-handle` user manager when system-unit installation is unavailable. Install exactly one of the two units; user persistence requires linger. | systemd user manager, linger, ROS Noetic, IG Handle install or workspace overlay, dialout membership | IG Handle host boot and GRANDE physical sensor readiness |
