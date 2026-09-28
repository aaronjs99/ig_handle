# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| natnet_bridge.launch | Selects NatNet or DataCollect UDP mocap transport through explicit launch arguments and publishes canonical mocap topics with stale-data handling. | scripts/mocap/bridge.py | GRANDE bringup and mocap-assisted state review |
| start_power.launch | Establishes the IG Handle-owned `/sense_heron` ingress and conditionally composes the public read-only battery provider after its commissioned identity resolves through the physical registry. | launch/battery.launch, scripts/power/, config/runtime_surface.yaml | GRANDE bringup |
| start_rosserial.launch | Starts the conditional Teensy launcher and forwards its configured diagnostic topic names to rosserial. | /dev/teensy, commissioned firmware build ID, scripts/teensy/teensy_rosserial_launcher.py | GRANDE bringup and embedded timing acquisition |
| start_timing_event_adapter.launch | Records Spinnaker frame counters and device clocks whenever real Heron cameras are available; optionally adds identified Teensy PPS, exposure, and IMU-sync events when the bridge is enabled. An explicit offline affine fit may be supplied; otherwise events remain unmapped, and no clocks are paired by receipt-time proximity. | scripts/teensy/timing_event_adapter.py, CameraFrameTiming, optional FirmwareTimingEvent | GRANDE real-Heron bringup |
