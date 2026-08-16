# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| Linux_DeltaT_v1023_x86_64 | Provides the pinned vendor DeltaT executable installed in the standard package libexec directory and launched only by the configured DeltaT provider. | Compatible x86-64 host and physical DeltaT interface | CMake installation, include/deltat_runner.py |
| sonar.py | Dispatches the passive Imagenex UDP receiver, DeltaT process wrapper, or guarded UDP Ping360 provider from one ROS entrypoint. | include package, configured provider and network values | CMake installation, config/sensors/sonar/profiles.yaml, launch/sensors/start_sonar.launch |
