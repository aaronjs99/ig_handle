# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| Linux_DeltaT_v1023_x86_64 | Provides the vendor DeltaT sonar executable used by the provider wrapper. | None | ig_handle/CMakeLists.txt, ig_handle/scripts/sonar/include/deltat_runner.py |
| sonar.py | Single entrypoint for passive UDP, DeltaT, and UDP Ping360 provider modes. | sys, pathlib, typing, include | ig_handle/CMakeLists.txt, ig_handle/config/sensors/sonar/profiles.yaml, ig_handle/launch/sensors/start_sonar.launch |
