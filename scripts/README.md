# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| network_config.py | Read the IG Handle network endpoints. | argparse, pathlib, typing, yaml | grande/grande/docs/simulation_web_viz_runbook.md, grande/grande/tests/tools/run_harbor_controller_comparison.sh, ig_handle/CMakeLists.txt |
| network_launch_eval.py | Implements the network launch eval Python module. | None | grande/grande/launch/bringup.launch, ig_handle/CMakeLists.txt, ig_handle/launch/core/natnet_bridge.launch |
| sensor_bringup.py | Launch and supervise enabled sensors from the IG Handle sensor contract. | signal, subprocess, sys, time | ig_handle/CMakeLists.txt, ig_handle/launch/sensors.launch |
| sensor_contract.py | Read the IG Handle sensor contract for launch-time wiring. | os, json, subprocess, pathlib | ig_handle/CMakeLists.txt |
| sensor_contract_launch_eval.py | Implements the sensor contract launch eval Python module. | None | grande/grande/launch/bringup.launch, ig_handle/CMakeLists.txt, ig_handle/launch/sensors/start_sonar.launch |
