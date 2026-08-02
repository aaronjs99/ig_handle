# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| network_config.py | Reads configured IG Handle and Heron network endpoints. | argparse, pathlib, typing, yaml | ig_handle/CMakeLists.txt |
| network_launch_eval.py | Implements the network launch eval Python module. | None | grande/grande/launch/bringup.launch, ig_handle/CMakeLists.txt, ig_handle/launch/core/natnet_bridge.launch |
| sensor_bringup.py | Launch and supervise enabled sensors from the IG Handle sensor contract. | signal, subprocess, sys, time | ig_handle/CMakeLists.txt, ig_handle/launch/sensors.launch |
| sensor_contract.py | Read the IG Handle sensor contract for launch-time wiring. | os, json, subprocess, pathlib | ig_handle/CMakeLists.txt |
| sensor_contract_launch_eval.py | Implements the sensor contract launch eval Python module. | None | grande/grande/launch/bringup.launch, ig_handle/CMakeLists.txt, ig_handle/launch/sensors/start_sonar.launch |
