# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| __init__.py | Sonar runtime implementation modules. | None | Python package imports |
| deltat_runner.py | DeltaT binary launcher and runtime INI generation. | argparse, os, sys, dataclasses | None |
| ping360_provider.py | Blue Robotics Ping360 UDP provider with a read-only identity mode. | hashlib, json, socket, struct | scripts/sonar/sonar.py, launch/sensors/start_sonar.launch |
| ping_protocol.py | Pure Ping Protocol framing and Ping360 profile parsing. | math, struct, dataclasses, typing | None |
| profiles.py | Load configured sonar runtime profiles. | collections, dataclasses, pathlib, typing | heron_simulator/CMakeLists.txt |
| receiver.py | Raw UDP receiver for Imagenex sonar datagrams. | socket, dataclasses, typing, rospy | None |
