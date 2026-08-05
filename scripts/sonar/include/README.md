# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| __init__.py | Sonar runtime implementation modules. | None | Python package imports |
| deltat_runner.py | DeltaT binary launcher and runtime INI generation. | argparse, os, sys, dataclasses, profiles.py | ../sonar.py |
| ping360_provider.py | Blue Robotics Ping360 UDP provider with a read-only identity mode. | hashlib, json, socket, struct | scripts/sonar/sonar.py, launch/sensors/start_sonar.launch |
| ping_protocol.py | Pure Ping Protocol framing and Ping360 profile parsing. | math, struct, dataclasses, typing | ping360_provider.py |
| profiles.py | Loads configured DeltaT sonar runtime profiles and network values. | collections, dataclasses, pathlib, typing, yaml, network_config.py | deltat_runner.py |
| receiver.py | Receives Imagenex sonar datagrams over UDP and publishes raw bytes. | socket, dataclasses, typing, rospy, network_config.py | ../sonar.py |
