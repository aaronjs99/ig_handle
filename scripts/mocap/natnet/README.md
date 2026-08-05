# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| DataDescriptions.py | Defines NatNet model and data-description structures. | copy, hashlib, random | NatNetClient.py |
| MoCapData.py | Defines NatNet frame-data structures. | copy, hashlib, random | NatNetClient.py |
| NatNetClient.py | Receives and parses OptiTrack NatNet UDP descriptions and frame data. | socket, threading, struct, DataDescriptions.py, MoCapData.py | ../mocap.py |
| __init__.py | Implements the init Python module. | None | Python package imports |
