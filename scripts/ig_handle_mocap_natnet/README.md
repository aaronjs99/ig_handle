# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| DataDescriptions.py | Defines NatNet model and data-description structures. | copy, hashlib, random | NatNetClient.py |
| MoCapData.py | Defines NatNet frame-data structures. | copy, hashlib, random | NatNetClient.py |
| NatNetClient.py | Receives and parses OptiTrack NatNet UDP descriptions and frame data. | socket, threading, struct, DataDescriptions.py, MoCapData.py | ../mocap/mocap.py |
| __init__.py | Marks the standard `ig_handle_mocap_natnet` transport package. | Python import system | ../mocap/mocap.py and NatNet module imports |
