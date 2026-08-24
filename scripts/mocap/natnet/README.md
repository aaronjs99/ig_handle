# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| DataDescriptions.py | Defines NatNet model and data-description structures. | copy | NatNetClient.py |
| MoCapData.py | Defines NatNet frame-data structures. | copy | NatNetClient.py |
| NatNetClient.py | Receives and parses OptiTrack NatNet UDP descriptions and frame data. | socket, threading, struct, DataDescriptions.py, MoCapData.py | ../bridge.py |
| __init__.py | Marks the standard `mocap.natnet` transport package. | Python import system | ../bridge.py and NatNet module imports |
