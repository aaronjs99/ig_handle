# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| DataDescriptions.py | Implements the DataDescriptions Python module. | copy, hashlib, random | None |
| MoCapData.py | Implements the MoCapData Python module. | copy, hashlib, random | None |
| NatNetClient.py | Implements the NatNetClient Python module. | sys, socket, threading, struct | ig_handle/scripts/mocap/natnet/DataDescriptions.py, ig_handle/scripts/mocap/natnet/MoCapData.py |
| __init__.py | Implements the init Python module. | None | Python package imports |
