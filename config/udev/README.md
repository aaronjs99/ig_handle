# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| 99-ig-handle.rules | Assigns stable Prolific, serial-qualified Xsens, and Teensy device paths, permissions, and the Xsens low-latency serial setup. | Reviewed USB vendor, product, interface, and Xsens serial identities | Host udev installation, IG Handle sensor bringup |
