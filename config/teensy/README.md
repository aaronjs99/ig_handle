# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| firmware_build.yaml | Records the exact reproducible Teensy 4.1 compile identity and artifact hashes while keeping flash, cold-start, runtime-identity, and electrical approvals false until measured. | scripts/teensy/build_firmware.sh, build/teensy41 artifacts | Tuesday commissioning checklist, expected rosserial firmware identity |
| firmware_config.h | Defines disabled-by-default board pins, polarities, reference qualification, four-camera trigger/feedback, dual-VLP clock, MTi sync, and telescope constants. | Official device electrical contracts, config/telescope/hardware.yaml | Teensy firmware build |
