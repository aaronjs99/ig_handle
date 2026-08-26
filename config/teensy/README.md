# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| firmware_build.yaml | Records the exact reproducible Teensy 4.1 compile identity and artifact hashes while keeping flash, cold-start, runtime-identity, and electrical approvals false until measured. | scripts/teensy/build_firmware.sh, build/teensy41 artifacts | Teensy build and commissioning workflow, expected rosserial firmware identity |
| firmware_config.h | Defines the fixed V6 pin contract and disabled-by-default commissioning gates for DS3231-falling-edge one-shot PPS capture, boot-time RTC SQW shutdown when timing is disabled, inseparable dual-VLP PPS hardware fanout, field-valid-gated camera/LiDAR/MTi timing, firmware-inverted dual-VLP status-V NMEA with per-unit qualifier readback, and telescope motion including the tested hard-E-stop gate. | Official device electrical contracts, config/telescope/hardware.yaml | Teensy firmware build |
