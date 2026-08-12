# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| profiles.yaml | Defines DeltaT pool/harbor range, gain, and sound speed plus the fail-closed expected 83P wire profile (480 beams, 120-degree sector, high resolution, intensity, and 240 kHz). | Deployment-environment assumptions and Imagenex 83P v1.10 | config/sensors/sensor_contract.yaml, scripts/sonar/include/deltat_runner.py, MARINER sonar_raw_to_cloud.py |
