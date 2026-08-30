# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| `__init__.py` | Declares the installed sensor-support package. | None | Python packaging |
| `contracts.py` | Loads, validates, and queries the canonical sensor lifecycle contract. | PyYAML, rospkg | sensor_bringup.py, external_sensor_provider.py, and launch integration |
| `imu_candidate_validation.py` | Provides reusable, fail-closed identity, provenance, units, finiteness, symmetry, positive-semidefinite, and standard-deviation consistency checks for offline stationary IMU candidate artifacts. | NumPy, PyYAML | IMU candidate validator |
| `network.py` | Loads and validates canonical host, interface, and sensor-network values. | PyYAML, rospkg | Sensor providers, mocap, sonar, and network entrypoints |
| `parameters.py` | Rejects ambiguous string and numeric stand-ins for boolean runtime parameters. | Python standard library | Sensor and transport nodes |
