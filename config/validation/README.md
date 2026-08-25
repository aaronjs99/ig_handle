# Validation Fixtures

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| battery_registry_cases.yaml | Defines the expected inventory, roles, commissioning states, and deliberately unknown JK serial used to verify fail-closed registry behavior. It is an offline fixture and does not commission hardware. | Battery registry and JK configuration schemas | `scripts/power/validate_battery_registry.py` |
