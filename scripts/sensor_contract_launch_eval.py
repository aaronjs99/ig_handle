# Roslaunch eval expression: resolve one sensor-contract launch value.
# This remains an expression because roslaunch loads it inline.
(
    lambda package_root, command, sensor_id="", field="", default="", contract_file="", extra_sensor_ids="", disabled_sensor_ids="", reachability_check="true": __import__(
        "ig_handle_runtime.sensor_contract", fromlist=["launch_value"]
    ).launch_value(
        package_root,
        command,
        sensor_id,
        field,
        default,
        contract_file,
        extra_sensor_ids,
        disabled_sensor_ids,
        reachability_check,
    )
)
