# Roslaunch eval expression: resolve one sensor-contract launch value.
# This remains an expression because roslaunch loads it inline.
(
    lambda package_root, command, sensor_id="", field="", default="", contract_file="", extra_sensor_ids="", disabled_sensor_ids="", reachability_check="true": (
        (
            __import__("sys").path.insert(
                0, __import__("os").path.join(package_root, "scripts")
            )
            if __import__("os").path.join(package_root, "scripts")
            not in __import__("sys").path
            else None
        ),
        __import__("sensor_contract", fromlist=["launch_value"]).launch_value(
            package_root,
            command,
            sensor_id,
            field,
            default,
            contract_file,
            extra_sensor_ids,
            disabled_sensor_ids,
            reachability_check,
        ),
    )[
        -1
    ]
)
