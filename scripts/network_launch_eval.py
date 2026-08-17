# Roslaunch eval expression: resolve one checked-in network configuration value.
# This remains an expression because roslaunch loads it inline.
(
    lambda package_root, key, default="": __import__(
        "ig_handle_runtime.network_config", fromlist=["network_value"]
    ).network_value(
        key,
        package_root=package_root,
        default=default,
    )
)
