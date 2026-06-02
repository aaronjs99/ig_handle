(
    lambda package_root, key, default="": (
        (
            __import__("sys").path.insert(
                0, __import__("os").path.join(package_root, "scripts")
            )
            if __import__("os").path.join(package_root, "scripts")
            not in __import__("sys").path
            else None
        ),
        __import__("network_config", fromlist=["network_value"]).network_value(
            key,
            package_root=package_root,
            default=default,
        ),
    )[-1]
)
