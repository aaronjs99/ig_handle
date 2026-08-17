"""Strict reusable runtime-parameter parsing for IG Handle."""


def strict_bool(value, *, name="value"):
    """Return a real boolean and reject ambiguous ROS/YAML coercions."""
    if not isinstance(value, bool):
        raise ValueError("{} must be a boolean".format(name))
    return value
