"""Heron base message health checks."""

from __future__ import annotations

from math import isfinite

from .env import float_env


def assert_heron_sense_is_healthy(msg) -> None:
    min_battery_v = float_env("IG_HANDLE_HERON_MIN_BATTERY_V", 14.0)
    values = (msg.battery, msg.current_left, msg.current_right)
    assert all(
        isfinite(value) for value in values
    ), "Heron /sense message contains non-finite numeric data"
    assert msg.battery >= min_battery_v, (
        f"Heron battery voltage is below the lab threshold: "
        f"{msg.battery:.3f} V < {min_battery_v:.3f} V"
    )


def assert_heron_status_is_healthy(msg) -> None:
    values = (
        msg.pcb_temperature,
        msg.user_current,
        msg.user_power_consumed,
        msg.motor_power_consumed,
        msg.total_power_consumed,
    )
    assert all(
        isfinite(value) for value in values
    ), "Heron /status message contains non-finite numeric data"
    assert (
        -20.0 <= msg.pcb_temperature <= 90.0
    ), f"Heron PCB temperature is implausible: {msg.pcb_temperature:.3f} C"
    assert (
        msg.user_current >= -0.1
    ), f"Heron user current is implausibly negative: {msg.user_current:.3f} A"
    assert msg.total_power_consumed >= 0.0, (
        "Heron total_power_consumed should be nonnegative: "
        f"{msg.total_power_consumed:.3f} Wh"
    )
