#!/usr/bin/env python3
"""Publish read-only JK BMS telemetry as standard and detailed ROS messages."""

import math
import threading
import time
from typing import Optional

import rospy
from sensor_msgs.msg import BatteryState

from ig_handle.msg import JkBmsDetails
from power.bluez_ble import BluezBleClient, BluezError
from power.jk_bms_protocol import (
    DeviceInfo,
    FrameAssembler,
    JkBmsDecoder,
    ProtocolError,
    Telemetry,
    build_query,
)


def _required_text(name: str) -> str:
    value = str(rospy.get_param("~" + name, "")).strip()
    if not value:
        raise ValueError("{} must be configured".format(name))
    return value


def _required_bool(name: str) -> bool:
    value = rospy.get_param("~" + name, None)
    if type(value) is not bool:
        raise ValueError("{} must be a native YAML boolean".format(name))
    return value


def _required_int(name: str) -> int:
    value = rospy.get_param("~" + name, None)
    if type(value) is not int:
        raise ValueError("{} must be a native YAML integer".format(name))
    return value


def _required_float(name: str) -> float:
    value = rospy.get_param("~" + name, None)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("{} must be a finite YAML number".format(name))
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("{} must be a finite YAML number".format(name))
    return value


class JkBmsNode:
    def __init__(self) -> None:
        if _required_int("schema_version") != 1:
            raise ValueError("unsupported JK BMS configuration schema")
        if not _required_bool("enabled"):
            raise ValueError("JK BMS node must not start while enabled is false")
        if not _required_bool("read_only"):
            raise ValueError("JK BMS integration is read-only by contract")
        self.provider = _required_text("provider")
        self.transport = _required_text("transport")
        if self.transport != "ble":
            raise ValueError("only the commissioned BLE transport is supported")
        self.protocol = _required_text("protocol")
        self.expected_device_name = _required_text("expected_device_name")
        self.expected_model = _required_text("expected_model")
        self.expected_serial = _required_text("expected_serial_number")
        self.expected_manufacturing_date = _required_text("expected_manufacturing_date")
        self.expected_hardware = str(
            rospy.get_param("~expected_hardware_version", "")
        ).strip()
        self.expected_software = str(
            rospy.get_param("~expected_software_version", "")
        ).strip()
        self.device_address = _required_text("device_address").upper()
        self.service_uuid = _required_text("service_uuid").lower()
        self.characteristic_uuid = _required_text("characteristic_uuid").lower()
        self.frame_id = _required_text("frame_id")
        self.chemistry = _required_text("chemistry")
        self.design_capacity_ah = _required_float("design_capacity_ah")
        self.sample_timeout_sec = _required_float("sample_timeout_sec")
        if self.design_capacity_ah <= 0.0 or self.sample_timeout_sec <= 0.0:
            raise ValueError("capacity and sample timeout must be positive")
        expected_cells = _required_int("expected_cell_count")
        self.decoder = JkBmsDecoder(
            self.protocol,
            expected_cells,
            cell_voltage_bounds_v=(
                _required_float("cell_voltage_min_v"),
                _required_float("cell_voltage_max_v"),
            ),
            pack_voltage_bounds_v=(
                _required_float("pack_voltage_min_v"),
                _required_float("pack_voltage_max_v"),
            ),
            temperature_bounds_c=(
                _required_float("temperature_min_c"),
                _required_float("temperature_max_c"),
            ),
            pack_cell_sum_tolerance_v=_required_float("pack_cell_sum_tolerance_v"),
        )
        self.battery_publisher = rospy.Publisher(
            str(rospy.get_param("~battery_topic", "/sense_ighandle")),
            BatteryState,
            queue_size=10,
        )
        self.details_publisher = rospy.Publisher(
            str(rospy.get_param("~details_topic", "/sense_ighandle/details")),
            JkBmsDetails,
            queue_size=10,
            latch=True,
        )
        self.assembler = FrameAssembler()
        self._lock = threading.Lock()
        self._publication_lock = threading.Lock()
        self._device_info: Optional[DeviceInfo] = None
        self._last_valid_monotonic: Optional[float] = None
        self._last_sequence = 0
        self._rx_frames = 0
        self._rx_errors = 0
        self._valid = False
        self._validity_reason = ""
        self._publish_unavailable("starting")
        self.client = BluezBleClient(
            adapter=_required_text("adapter"),
            device_address=self.device_address,
            expected_device_name=self.expected_device_name,
            service_uuid=self.service_uuid,
            characteristic_uuid=self.characteristic_uuid,
            connect_timeout_sec=_required_float("connect_timeout_sec"),
            sample_timeout_sec=self.sample_timeout_sec,
            initial_query_delay_sec=_required_float("initial_query_delay_sec"),
            request_period_sec=_required_float("request_period_sec"),
            reconnect_delay_sec=_required_float("reconnect_delay_sec"),
            on_notification=self._notification,
            on_state=self._connection_state,
        )
        self.worker = threading.Thread(
            target=self.client.run,
            args=((build_query(0x97),), (build_query(0x96),)),
            name="jk_bms_ble",
            daemon=True,
        )
        self.worker.start()
        rospy.on_shutdown(self.client.stop)
        self.watchdog = rospy.Timer(rospy.Duration(0.5), self._watchdog)

    def _connection_state(self, connected: bool, reason: str) -> None:
        if not connected:
            with self._lock:
                self._device_info = None
            self._publish_unavailable(reason)

    def _notification(self, chunk: bytes) -> None:
        try:
            for frame in self.assembler.feed(chunk):
                self._rx_frames += 1
                if frame[4] == 0x03:
                    info = self.decoder.decode_device_info(frame)
                    self._admit_identity(info)
                elif frame[4] == 0x02:
                    telemetry = self.decoder.decode_telemetry(frame)
                    self._publish_telemetry(telemetry)
        except (ProtocolError, ValueError) as exc:
            self._rx_errors += 1
            self._publish_unavailable("protocol_error:{}".format(str(exc)))

    def _admit_identity(self, info: DeviceInfo) -> None:
        # Every connection must establish its own identity. Clear the previous
        # admission before checking a new response so rejected hardware can
        # never inherit identity from an earlier connection.
        with self._lock:
            self._device_info = None
        if info.model != self.expected_model:
            raise ProtocolError("JK model does not match commissioned identity")
        if info.device_name != self.expected_device_name:
            raise ProtocolError("JK device name does not match commissioned identity")
        if info.serial_number != self.expected_serial:
            raise ProtocolError("JK serial number does not match commissioned identity")
        if info.manufacturing_date != self.expected_manufacturing_date:
            raise ProtocolError(
                "JK manufacturing date does not match commissioned identity"
            )
        if self.expected_hardware and info.hardware_version != self.expected_hardware:
            raise ProtocolError("JK hardware version does not match configuration")
        if self.expected_software and info.software_version != self.expected_software:
            raise ProtocolError("JK software version does not match configuration")
        with self._lock:
            self._device_info = info

    def _publish_telemetry(self, telemetry: Telemetry) -> None:
        with self._publication_lock:
            with self._lock:
                info = self._device_info
            if info is None:
                raise ProtocolError("telemetry arrived before admitted device identity")
            now = rospy.Time.now()
            battery = BatteryState()
            battery.header.stamp = now
            battery.header.frame_id = self.frame_id
            battery.voltage = telemetry.pack_voltage_v
            battery.temperature = max(
                (*telemetry.temperatures_c, telemetry.mos_temperature_c)
            )
            battery.current = telemetry.current_a
            battery.charge = telemetry.remaining_capacity_ah
            battery.capacity = telemetry.nominal_capacity_ah
            battery.design_capacity = self.design_capacity_ah
            battery.percentage = telemetry.state_of_charge
            if telemetry.current_a > 0.05:
                battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
            elif telemetry.current_a < -0.05:
                battery.power_supply_status = (
                    BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
                )
            else:
                battery.power_supply_status = (
                    BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
                )
            battery.power_supply_health = (
                BatteryState.POWER_SUPPLY_HEALTH_GOOD
                if not telemetry.has_critical_alarm
                else BatteryState.POWER_SUPPLY_HEALTH_UNSPEC_FAILURE
            )
            battery.power_supply_technology = self._technology()
            battery.present = True
            battery.cell_voltage = list(telemetry.cell_voltages_v)
            battery.cell_temperature = []
            battery.location = "ig_handle"
            battery.serial_number = info.serial_number
            details = self._details_message(now, info, telemetry)
            with self._lock:
                self._last_valid_monotonic = time.monotonic()
                self._last_sequence = telemetry.sequence
                self._valid = True
                self._validity_reason = "valid"
            self.battery_publisher.publish(battery)
            self.details_publisher.publish(details)

    def _technology(self) -> int:
        if self.chemistry in ("ncm", "nca", "ncm_nca", "li_ion"):
            return BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        if self.chemistry == "lifepo4":
            return BatteryState.POWER_SUPPLY_TECHNOLOGY_LIFE
        if self.chemistry == "lipo":
            return BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        return BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN

    def _details_message(
        self, stamp: rospy.Time, info: DeviceInfo, telemetry: Telemetry
    ) -> JkBmsDetails:
        message = self._base_details(stamp, True, "valid")
        message.model = info.model
        message.hardware_version = info.hardware_version
        message.software_version = info.software_version
        message.serial_number = info.serial_number
        message.device_name = info.device_name
        message.manufacturing_date = info.manufacturing_date
        message.cell_count = len(telemetry.cell_voltages_v)
        message.pack_voltage_v = telemetry.pack_voltage_v
        message.pack_current_a = telemetry.current_a
        message.pack_power_w = telemetry.pack_power_w
        message.state_of_charge = telemetry.state_of_charge
        message.state_of_health = telemetry.state_of_health
        message.remaining_capacity_ah = telemetry.remaining_capacity_ah
        message.nominal_capacity_ah = telemetry.nominal_capacity_ah
        message.cell_voltages_v = list(telemetry.cell_voltages_v)
        message.temperature_sensors_c = list(telemetry.temperatures_c)
        message.mos_temperature_c = telemetry.mos_temperature_c
        message.average_cell_voltage_v = telemetry.average_cell_voltage_v
        message.delta_cell_voltage_v = telemetry.delta_cell_voltage_v
        message.balancing_current_a = telemetry.balancing_current_a
        message.cell_resistances_ohm = list(telemetry.cell_resistances_ohm)
        message.cycle_count = telemetry.cycle_count
        message.cycle_capacity_ah = telemetry.cycle_capacity_ah
        message.runtime_sec = telemetry.runtime_sec
        message.power_on_count = info.power_on_count
        message.alarm_flags = telemetry.alarm_flags
        message.active_alarms = list(telemetry.active_alarms)
        message.charge_enabled = telemetry.charge_enabled
        message.discharge_enabled = telemetry.discharge_enabled
        message.balancing_active = telemetry.balancing_active
        message.precharge_active = telemetry.precharge_active
        message.heating_active = telemetry.heating_active
        message.sequence = telemetry.sequence
        return message

    def _base_details(
        self, stamp: rospy.Time, valid: bool, reason: str
    ) -> JkBmsDetails:
        message = JkBmsDetails()
        message.header.stamp = stamp
        message.header.frame_id = self.frame_id
        message.valid = valid
        message.validity_reason = reason
        message.synthetic = False
        message.provider = self.provider
        message.transport = self.transport
        message.device_address = self.device_address
        message.protocol = self.protocol
        message.service_uuid = self.service_uuid
        message.characteristic_uuid = self.characteristic_uuid
        message.pack_voltage_v = math.nan
        message.pack_current_a = math.nan
        message.pack_power_w = math.nan
        message.state_of_charge = math.nan
        message.state_of_health = math.nan
        message.remaining_capacity_ah = math.nan
        message.nominal_capacity_ah = math.nan
        message.mos_temperature_c = math.nan
        message.average_cell_voltage_v = math.nan
        message.delta_cell_voltage_v = math.nan
        message.balancing_current_a = math.nan
        message.cycle_capacity_ah = math.nan
        message.rx_frame_count = self._rx_frames
        message.rx_error_count = self._rx_errors
        message.sequence = self._last_sequence
        return message

    def _publish_unavailable(
        self, reason: str, *, stale_before: Optional[float] = None
    ) -> None:
        with self._publication_lock:
            with self._lock:
                if stale_before is not None and (
                    self._last_valid_monotonic is None
                    or self._last_valid_monotonic > stale_before
                ):
                    return
                if not self._valid and reason == self._validity_reason:
                    return
                self._valid = False
                self._validity_reason = reason
                info = self._device_info
            stamp = rospy.Time.now()
            battery = BatteryState()
            battery.header.stamp = stamp
            battery.header.frame_id = self.frame_id
            battery.voltage = math.nan
            battery.temperature = math.nan
            battery.current = math.nan
            battery.charge = math.nan
            battery.capacity = math.nan
            battery.design_capacity = self.design_capacity_ah
            battery.percentage = math.nan
            battery.present = False
            battery.location = "ig_handle"
            message = self._base_details(stamp, False, reason)
            if info is not None:
                message.model = info.model
                message.hardware_version = info.hardware_version
                message.software_version = info.software_version
                message.serial_number = info.serial_number
                message.device_name = info.device_name
                message.manufacturing_date = info.manufacturing_date
                message.power_on_count = info.power_on_count
            self.battery_publisher.publish(battery)
            self.details_publisher.publish(message)

    def _watchdog(self, _event) -> None:
        stale_before = time.monotonic() - self.sample_timeout_sec
        self._publish_unavailable("telemetry_stale", stale_before=stale_before)


def main() -> None:
    rospy.init_node("ighandle_jk_bms")
    try:
        JkBmsNode()
    except (BluezError, KeyError, ProtocolError, TypeError, ValueError) as exc:
        rospy.logfatal("Invalid JK BMS configuration: %s", exc)
        raise SystemExit(2)
    rospy.spin()


if __name__ == "__main__":
    main()
