#!/usr/bin/env python3
"""Small BlueZ D-Bus central used only for read-only JK telemetry queries."""

import re
import threading
import time
from typing import Callable, Dict, Optional, Tuple

import dbus
from dbus.mainloop.glib import DBusGMainLoop
from gi.repository import GLib


BLUEZ = "org.bluez"
OBJECT_MANAGER = "org.freedesktop.DBus.ObjectManager"
PROPERTIES = "org.freedesktop.DBus.Properties"
ADAPTER = "org.bluez.Adapter1"
DEVICE = "org.bluez.Device1"
CHARACTERISTIC = "org.bluez.GattCharacteristic1"


class BluezError(RuntimeError):
    """Raised when discovery, identity, GATT, or connection checks fail."""


def canonical_address(value: str) -> str:
    value = str(value).strip().upper()
    if not re.fullmatch(r"(?:[0-9A-F]{2}:){5}[0-9A-F]{2}", value):
        raise BluezError("device_address must be an exact Bluetooth MAC address")
    return value


class BluezBleClient:
    """One-device BlueZ client with exact address/name/service admission."""

    def __init__(
        self,
        *,
        adapter: str,
        device_address: str,
        expected_device_name: str,
        service_uuid: str,
        characteristic_uuid: str,
        connect_timeout_sec: float,
        sample_timeout_sec: float,
        initial_query_delay_sec: float,
        request_period_sec: float,
        reconnect_delay_sec: float,
        on_notification: Callable[[bytes], None],
        on_state: Callable[[bool, str], None],
    ) -> None:
        DBusGMainLoop(set_as_default=True)
        self.address = canonical_address(device_address)
        self.expected_device_name = str(expected_device_name).strip()
        self.service_uuid = service_uuid.lower()
        self.characteristic_uuid = characteristic_uuid.lower()
        self.connect_timeout_sec = float(connect_timeout_sec)
        self.sample_timeout_sec = float(sample_timeout_sec)
        self.initial_query_delay_sec = float(initial_query_delay_sec)
        self.request_period_sec = float(request_period_sec)
        self.reconnect_delay_sec = float(reconnect_delay_sec)
        if (
            min(
                self.connect_timeout_sec,
                self.sample_timeout_sec,
                self.initial_query_delay_sec,
                self.request_period_sec,
                self.reconnect_delay_sec,
            )
            <= 0.0
        ):
            raise BluezError("all BLE timing parameters must be positive")
        self.on_notification = on_notification
        self.on_state = on_state
        self.adapter_path = "/org/bluez/{}".format(str(adapter).strip())
        self.device_path = "{}/dev_{}".format(
            self.adapter_path, self.address.replace(":", "_")
        )
        self.bus = dbus.SystemBus()
        self.manager = dbus.Interface(self.bus.get_object(BLUEZ, "/"), OBJECT_MANAGER)
        self._stop = threading.Event()
        self._last_notification = 0.0
        self._notify_path = ""
        self._signal_match = None
        self._main_loop = GLib.MainLoop()
        self._main_loop_thread = threading.Thread(
            target=self._main_loop.run, name="bluez_glib", daemon=True
        )
        self._main_loop_thread.start()

    def stop(self) -> None:
        self._stop.set()
        try:
            self._main_loop.quit()
        except Exception:
            pass

    def run(
        self,
        initial_queries: Tuple[bytes, ...],
        periodic_queries: Tuple[bytes, ...],
    ) -> None:
        while not self._stop.is_set():
            try:
                self._run_connection(initial_queries, periodic_queries)
            except Exception as exc:
                self.on_state(False, "ble_error:{}".format(type(exc).__name__))
            finally:
                self._disconnect()
            self._stop.wait(self.reconnect_delay_sec)

    def _objects(self) -> Dict[str, Dict[str, Dict[str, object]]]:
        return dict(self.manager.GetManagedObjects())

    def _wait_for_device(self) -> None:
        adapter = dbus.Interface(self.bus.get_object(BLUEZ, self.adapter_path), ADAPTER)
        deadline = time.monotonic() + self.connect_timeout_sec
        discovering = False
        try:
            if self.device_path not in self._objects():
                adapter.StartDiscovery()
                discovering = True
            while self.device_path not in self._objects():
                if self._stop.is_set() or time.monotonic() >= deadline:
                    raise BluezError("configured BMS address was not discovered")
                time.sleep(0.1)
        finally:
            if discovering:
                try:
                    adapter.StopDiscovery()
                except dbus.DBusException:
                    pass

    def _run_connection(
        self,
        initial_queries: Tuple[bytes, ...],
        periodic_queries: Tuple[bytes, ...],
    ) -> None:
        self._wait_for_device()
        device_object = self.bus.get_object(BLUEZ, self.device_path)
        device = dbus.Interface(device_object, DEVICE)
        properties = dbus.Interface(device_object, PROPERTIES)
        device.Connect()
        deadline = time.monotonic() + self.connect_timeout_sec
        while not bool(properties.Get(DEVICE, "ServicesResolved")):
            if self._stop.is_set() or time.monotonic() >= deadline:
                raise BluezError("BMS GATT services were not resolved")
            time.sleep(0.1)
        if self.expected_device_name:
            try:
                name = str(properties.Get(DEVICE, "Name"))
            except dbus.DBusException:
                name = str(properties.Get(DEVICE, "Alias"))
            if name != self.expected_device_name:
                raise BluezError("BMS Bluetooth name does not match configuration")
        write_path, notify_path, write_kind = self._find_characteristics()
        self._notify_path = notify_path
        self._signal_match = self.bus.add_signal_receiver(
            self._properties_changed,
            signal_name="PropertiesChanged",
            dbus_interface=PROPERTIES,
            path_keyword="path",
        )
        notify = dbus.Interface(self.bus.get_object(BLUEZ, notify_path), CHARACTERISTIC)
        writer = dbus.Interface(self.bus.get_object(BLUEZ, write_path), CHARACTERISTIC)
        notify.StartNotify()
        self._last_notification = time.monotonic()
        self._write_queries(writer, write_kind, initial_queries)
        if self._stop.wait(self.initial_query_delay_sec):
            return
        self._write_queries(writer, write_kind, periodic_queries)
        self.on_state(True, "connected")
        next_request = time.monotonic() + self.request_period_sec
        while not self._stop.wait(0.1):
            if not bool(properties.Get(DEVICE, "Connected")):
                raise BluezError("BMS disconnected")
            now = time.monotonic()
            if now - self._last_notification > self.sample_timeout_sec:
                raise BluezError("BMS telemetry timeout")
            if now >= next_request:
                self._write_queries(writer, write_kind, periodic_queries)
                next_request = now + self.request_period_sec

    @staticmethod
    def _write_queries(writer, write_kind: str, queries: Tuple[bytes, ...]) -> None:
        for query in queries:
            writer.WriteValue(
                dbus.Array(query, signature="y"),
                dbus.Dictionary({"type": write_kind}, signature="sv"),
            )
            time.sleep(0.15)

    def _find_characteristics(self) -> Tuple[str, str, str]:
        objects = self._objects()
        service_paths = {
            str(path)
            for path, interfaces in objects.items()
            if str(path).startswith(self.device_path + "/")
            and interfaces.get("org.bluez.GattService1")
            and str(interfaces["org.bluez.GattService1"].get("UUID", "")).lower()
            == self.service_uuid
        }
        write_candidate: Optional[Tuple[str, str]] = None
        notify_path = ""
        for path, interfaces in objects.items():
            if not str(path).startswith(self.device_path + "/"):
                continue
            characteristic = interfaces.get(CHARACTERISTIC)
            if not characteristic:
                continue
            if str(characteristic.get("Service", "")) not in service_paths:
                continue
            uuid = str(characteristic.get("UUID", "")).lower()
            if uuid != self.characteristic_uuid:
                continue
            flags = {str(flag) for flag in characteristic.get("Flags", ())}
            if "notify" in flags or "indicate" in flags:
                notify_path = str(path)
            if "write-without-response" in flags:
                write_candidate = (str(path), "command")
            elif "write" in flags and write_candidate is None:
                write_candidate = (str(path), "request")
        if not service_paths:
            raise BluezError("configured JK BLE service UUID is absent")
        if write_candidate is None or not notify_path:
            raise BluezError("configured JK BLE write/notify characteristic is absent")
        return write_candidate[0], notify_path, write_candidate[1]

    def _properties_changed(
        self, interface: str, changed: Dict[str, object], _invalidated, path=None
    ) -> None:
        if (
            str(path) != self._notify_path
            or str(interface) != CHARACTERISTIC
            or "Value" not in changed
        ):
            return
        self._last_notification = time.monotonic()
        self.on_notification(bytes(bytearray(changed["Value"])))

    def _disconnect(self) -> None:
        if self._signal_match is not None:
            try:
                self._signal_match.remove()
            except Exception:
                pass
            self._signal_match = None
        self._notify_path = ""
        try:
            device = dbus.Interface(
                self.bus.get_object(BLUEZ, self.device_path), DEVICE
            )
            device.Disconnect()
        except Exception:
            pass
