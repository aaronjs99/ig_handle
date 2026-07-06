# Shreyam Sensor Readiness Package

This folder preserves the ROS Noetic sensor readiness package shared by
Shreyam Bhattacharya via email on 2026-07-04.

Source attachment:

- `sensor_reliability_tests.zip`
- SHA-256: `DE95C65026B4A16B3A1F68A45BAE440409E1461FDAA2B42851493C4D427876A7`

The package is kept under `contrib/` so it is available for review and
integration without becoming an active nested catkin package by accident.
Before wiring it into runtime launches, map its sample sensor topics and
network hosts to the canonical IG Handle sensor contract in
`config/sensors/sensor_contract.yaml`.
