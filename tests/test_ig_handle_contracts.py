from __future__ import annotations

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]


def test_ig_handle_repo_keeps_real_platform_launch_and_data_collection_contracts():
    assert (REPO_ROOT / "ig_handle/launch/robots/heron.launch").exists()
    assert (REPO_ROOT / "ig_handle/launch/sensors/start_cam.launch").exists()
    assert (REPO_ROOT / "ig_handle/scripts/pipeline/process_raw_bag.py").exists()
    assert (REPO_ROOT / "ig_handle/scripts/teensy_launcher.py").exists()
