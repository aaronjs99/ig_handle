"""Read immutable Linux process identity fields for owned-process handling."""

from pathlib import Path
from typing import Any, Dict


def read_process_stat(pid: int) -> Dict[str, Any]:
    """Read the identity and parentage fields needed for safe signalling."""
    raw = (Path("/proc") / str(int(pid)) / "stat").read_text()
    closing_paren = raw.rfind(")")
    if closing_paren < 0:
        raise RuntimeError("process {} has an invalid stat record".format(pid))
    fields = raw[closing_paren + 2 :].split()
    if len(fields) <= 19:
        raise RuntimeError("process {} has a short stat record".format(pid))
    return {
        "pid": int(pid),
        "state": fields[0],
        "parent_pid": int(fields[1]),
        "process_group_id": int(fields[2]),
        "session_id": int(fields[3]),
        "start_time_ticks": int(fields[19]),
    }
