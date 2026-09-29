#!/usr/bin/env bash
set -Eeuo pipefail

MASTER_URI_DEFAULT="http://192.168.131.10:11311"
ROS_IP_DEFAULT="192.168.131.1"
INTERFACE_DEFAULT="br0"
BACKUP_ROOT="/var/backups/grande-heron-ros"
BASE_LAUNCH="/etc/ros/kinetic/ros.d/base.launch"
ROS_START="/usr/sbin/ros-start"
MASTER_MODULE="/etc/ros/kinetic/ros_master.py"
SERVICE_OVERRIDE="/etc/systemd/system/ros.service.d/20-ighandle-client.conf"
STAGING=""
ACTIVE_BACKUP=""
APPLYING=false

usage() {
  cat <<'EOF'
Usage:
  sudo deploy_heron_ighandle_client.sh install CLIENT_LAUNCH MASTER_MODULE [MASTER_URI] [ROS_IP] [INTERFACE]
  sudo deploy_heron_ighandle_client.sh rollback [BACKUP_DIRECTORY]

Installation prepares and validates every replacement before backing up and
changing the Heron ROS job. Its existing systemd service owns master-recovery
restarts. No service is restarted by this helper. Review the installed files,
then deliberately restart ros.service while stationary.
EOF
}

copy_if_present() {
  if [[ -e $1 ]]; then cp -a -- "$1" "$2"; fi
}

restore_optional() {
  if [[ -f $1 ]]; then
    install -m 0644 "$1" "$2" || return
  else
    rm -f -- "$2" || return
  fi
}

restore_files() {
  local backup=$1
  [[ -f ${backup}/base.launch && -f ${backup}/ros-start ]] || return 1
  install -m 0644 "${backup}/base.launch" "${BASE_LAUNCH}" || return
  install -m 0755 "${backup}/ros-start" "${ROS_START}" || return
  restore_optional "${backup}/ros_master.py" "${MASTER_MODULE}" || return
  restore_optional "${backup}/20-ighandle-client.conf" "${SERVICE_OVERRIDE}" || return
}

cleanup() {
  local result=$?
  if [[ ${result} -ne 0 && ${APPLYING} == true ]]; then
    echo "Installation failed; restoring the prior ROS job." >&2
    if restore_files "${ACTIVE_BACKUP}"; then
      systemctl daemon-reload || result=1
    else
      echo "Restoration failed; retained backup: ${ACTIVE_BACKUP}" >&2
      result=1
    fi
  fi
  if [[ -n ${STAGING} ]]; then
    rm -f -- "${STAGING}/base.launch" "${STAGING}/ros-start" \
      "${STAGING}/ros_master.py" "${STAGING}/20-ighandle-client.conf"
    rmdir -- "${STAGING}"
  fi
  return "${result}"
}
trap cleanup EXIT
trap 'exit 129' HUP
trap 'exit 130' INT
trap 'exit 143' TERM

install_client() {
  local client_launch=${1:?client launch is required}
  local master_module=${2:?shared ros_master.py module is required}
  local master_uri=${3:-${MASTER_URI_DEFAULT}}
  local ros_ip=${4:-${ROS_IP_DEFAULT}}
  local interface=${5:-${INTERFACE_DEFAULT}}

  [[ -f ${client_launch} && -f ${master_module} && -f ${BASE_LAUNCH} && -f ${ROS_START} ]] || {
    echo "The client inputs or existing robot_upstart job are incomplete." >&2
    return 1
  }
  ip -o -4 addr show dev "${interface}" | grep -Fq " ${ros_ip}/" || {
    echo "${ros_ip} is not assigned to ${interface}." >&2
    return 1
  }

  STAGING=$(mktemp -d /tmp/grande-heron-deploy.XXXXXXXX)
  python3 - "${client_launch}" "${master_module}" "${ROS_START}" \
    "${STAGING}" "${master_uri}" "${ros_ip}" "${MASTER_MODULE}" "${BASE_LAUNCH}" <<'PY'
import ast
import ipaddress
from pathlib import Path
import re
import shlex
import sys
from urllib.parse import urlsplit
import xml.etree.ElementTree as ET

client, module, launcher, stage, uri, ros_ip, installed_module, installed_launch = sys.argv[1:]
endpoint = urlsplit(uri)
if (endpoint.scheme != "http" or endpoint.username or endpoint.password
        or endpoint.path not in ("", "/") or endpoint.query or endpoint.fragment
        or endpoint.port != 11311):
    raise SystemExit("The physical ROS master must be an HTTP endpoint on port 11311.")
ipaddress.IPv4Address(endpoint.hostname)
ipaddress.IPv4Address(ros_ip)
ET.parse(client)
module_text = Path(module).read_text()
parsed = ast.parse(module_text)
if not any(isinstance(node, ast.FunctionDef) and node.name == "run_client" for node in parsed.body):
    raise SystemExit("The shared master module has no client lifecycle entry point.")
source = Path(launcher).read_text()
if len(re.findall(r"^export ROS_MASTER_URI=.*$", source, re.M)) != 1:
    raise SystemExit("The existing launcher must declare exactly one ROS master.")
source = re.sub(r"^(?:export ROS_HOSTNAME=.*|unset ROS_HOSTNAME)\n?", "", source, flags=re.M)
source = re.sub(r"^export ROS_MASTER_URI=.*$",
                "export ROS_MASTER_URI=" + shlex.quote(uri), source, flags=re.M)
if re.search(r"^export ROS_IP=.*$", source, re.M):
    source = re.sub(r"^export ROS_IP=.*$", "unset ROS_HOSTNAME\nexport ROS_IP=" + ros_ip, source, flags=re.M)
else:
    source = source.replace("export ROS_MASTER_URI=", "unset ROS_HOSTNAME\nexport ROS_IP=" + ros_ip + "\nexport ROS_MASTER_URI=", 1)
lines = source.splitlines()
pattern = re.compile(
    r"^([ \t]*(?:exec[ \t]+)?(?:setuidgid[ \t]+[^ \t]+[ \t]+)?)"
    + r"(?:/usr/bin/python3[ \t]+" + re.escape(installed_module) + r"[ \t]+--[ \t]+)?"
    + r"((?:/[^ \t]+/)?roslaunch)[ \t]+(.*)$"
)
commands = [(i, pattern.match(line)) for i, line in enumerate(lines) if pattern.match(line)]
if len(commands) != 1:
    raise SystemExit("Unsupported robot_upstart launcher: expected one direct roslaunch command.")
index, command = commands[0]
background = command.group(3).rstrip().endswith("&")
if background:
    tail = [line.strip() for line in lines[index + 1:]
            if line.strip() and not line.lstrip().startswith("#")]
    pid_capture = tail and re.fullmatch(r'PID=(?:\$!|"\$!")', tail[0])
    completion = tail and tail[-1] in ('wait "$PID"', 'wait "${PID}"')
    # Kinetic robot_upstart retains its log message and PID file before wait.
    middle = tail[1:-1]
    stock_logging = (len(middle) == 2
        and re.fullmatch(
            r'log info "[A-Za-z0-9_.-]+: Started roslaunch as background process, '
            r'PID \$(?:PID|\{PID\}), ROS_LOG_DIR=\$(?:ROS_LOG_DIR|\{ROS_LOG_DIR\})"',
            middle[0])
        and re.fullmatch(
            r'echo "\$(?:PID|\{PID\})" > '
            r'(?:\$(?:log_path|\{log_path\})/[A-Za-z0-9_.-]+\.pid'
            r'|"\$(?:log_path|\{log_path\})/[A-Za-z0-9_.-]+\.pid")',
            middle[1]))
    if not (pid_capture and completion and (not middle or stock_logging)):
        raise SystemExit("Unsupported launcher PID capture or completion path; no files were installed.")
elif index != len(lines) - 1:
    raise SystemExit("Unsupported launcher completion path; no files were installed.")
lines[index] = (command.group(1) + "/usr/bin/python3 " + shlex.quote(installed_module)
                + " -- " + command.group(2) + " --wait " + shlex.quote(installed_launch)
                + (" &" if background else ""))
staged = Path(stage)
(staged / "base.launch").write_bytes(Path(client).read_bytes())
(staged / "ros_master.py").write_text(module_text)
(staged / "ros-start").write_text("\n".join(lines) + "\n")
PY
  bash -n "${STAGING}/ros-start"
  cat >"${STAGING}/20-ighandle-client.conf" <<'EOF'
[Service]
Restart=on-failure
RestartSec=3
KillMode=control-group
KillSignal=SIGINT
TimeoutStopSec=20
EOF

  install -d -m 0750 "${BACKUP_ROOT}"
  ACTIVE_BACKUP=$(mktemp -d "${BACKUP_ROOT}/$(date -u +%Y%m%dT%H%M%SZ)-XXXXXXXX")
  copy_if_present "${BASE_LAUNCH}" "${ACTIVE_BACKUP}/base.launch"
  copy_if_present "${ROS_START}" "${ACTIVE_BACKUP}/ros-start"
  copy_if_present "${MASTER_MODULE}" "${ACTIVE_BACKUP}/ros_master.py"
  copy_if_present "${SERVICE_OVERRIDE}" "${ACTIVE_BACKUP}/20-ighandle-client.conf"
  copy_if_present /etc/ros/setup.bash "${ACTIVE_BACKUP}/setup.bash"
  copy_if_present /lib/systemd/system/ros.service "${ACTIVE_BACKUP}/ros.service"
  copy_if_present /etc/ros/kinetic/ros.d/.installed_files "${ACTIVE_BACKUP}/installed_files"

  APPLYING=true
  install -d -m 0755 "$(dirname "${SERVICE_OVERRIDE}")"
  install -m 0644 "${STAGING}/base.launch" "${BASE_LAUNCH}"
  install -m 0644 "${STAGING}/ros_master.py" "${MASTER_MODULE}"
  install -m 0755 "${STAGING}/ros-start" "${ROS_START}"
  install -m 0644 "${STAGING}/20-ighandle-client.conf" "${SERVICE_OVERRIDE}"
  systemctl daemon-reload
  ln -sfn "${ACTIVE_BACKUP}" "${BACKUP_ROOT}/latest"
  APPLYING=false
  echo "Prepared Heron hardware client. Backup: ${ACTIVE_BACKUP}"
  echo "No service restarted. Review the installed files while stationary, then run:"
  echo "  sudo systemctl restart ros.service"
}

rollback_client() {
  local backup
  backup=$(readlink -f -- "${1:-${BACKUP_ROOT}/latest}")
  [[ ${backup} == "${BACKUP_ROOT}/"* && -d ${backup} ]] || {
    echo "Select an existing backup inside ${BACKUP_ROOT}." >&2
    return 1
  }
  restore_files "${backup}"
  systemctl daemon-reload
  echo "Restored the ROS job from ${backup}. No service restarted."
}

[[ ${EUID} -eq 0 ]] || { echo "Run this helper as root on Heron." >&2; exit 1; }
case ${1:-} in
  install) shift; install_client "$@" ;;
  rollback) shift; rollback_client "$@" ;;
  *) usage >&2; exit 2 ;;
esac
