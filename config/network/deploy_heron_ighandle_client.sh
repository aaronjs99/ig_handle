#!/usr/bin/env bash
set -Eeuo pipefail

MASTER_URI_DEFAULT="http://192.168.131.10:11311"
ROS_IP_DEFAULT="192.168.131.1"
INTERFACE_DEFAULT="br0"
BACKUP_ROOT="/var/backups/grande-heron-ros"
BASE_LAUNCH="/etc/ros/kinetic/ros.d/base.launch"
ROS_START="/usr/sbin/ros-start"

usage() {
  cat <<'EOF'
Usage:
  sudo deploy_heron_ighandle_client.sh install CLIENT_LAUNCH [MASTER_URI] [ROS_IP] [INTERFACE]
  sudo deploy_heron_ighandle_client.sh rollback [BACKUP_DIRECTORY]

The install operation backs up the current Heron ROS job, installs the
hardware-only client launch, and rewrites the generated robot_upstart launcher
for IGHandle's master. It does not restart ROS. Review the files, then perform a
stationary `systemctl restart ros.service` deliberately.
EOF
}

require_root() {
  if [[ ${EUID} -ne 0 ]]; then
    echo "This operation must run as root on Heron." >&2
    exit 1
  fi
}

copy_if_present() {
  local source=$1 destination=$2
  if [[ -e ${source} ]]; then
    cp -a -- "${source}" "${destination}"
  fi
}

install_client() {
  local client_launch=${1:?client launch is required}
  local master_uri=${2:-${MASTER_URI_DEFAULT}}
  local ros_ip=${3:-${ROS_IP_DEFAULT}}
  local interface=${4:-${INTERFACE_DEFAULT}}

  [[ -f ${client_launch} ]] || {
    echo "Client launch is unavailable: ${client_launch}" >&2
    exit 1
  }
  [[ -f ${BASE_LAUNCH} && -f ${ROS_START} ]] || {
    echo "The existing Heron robot_upstart job is incomplete." >&2
    exit 1
  }
  ip -o -4 addr show dev "${interface}" | grep -Fq " ${ros_ip}/" || {
    echo "${ros_ip} is not assigned to ${interface}." >&2
    exit 1
  }
  [[ ${master_uri} == http://*:* ]] || {
    echo "Invalid ROS master URI: ${master_uri}" >&2
    exit 1
  }

  local stamp backup generated
  stamp=$(date -u +%Y%m%dT%H%M%SZ)
  backup="${BACKUP_ROOT}/${stamp}"
  install -d -m 0750 "${backup}"
  copy_if_present "${BASE_LAUNCH}" "${backup}/base.launch"
  copy_if_present "${ROS_START}" "${backup}/ros-start"
  copy_if_present /etc/ros/setup.bash "${backup}/setup.bash"
  copy_if_present /lib/systemd/system/ros.service "${backup}/ros.service"
  copy_if_present /etc/ros/kinetic/ros.d/.installed_files "${backup}/installed_files"
  ln -sfn "${backup}" "${BACKUP_ROOT}/latest"

  install -m 0644 "${client_launch}" "${BASE_LAUNCH}"
  generated=$(mktemp)
  trap 'rm -f "${generated}"' RETURN
  sed \
    -e "s#^export ROS_HOSTNAME=.*#unset ROS_HOSTNAME\\nexport ROS_IP=${ros_ip}#" \
    -e "s#^export ROS_MASTER_URI=.*#export ROS_MASTER_URI=${master_uri}#" \
    "${backup}/ros-start" >"${generated}"
  grep -Fxq "export ROS_IP=${ros_ip}" "${generated}"
  grep -Fxq "export ROS_MASTER_URI=${master_uri}" "${generated}"
  if grep -q '^export ROS_HOSTNAME=' "${generated}"; then
    echo "Generated launcher still exports ROS_HOSTNAME; refusing install." >&2
    exit 1
  fi
  install -m 0755 "${generated}" "${ROS_START}"
  systemctl daemon-reload

  echo "Prepared Heron client job. Backup: ${backup}"
  echo "No service was restarted. While stationary, review ${BASE_LAUNCH} and ${ROS_START}, then run:"
  echo "  sudo systemctl restart ros.service"
}

rollback_client() {
  local backup=${1:-${BACKUP_ROOT}/latest}
  backup=$(readlink -f -- "${backup}")
  [[ -d ${backup} ]] || {
    echo "Backup directory is unavailable: ${backup}" >&2
    exit 1
  }
  [[ -f ${backup}/base.launch && -f ${backup}/ros-start ]] || {
    echo "Backup does not contain the required Heron job files." >&2
    exit 1
  }
  install -m 0644 "${backup}/base.launch" "${BASE_LAUNCH}"
  install -m 0755 "${backup}/ros-start" "${ROS_START}"
  [[ ! -f ${backup}/setup.bash ]] || install -m 0644 "${backup}/setup.bash" /etc/ros/setup.bash
  [[ ! -f ${backup}/ros.service ]] || install -m 0644 "${backup}/ros.service" /lib/systemd/system/ros.service
  [[ ! -f ${backup}/installed_files ]] || install -m 0644 "${backup}/installed_files" /etc/ros/kinetic/ros.d/.installed_files
  systemctl daemon-reload
  echo "Restored Heron ROS job from ${backup}. No service was restarted."
}

require_root
case ${1:-} in
  install)
    shift
    install_client "$@"
    ;;
  rollback)
    shift
    rollback_client "$@"
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac
