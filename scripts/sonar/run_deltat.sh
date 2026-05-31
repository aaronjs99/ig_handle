#!/usr/bin/env bash
# ig_handle/scripts/sonar/run_deltat.sh
set -euo pipefail
PKG_DIR="$(rospack find ig_handle)"
BIN_DIR="$PKG_DIR/scripts/sonar/deltat"
BIN="$BIN_DIR/Linux_DeltaT_v1023_x86_64"
RUNTIME_SUBDIR="ig_handle/deltat"
RUNTIME_DIR="${ROS_HOME:-$HOME/.ros}/$RUNTIME_SUBDIR"
RUNTIME_BIN="$RUNTIME_DIR/Linux_DeltaT_v1023_x86_64"
RUNTIME_INI="$RUNTIME_DIR/Linux_DeltaT.INI"

PROFILE_CONFIG="${SONAR_PROFILE_CONFIG:-$PKG_DIR/config/sonar_profiles.yaml}"
SONAR_PROFILE="${SONAR_PROFILE:-}"
SONAR_IP="${SONAR_IP:-}"
UDP_DEST_IP="${UDP_DEST_IP:-}"
UDP_PORT="${UDP_PORT:-}"
SOUND_VELOCITY="${SOUND_VELOCITY:-}"
VERBOSE="${VERBOSE:-0}"

usage() {
  cat >&2 <<EOF
Usage: run_deltat.sh [options]

Options:
  --profile-config PATH      Sonar profile YAML config
  --profile NAME             DeltaT settings profile from the config
  --sonar-ip IP              Override sonar head IP address
  --udp-ip IP                Destination IP for forwarded UDP packets
  --udp-port PORT            Destination UDP port
  --sound-velocity M_PER_S   Override sound velocity written to the runtime INI
  --verbose                  Print generated runtime INI before exec
  -h, --help                 Show this help
EOF
}

require_value() {
  if [ -z "${2-}" ]; then
    echo "$1 requires a value" >&2
    usage
    exit 2
  fi
}

is_true() {
  case "${1:-0}" in
    1|true|TRUE|yes|YES|on|ON) return 0 ;;
    *) return 1 ;;
  esac
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --profile-config)
      require_value "$1" "${2-}"
      PROFILE_CONFIG="$2"
      shift 2
      ;;
    --profile-config=*)
      PROFILE_CONFIG="${1#*=}"
      shift
      ;;
    --profile)
      require_value "$1" "${2-}"
      SONAR_PROFILE="$2"
      shift 2
      ;;
    --profile=*)
      SONAR_PROFILE="${1#*=}"
      shift
      ;;
    --sonar-ip)
      require_value "$1" "${2-}"
      SONAR_IP="$2"
      shift 2
      ;;
    --sonar-ip=*)
      SONAR_IP="${1#*=}"
      shift
      ;;
    --udp-ip|--udp-dest-ip)
      require_value "$1" "${2-}"
      UDP_DEST_IP="$2"
      shift 2
      ;;
    --udp-ip=*|--udp-dest-ip=*)
      UDP_DEST_IP="${1#*=}"
      shift
      ;;
    --udp-port)
      require_value "$1" "${2-}"
      UDP_PORT="$2"
      shift 2
      ;;
    --udp-port=*)
      UDP_PORT="${1#*=}"
      shift
      ;;
    --sound-velocity)
      require_value "$1" "${2-}"
      SOUND_VELOCITY="$2"
      shift 2
      ;;
    --sound-velocity=*)
      SOUND_VELOCITY="${1#*=}"
      shift
      ;;
    --verbose)
      VERBOSE=1
      shift
      ;;
    --verbose=*)
      VERBOSE="${1#*=}"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown DeltaT argument: $1" >&2
      usage
      exit 2
      ;;
  esac
done

if [ ! -x "$BIN" ]; then
  echo "DeltaT binary not found or not executable: $BIN" >&2
  exit 1
fi
if [ ! -r "$PROFILE_CONFIG" ]; then
  echo "Sonar profile config not found or not readable: $PROFILE_CONFIG" >&2
  exit 1
fi

PROFILE_ARGS=(--config "$PROFILE_CONFIG")
if [ -n "$SONAR_PROFILE" ]; then
  PROFILE_ARGS+=(--profile "$SONAR_PROFILE")
fi
if [ -n "$SONAR_IP" ]; then
  PROFILE_ARGS+=(--sonar-ip "$SONAR_IP")
fi
if [ -n "$UDP_DEST_IP" ]; then
  PROFILE_ARGS+=(--udp-ip "$UDP_DEST_IP")
fi
if [ -n "$UDP_PORT" ]; then
  PROFILE_ARGS+=(--udp-port "$UDP_PORT")
fi
if [ -n "$SOUND_VELOCITY" ]; then
  PROFILE_ARGS+=(--sound-velocity "$SOUND_VELOCITY")
fi

PROFILE_DATA="$(python3 "$PKG_DIR/scripts/sonar/profiles.py" "${PROFILE_ARGS[@]}" --format shell)"
eval "$PROFILE_DATA"

mkdir -p "$RUNTIME_DIR"
cat > "$RUNTIME_INI" <<EOF
IPAddress:
$SONAR_IP
Range:
$RANGE_M
Gain:
$GAIN
UDPAddress:
$UDP_DEST_IP
UDPPort:
$UDP_PORT
ExitOnKeyStroke:
0
SoundVelocity:
$SOUND_VELOCITY
EOF
ln -sf "$BIN" "$RUNTIME_BIN"

echo "Starting DeltaT with runtime INI: $RUNTIME_INI"
echo "Sonar profile:            $SONAR_PROFILE"
echo "Profile config:           $PROFILE_CONFIG"
echo "Sonar IP:                 $SONAR_IP"
echo "Range/Gain:               $RANGE_M / $GAIN"
echo "UDP destination:          $UDP_DEST_IP:$UDP_PORT"

if is_true "$VERBOSE"; then
  echo "----- INI just before exec -----"
  nl -ba "$RUNTIME_INI" | sed -n '1,80p'
  echo "--------------------------------"
fi

cd "$RUNTIME_DIR"
exec "$RUNTIME_BIN"
