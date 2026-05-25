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

UDP_DEST_IP="${1:-}"
UDP_PORT="${2:-}"
SONAR_PROFILE="${3:-pool}"

case "$SONAR_PROFILE" in
  pool|harbor)
    INI="$BIN_DIR/Linux_DeltaT.$SONAR_PROFILE.INI"
    ;;
  *)
    echo "Unknown DeltaT sonar profile: $SONAR_PROFILE" >&2
    echo "Expected one of: pool, harbor" >&2
    exit 2
    ;;
esac

if [ ! -x "$BIN" ]; then
  echo "DeltaT binary not found or not executable: $BIN" >&2
  exit 1
fi
if [ ! -r "$INI" ]; then
  echo "DeltaT INI not found or not readable: $INI" >&2
  exit 1
fi

mkdir -p "$RUNTIME_DIR"
cp "$INI" "$RUNTIME_INI"
ln -sf "$BIN" "$RUNTIME_BIN"

echo "Starting DeltaT with runtime INI: $RUNTIME_INI"
echo "Sonar profile:            $SONAR_PROFILE"
echo "Profile INI:              $INI"
echo "UDP_DEST_IP override:     ${UDP_DEST_IP:-<none>}"
echo "UDP_PORT override:        ${UDP_PORT:-<none>}"

if [ -n "$UDP_DEST_IP" ]; then
  # Replace the line after UDPAddress: with the new IP
  sed -i '/^UDPAddress:$/{ n; s/.*/'"$UDP_DEST_IP"'/; }' "$RUNTIME_INI"
fi
if [ -n "$UDP_PORT" ]; then
  # Replace the line after UDPPort: with the new port
  sed -i '/^UDPPort:$/{ n; s/.*/'"$UDP_PORT"'/; }' "$RUNTIME_INI"
fi

echo "----- INI just before exec -----"
nl -ba "$RUNTIME_INI" | sed -n '1,80p'
echo "--------------------------------"

cd "$RUNTIME_DIR"
exec "$RUNTIME_BIN"
