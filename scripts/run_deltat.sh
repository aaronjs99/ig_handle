#!/usr/bin/env bash
# ig_handle/scripts/run_deltat.sh
set -euo pipefail

PKG_DIR="$(rospack find ig_handle)"
BIN_DIR="$PKG_DIR/scripts/deltat"
BIN="$BIN_DIR/Linux_DeltaT_v1023_x86_64"
INI="$BIN_DIR/Linux_DeltaT.INI"

# optional overrides
UDP_DEST_IP="${1:-}"
UDP_PORT="${2:-}"

echo "Starting DeltaT with INI: $INI"
echo "UDP_DEST_IP override:     $UDP_DEST_IP"
echo "UDP_PORT override:        $UDP_PORT"

# patch INI if overrides provided
if [[ -n "$UDP_DEST_IP" ]]; then
  sed -i "s/^UDPAddress:.*/UDPAddress:\n$UDP_DEST_IP/" "$INI"
fi

if [[ -n "$UDP_PORT" ]]; then
  sed -i "s/^UDPPort:.*/UDPPort:\n$UDP_PORT/" "$INI"
fi

chmod +x "$BIN"
cd "$BIN_DIR"
exec "$BIN"
