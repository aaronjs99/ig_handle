#!/usr/bin/env bash
# ig_handle/scripts/run_deltat.sh
set -euo pipefail
PKG_DIR="$(rospack find ig_handle)"
BIN_DIR="$PKG_DIR/scripts/deltat"
BIN="$BIN_DIR/Linux_DeltaT_v1023_x86_64"
INI="$BIN_DIR/Linux_DeltaT.INI"

UDP_DEST_IP="${1:-}"
UDP_PORT="${2:-}"

echo "Starting DeltaT with INI: $INI"
echo "UDP_DEST_IP override:     ${UDP_DEST_IP:-<none>}"
echo "UDP_PORT override:        ${UDP_PORT:-<none>}"

tmp="$(mktemp)"
# Simple sed-based replacement that preserves all other lines
cp "$INI" "$tmp"
if [ -n "$UDP_DEST_IP" ]; then
  # Replace the line after UDPAddress: with the new IP
  sed -i '/^UDPAddress:$/{ n; s/.*/'"$UDP_DEST_IP"'/; }' "$tmp"
fi
if [ -n "$UDP_PORT" ]; then
  # Replace the line after UDPPort: with the new port
  sed -i '/^UDPPort:$/{ n; s/.*/'"$UDP_PORT"'/; }' "$tmp"
fi
mv "$tmp" "$INI"

echo "----- INI just before exec -----"
nl -ba "$INI" | sed -n '1,80p'
echo "--------------------------------"

chmod +x "$BIN"
cd "$BIN_DIR"
exec "$BIN"
