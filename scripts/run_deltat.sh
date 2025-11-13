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
awk -v ip="$UDP_DEST_IP" -v port="$UDP_PORT" '
  BEGIN{inA=0; inP=0}
  # Rebuild UDPAddress block
  /^UDPAddress:[[:space:]]*$/ {
    print "UDPAddress:";
    if (ip!="") print ip; else { inA=1; }  # if no override, fall through to keep old lines
    # If we printed override, skip old lines in the block:
    if (ip!="") { while ( (getline line) > 0 ) {
        if (line ~ /^[[:space:]]*$/) { print ""; break }  # keep the blank separator
        # otherwise swallow old address lines
      }
      next
    }
    next
  }
  # Rebuild UDPPort block
  /^UDPPort:[[:space:]]*$/ {
    print "UDPPort:";
    if (port!="") print port; else { inP=1; }
    if (port!="") { while ( (getline line) > 0 ) {
        if (line ~ /^[[:space:]]*$/) { print ""; break }
      }
      next
    }
    next
  }
  # If no override was provided, and we are inside a block, keep only numeric lines
  inA && /^[0-9.]+$/ { print; next }
  inA && /^[[:space:]]*$/ { print ""; inA=0; next }
  inP && /^[0-9]+$/ { print; next }
  inP && /^[[:space:]]*$/ { print ""; inP=0; next }

  { print }
' "$INI" > "$tmp"
mv "$tmp" "$INI"

echo "----- INI just before exec -----"
nl -ba "$INI" | sed -n '1,80p'
echo "--------------------------------"

chmod +x "$BIN"
cd "$BIN_DIR"
exec "$BIN"
