#!/bin/bash
set -euo pipefail
DIR="$(cd "$(dirname "$0")" && pwd)"
EXPECTED=f2b23be0d9421ecc5b583611c64bd35b90e35b97ffc942ce7a741f38a27954d3
ACTUAL="$(shasum -a 256 "$DIR/BASELINE.html" | awk '{print $1}')"
[ "$ACTUAL" = "$EXPECTED" ] || { echo 'baseline_hash_mismatch'; exit 2; }
if [ "${1:-}" = '--board' ]; then
 ADB=/opt/homebrew/bin/adb
 SERIAL=03801aa8f417ee51
 TARGET=/root/radar_system/templates/index.html
 "$ADB" -s "$SERIAL" push "$DIR/BASELINE.html" "${TARGET}.restore" >/dev/null
 "$ADB" -s "$SERIAL" shell "chmod 644 '${TARGET}.restore' && mv '${TARGET}.restore' '$TARGET'"
 HASH="$("$ADB" -s "$SERIAL" shell "sha256sum '$TARGET'" | awk '{print $1}')"
 [ "$HASH" = "$EXPECTED" ]
 cp -p "$DIR/WORKSPACE_BASELINE.html" "$DIR/../templates/index.html"
 printf 'restored_sha256=%s\n' "$HASH"
else
 TARGET="${1:-$DIR/../templates/index.html}"
 [ "$TARGET" != "$DIR/MODIFIED_FILE.html" ] || { echo 'choose_a_copy_target'; exit 2; }
 cp -p "$DIR/BASELINE.html" "${TARGET}.restore"
 mv "${TARGET}.restore" "$TARGET"
 HASH="$(shasum -a 256 "$TARGET" | awk '{print $1}')"
 [ "$HASH" = "$EXPECTED" ]
 printf 'restored_sha256=%s\n' "$HASH"
fi
