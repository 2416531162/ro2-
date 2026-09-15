#!/bin/bash
set -euo pipefail
D="$(cd "$(dirname "$0")" && pwd)"
if [ "${1:-}" = --board ]; then
  adb shell "bash -s" < "$D/rollback_board.sh"
  exit $?
fi
TARGET="${1:-$(dirname "$D")}"
python3 - "$D" "$TARGET" <<'PY'
import hashlib,pathlib,shutil,sys
base,target=pathlib.Path(sys.argv[1]),pathlib.Path(sys.argv[2])
assert target.resolve() not in [(base/'FROZEN_RELEASE').resolve(),(base/'BASELINE').resolve()]
files=['board_radar_gui.py','real_lidar_node.py','n10p_pipeline.py']
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
for name in files:
    assert (target/name).is_file(), 'missing target '+name
    assert sha(target/name)==sha(base/'FROZEN_RELEASE'/name), 'target has unrelated changes: '+name
for name in files[:2]: shutil.copy2(base/'BASELINE'/name,target/name)
(target/'n10p_pipeline.py').unlink()
assert all(sha(target/n)==sha(base/'BASELINE'/n) for n in files[:2])
print('restored_files=2 removed_added_module=1 hashes=baseline')
PY
