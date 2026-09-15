#!/bin/bash
set -euo pipefail
D="$(cd "$(dirname "$0")" && pwd)"
TARGET="${1:-$(dirname "$D")}"
python3 - "$D" "$TARGET" <<'PY'
import hashlib, pathlib, shutil, sys
base, target = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2])
src, dst = base/'BASELINE'/'board_radar_gui.py', target/'board_radar_gui.py'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
assert dst.is_file()
current = sha(dst)
modified = sha(base/'MODIFIED_FILE'/'board_radar_gui.py')
baseline = sha(src)
assert current in (modified, baseline), 'target has unrelated changes'
shutil.copy2(src, dst)
assert sha(dst) == baseline
print('restored_files=1 hashes=baseline')
PY
