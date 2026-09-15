#!/bin/sh
set -eu
P=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
test "$#" -eq 1
python3 - "$P" "$1" <<'PY'
from pathlib import Path
import shutil,sys
p=Path(sys.argv[1]);target=Path(sys.argv[2]).resolve()
assert target.is_file() and target not in [(p/'BASELINE.py').resolve(),(p/'MODIFIED_FILE.py').resolve()]
shutil.copy2(p/'BASELINE.py',target)
assert target.read_bytes()==(p/'BASELINE.py').read_bytes()
print('ROLLBACK_RESTORED_ZERO_ONLY')
PY
