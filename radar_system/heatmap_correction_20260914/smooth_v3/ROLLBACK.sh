#!/bin/bash
set -euo pipefail
D="$(cd "$(dirname "$0")" && pwd)"
python3 - "$D" "${1:?Specify a local target copy}" <<'PY'
import pathlib,sys,hashlib,shutil
p,t=pathlib.Path(sys.argv[1]),pathlib.Path(sys.argv[2])
sha=lambda x:hashlib.sha256(x.read_bytes()).hexdigest()
assert t.resolve() not in [(p/'MODIFIED_FILE.py').resolve(),(p/'BASELINE.py').resolve()]
assert sha(t)==sha(p/'MODIFIED_FILE.py'),'target has subsequent changes'
shutil.copy2(p/'BASELINE.py',t)
assert sha(t)==sha(p/'BASELINE.py')
print('restored=baseline hash_verified=1')
PY
