from pathlib import Path
import subprocess,shutil,hashlib,json
D=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
modified_hash=sha(D/'MODIFIED_FILE.html')
copy=D/'rollback_test.html';shutil.copy2(D/'MODIFIED_FILE.html',copy)
action=subprocess.run([str(D/'ROLLBACK.sh'),str(copy)],capture_output=True,text=True)
(D/'rollback_action.out').write_text(action.stdout);(D/'rollback_action.exit').write_text(str(action.returncode)+'\n')
assert action.returncode==0,action.stderr
result=subprocess.run(['node',str(D/'test_ui.cjs'),str(copy)],capture_output=True,text=True)
(D/'rollback_behavior.out').write_text(result.stdout);(D/'rollback_behavior.exit').write_text(str(result.returncode)+'\n')
assert sha(copy)==sha(D/'BASELINE.html')
assert result.returncode==int((D/'baseline.exit').read_text())
assert result.stdout==(D/'baseline.out').read_text()
assert sha(D/'MODIFIED_FILE.html')==modified_hash
print('restored=1 baseline_behavior=matched modified_preserved=1')
