import sys,pathlib,ast
import numpy as np
source=pathlib.Path(sys.argv[1]).read_text();tree=ast.parse(source);checks=[]
def test(name,fn):
    try:assert fn();checks.append(True)
    except Exception:checks.append(False);print('FAIL '+name)
test('professional_name',lambda:'深度热力图' not in source and '深度距离图 · Z / m' in source)
ns=dict(np=np,DEPTH_VALID_MIN_MM=200,DEPTH_VALID_MAX_MM=5500)
for n in tree.body:
    if isinstance(n,ast.FunctionDef) and n.name=='depth_quality':exec(compile(ast.Module(body=[n],type_ignores=[]),'<quality>','exec'),ns)
a=np.array([0,65535,np.nan,100,6000,200,1000,3000],np.float32)
test('quality_partition',lambda:ns['depth_quality'](a,2000)==dict(valid=.375,missing=.375,outside=.25,clipped=.125))
test('metric_legend',lambda:'DEPTH Z (m)' in source and 'HEAT_DEFAULT_FAR_MM = 2000.0' in source)
test('invalid_legend',lambda:'DARK = INVALID / OUT OF RANGE' in source)
print('checks=4 passed=%d failed=%d'%(sum(checks),4-sum(checks)))
sys.exit(0 if all(checks) else 1)
