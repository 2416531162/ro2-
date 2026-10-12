"""Paired fixed/adaptive control, including unannounced plant changes; all samples count."""
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import gzip,hashlib,itertools,json
import numpy as np
import validate_source as v

ROOT=Path(__file__).resolve().parent/'results'/'adaptive'

def transitions():
    rows=[]
    for delay,gains,rough in itertools.product([.23,.47,.63],[(.72,1.28),(1.28,.72),(.9,1.1)],[False,True]):
        rows.append(dict(name='rough_change' if rough else 'response_change',delay=delay,gain=gains[0],gain_after=gains[1],
          change_at_m=35.,length=100.,tau=.17,seed=712 if rough else 317,initial_cm=.8,imu_bias=.4,
          slip=.004,slip_period=17.,noise_amp=.006,noise_period=4.1,
          turn_disturbance=.011 if rough else .001,gyro_vibration=.013 if rough else .001))
    return rows

def run(job):
    suite,index,case=job
    policy=v.production_policy();p=dict(policy,inner=dict(policy['inner'],adaptive_response=False))
    before,b=v.sim.run(v.sim.current,'after',dict(case,**p))
    after,a=v.sim.run(v.sim.current,'after',dict(case,**policy))
    assert all(np.isfinite(trace[key]).all() for trace in (a,b) for key in ('e','cmd','target','response_scale'))
    expected=round(case.get('length',64.)/.01)
    assert len(a['s'])==len(b['s'])==expected
    assert a['s'][0]<.01001 and a['s'][-1]>=case.get('length',64.)-.01001
    row=dict(suite=suite,index=index,case=case,before=before,after=after,
             max_change_mm=(after['max_cm']-before['max_cm'])*10,
             scale_min=min(a['response_scale']),scale_max=max(a['response_scale']),
             all_samples_scored=True)
    if suite=='transition':
        mask=np.asarray(a['s'])>=case['change_at_m']
        row['after_change_rms_mm']=[float(np.sqrt(np.mean(np.asarray(t['e'])[mask]**2))*1000) for t in (b,a)]
    with gzip.open(ROOT/f'{suite}-{index:03d}.json.gz','wt',compresslevel=1) as f:
        json.dump(dict(row=row,before=b,after=a),f,separators=(',',':'))
    return row

def main():
    ROOT.mkdir(parents=True,exist_ok=True)
    groups=[('benchmark',v.suites.benchmark()),('previous_holdout',v.suites.holdout()),
            ('fresh_holdout',v.fresh_cases()),('final_holdout',v.final_holdout()),('transition',transitions())]
    jobs=[(name,i,c) for name,cases in groups for i,c in enumerate(cases)]
    with ProcessPoolExecutor(max_workers=2) as pool:
        rows=[]
        for r in pool.map(run,jobs):
            rows.append(r)
            print(r['suite'],r['index'],round(r['max_change_mm'],4),r['after']['response']['response_accepted_windows'],flush=True)
    prior=[r for r in rows if r['suite']!='transition'];new=[r for r in rows if r['suite']=='transition']
    summary=dict(cases=len(rows),original_cases=len(prior),
        original_passed_before=sum(r['before']['max_cm']<=1.5 for r in prior),
        original_passed_after=sum(r['after']['max_cm']<=1.5 for r in prior),
        original_worst_after_mm=max(r['after']['max_cm']*10 for r in prior),
        original_max_regression_mm=max(r['max_change_mm'] for r in prior),
        learning_cases=sum(r['after']['response']['response_accepted_windows']>0 for r in rows),
        transition_rms_improved=sum(r['after_change_rms_mm'][1]<r['after_change_rms_mm'][0] for r in new),
        transition_worst_regression_mm=max(r['max_change_mm'] for r in new),
        scale_min=min(r['scale_min'] for r in rows),scale_max=max(r['scale_max'] for r in rows))
    hashes={f:hashlib.sha256((v.sim.SOURCE/f).read_bytes()).hexdigest() for f in
            ['radar_system/turn_response.py','radar_system/heading_control.py','radar_system/path_tracker.py',
             'radar_system/line_cascade.py','radar_system/paint_precision.py','tools/grass_precision/simulate.py',
             'tools/grass_precision/validate_source.py','tools/grass_precision/scenarios.py']}
    (ROOT/'validation.json').write_text(json.dumps(dict(summary=summary,source_sha256=hashes,records=rows),indent=2))
    print(json.dumps(summary,indent=2),flush=True)
    assert summary['scale_min']>=.85 and summary['scale_max']<=1.15
    assert summary['original_max_regression_mm']<=.1
    assert summary['original_passed_after']>=summary['original_passed_before']
    assert summary['transition_worst_regression_mm']<=.2
    assert summary['transition_rms_improved']>0

if __name__=='__main__':main()
