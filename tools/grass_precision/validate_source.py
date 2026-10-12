"""Hard maximum-error check of the actual source; model results are not field certification."""
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import dataclasses,gzip,hashlib,itertools,json,sys
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE/'results'
import scenarios as suites
sim=suites.simulation
from path_tracker import Path,PathConfig,PathTracker,differential_config
from robot_core.kinematics import ChassisGeometry

def fresh_cases():
    # Frozen before validation; never used by the parameter optimizer.
    rows=[]
    for delay,gain,seed,terrain in itertools.product([.30,.50,.70],[.85,1.15],[911,2219],['rough','noise','slip']):
        c=dict(name=terrain,delay=delay,gain=gain,seed=seed,length=64.,
               initial_cm=-1. if seed==911 else 1.,imu_bias=-.6 if seed==911 else .6)
        if terrain=='rough':c.update(slip=.0045,slip_period=15.,turn_disturbance=.011,gyro_vibration=.0135)
        if terrain=='noise':c.update(slip=0.,noise_amp=.0075,noise_period=5.3 if seed==911 else 7.3)
        if terrain=='slip':c.update(slip=.003,slip_period=21.,step_slip=True)
        rows.append(c)
    return rows

def final_holdout():
    # This second set is frozen after fitting expanded failures; do not tune on it.
    rows=[]
    for delay,gain,seed,terrain in itertools.product([.28,.58,.68],[.9,1.1],[331,472],['rough','noise']):
        c=dict(name=terrain,delay=delay,gain=gain,seed=seed,length=64.,
               initial_cm=-1. if seed==331 else 1.,imu_bias=-.45 if seed==331 else .45)
        if terrain=='rough':c.update(slip=.0045,slip_period=16.5,turn_disturbance=.011,gyro_vibration=.0135)
        else:c.update(slip=0.,noise_amp=.008,noise_period=7.7 if seed==331 else 4.1)
        rows.append(c)
    return rows

def production_policy():
    tracker=PathTracker(Path((0.,0.,0.),[(64.,0.)]),differential_config(PathConfig(cruise_mps=.5)),
                        ChassisGeometry(drive='differential',track_m=.675,diff_min_radius_m=.5))
    return dict(inner=dataclasses.asdict(tracker._straight_core.cfg),
                policy=dataclasses.asdict(tracker._straight_guide.cfg),
                feedforward=tracker.cfg.straight_target_feedforward)

def run(item):
    suite,index,case=item
    applied=dict(case,**production_policy())
    metrics,trace=sim.run(sim.current,'after',applied)
    e=np.abs(trace['e']);s=np.asarray(trace['s']);peak=int(e.argmax())
    expected=int(round(case.get('length',64.)/.01))
    complete=bool(len(e)==expected and s[0]<.01001 and s[-1]>=case.get('length',64.)-.01001)
    passed=complete and bool(np.isfinite(e).all()) and float(e.max())<=.015
    row=dict(suite=suite,index=index,case=case,metrics=metrics,samples=len(e),expected_samples=expected,
             all_intended_samples_scored=complete,max_error_m=float(e.max()),max_at_m=float(s[peak]),
             exceeded_samples=int((e>.015).sum()),pass_15mm=passed)
    with gzip.open(ROOT/'source-traces'/f'{suite}-{index:03d}.json.gz','wt',compresslevel=1) as f:
        json.dump(dict(row=row,trace=trace),f,separators=(',',':'))
    return row

def main():
    (ROOT/'source-traces').mkdir(parents=True,exist_ok=True)
    policy=production_policy()
    files=['radar_system/'+p+'.py' for p in ['line_cascade','path_tracker','heading_control','turn_response','paint_precision','paint_quality','paint_start','line_mission','route_operations']]
    hashes={p:hashlib.sha256((sim.SOURCE/p).read_bytes()).hexdigest() for p in files}
    manifest=dict(speed_mps=.5,max_error_m=.015,source_sha256=hashes,production_policy=policy,
                  scope='Controller component closed loop; all intended samples included. Paint pauses are not used to hide errors. Synthetic plant is not field-calibrated.')
    (ROOT/'source-manifest.json').write_text(json.dumps(manifest,indent=2))
    jobs=[(name,i,c) for name,cases in [('benchmark',suites.benchmark()),('previous_holdout',suites.holdout()),('fresh_holdout',fresh_cases()),('final_holdout',final_holdout())]
          for i,c in enumerate(cases)]
    results=[]
    with ProcessPoolExecutor(max_workers=2) as pool:
        for row in pool.map(run,jobs):
            results.append(row)
            print(row['suite'],row['index'],round(row['max_error_m']*100,5),row['pass_15mm'],flush=True)
    summaries={}
    for name in ['benchmark','previous_holdout','fresh_holdout','final_holdout']:
        rows=[r for r in results if r['suite']==name]
        summaries[name]=dict(cases=len(rows),passed=sum(r['pass_15mm'] for r in rows),
                             worst_error_m=max(r['max_error_m'] for r in rows))
    qualified=all(r['pass_15mm'] for r in results)
    report=dict(manifest=manifest,summaries=summaries,all_passed=qualified,field_qualified=False,
                validation_history='Failures in previous_holdout and fresh_holdout were disclosed and later used for fitting. Only final_holdout was not used for fitting.',
                failures=[dict(suite=r['suite'],index=r['index'],max_error_m=r['max_error_m'],max_at_m=r['max_at_m']) for r in results if not r['pass_15mm']],records=results)
    (ROOT/'source-validation.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(summaries,indent=2),flush=True)
    raise SystemExit(0 if qualified else 1)

if __name__=='__main__':main()
