"""Run real PathTracker transitions, not only the extracted controller components."""
from pathlib import Path
import itertools,json,sys
HERE=Path(__file__).resolve().parent
ROOT=HERE/'results'
ROOT.mkdir(exist_ok=True)
sys.path.insert(0,str(HERE.parents[1]/'tests'))
from test_grass_closed_loop import run
from route_plan import LINE_RUN_OUT_M

rows=[]
for delay,gain,seed in itertools.product([.25,.44,.65],[.8,1.2],[31,32]):
    metrics,trace=run(True,delay,gain,seed,length=64.,run_out=LINE_RUN_OUT_M)
    complete=metrics['state']=='DONE' and trace and trace[0][0]<.02 and trace[-1][0]>63.98
    rows.append(dict(delay=delay,gain=gain,seed=seed,metrics=metrics,
                     planned_line_m=64.,planned_runout_m=LINE_RUN_OUT_M,
                     scored_from_m=trace[0][0],scored_to_m=trace[-1][0],
                     complete=bool(complete),pass_15mm=bool(complete and metrics['max_cm']<=1.5)))
    print(delay,gain,seed,metrics,flush=True)
(ROOT/'integrated-validation.json').write_text(json.dumps(rows,indent=2))
raise SystemExit(0 if all(r['pass_15mm'] for r in rows) else 1)
