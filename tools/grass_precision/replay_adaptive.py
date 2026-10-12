"""Read-only response-observer replay; original physical trajectories are unchanged.

Requires the local 27-job backup index. Matches previously received IMU samples
to the logged heading. Wall/monotonic conversion uses the median trace offset;
this is a data-usability audit, not a counterfactual field-accuracy measurement.
"""
from pathlib import Path
from bisect import bisect_right
import argparse,gzip,json,math,statistics,sys
from mcap_ros2.reader import read_ros2_messages

SOURCE=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(SOURCE/'radar_system'))
from turn_response import TurnResponse

def run(item):
    data=json.load(gzip.open(item['decoded'],'rt'))
    trace=data['/line/trace'];job=Path(item['job'])
    info=json.loads((job/'job.json').read_text())
    cruise=(info.get('job') or {}).get('cruise')
    out=dict(job=job.name,segment=item['segment'],cruise_mps=cruise)
    if cruise is None or cruise>.5+1e-9:
        out['status']='outside_low_speed_scope';return out
    if not trace:out['status']='no_trace';return out
    offset=statistics.median(wall-d['t'] for wall,d in trace)
    samples=[]
    for p in sorted((job/item['segment']).glob('*.mcap')):
        for m in read_ros2_messages(str(p),topics=['/imu/selected/data']):
            x=m.ros_msg;q=x.orientation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            samples.append((m.log_time_ns/1e9,x.header.stamp.sec+x.header.stamp.nanosec/1e9,yaw,x.angular_velocity.z))
    samples.sort();arrivals=[r[0] for r in samples]
    model=TurnResponse();last_section=None;last_t=None
    accepted=rejected=eligible_count=timing_rejected=0;scales=[];reasons={}
    for wall,d in trace:
        now=d['t'];tr=d.get('tracker',{});h=d.get('straight_hold',{});pose=d.get('pose',{})
        section=tr.get('section')
        if last_t is not None and (now-last_t>.5 or section!=last_section):
            accepted+=model.accepted_windows;rejected+=model.rejected_windows;model=TurnResponse()
        dt=.02 if last_t is None else max(0.,min(.1,now-last_t))
        last_t,last_section=now,section
        # Restrict by both receipt and sample timestamps, never read future input.
        i=bisect_right(arrivals,min(wall,now+offset))
        choices=[r for r in samples[max(0,i-12):i] if r[1]-offset<=now]
        matched=None
        if choices:
            theta=h.get('align_theta_deg')
            matched=(min(choices,key=lambda r:abs(math.atan2(math.sin(r[2]-math.radians(theta)),math.cos(r[2]-math.radians(theta)))))
                     if theta is not None else choices[-1])
        good=(d.get('phase')=='RUNNING' and tr.get('gear')=='forward'
              and h.get('weight',0)>=1. and h.get('first_rtk_accepted',False)
              and pose.get('rtk_trusted',False) and .3<=pose.get('speed',0)<=.7
              and not tr.get('integral_frozen',True) and not tr.get('pivoting',False))
        if matched is None:
            timing_rejected+=1;model.observe(now,now,0.,eligible=False)
        else:
            t=matched[1]-offset
            model.observe(now,t,matched[3],eligible=good)
            eligible_count+=bool(good and now-t<=.1)
        model.advance(now,dt)
        model.record_sent(now,d.get('wz',0.))
        scales.append(model.scale)
        reasons[model.reason]=reasons.get(model.reason,0)+1
    out.update(status='replayed',eligible_samples=eligible_count,timing_rejected=timing_rejected,
               accepted_windows=accepted+model.accepted_windows,rejected_windows=rejected+model.rejected_windows,
               scale_min=min(scales,default=1.),scale_max=max(scales,default=1.),reasons=reasons)
    return out

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--index',type=Path,required=True);args=parser.parse_args()
    rows=[]
    for item in json.loads(args.index.read_text()):
        if item['group']!='grass':continue
        row=run(item);rows.append(row);print(row,flush=True)
    out=SOURCE/'tools/grass_precision/results/adaptive';out.mkdir(parents=True,exist_ok=True)
    report=dict(scope=__doc__,jobs=len(set(r['job'] for r in rows)),segments=len(rows),records=rows)
    (out/'recording-replay.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
