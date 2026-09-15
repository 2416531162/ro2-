import contextlib
import importlib.util
import io
import json
from pathlib import Path
import struct
import sys

spec=importlib.util.spec_from_file_location('bench_test',sys.argv[1]);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
from driver_reference import STOP_FRAME,bcc

class Clock:
    def __init__(self):self.t=0.0
    def monotonic(self):return self.t
    def sleep(self,s):self.t+=s

class Serial:
    def __init__(self,clock,case):
        self.clock=clock;self.case=case;self.tx=[];self.is_open=True;self.next_rx=0;self.backlog_cleared=False
    @property
    def in_waiting(self):return 24 if self.clock.t>=self.next_rx and not (self.case=='stale' and self.clock.t>3.2) else 0
    @property
    def out_waiting(self):return 1 if self.case=='backlog' and self.clock.t>3.1 and not self.backlog_cleared else 0
    def reset_output_buffer(self):self.backlog_cleared=True
    def write(self,frame):self.tx.append((self.clock.t,frame));return len(frame)
    def read(self,n):
        if not n:return b''
        self.next_rx=self.clock.t+.05
        cutoff=self.clock.t-(1.2 if self.case=='delayed' else 0)
        prior=[f for t,f in self.tx if t<=cutoff]
        vx=struct.unpack('>h',prior[-1][3:5])[0] if prior else 0
        if self.case=='forward':vx=-vx
        f=bytes([0x7b,0])+struct.pack('>9hH',vx,0,0,0,0,16384,0,0,0,22000)
        return f+bytes([bcc(f),0x7d])
    def close(self):self.is_open=False

results=[]
for case in ['normal','delayed','forward','stale','backlog']:
    clock=Clock();serial=Serial(clock,case);bench=m.Bench(serial,clock)
    with contextlib.redirect_stdout(io.StringIO()):r=bench.run()
    nonzero=[(t,f) for t,f in serial.tx if f!=STOP_FRAME]
    assert all(f[5:9]==bytes(4) and struct.unpack('>h',f[3:5])[0]==-50 for t,f in nonzero)
    if m.PULSE_SPEED==0:assert not nonzero
    elif case in ['normal','delayed']:
        assert r['reverse_feedback_observed'] and r['final_stop_confirmed'] and not r['errors'],(case,r)
        assert int(m.PULSE_SECONDS*45)<=len(nonzero)<=int(m.PULSE_SECONDS*50)+1 and nonzero[-1][0]-nonzero[0][0]<=m.PULSE_SECONDS+.001
    elif case=='forward':assert 'unexpected_forward_feedback' in r['errors'] and len(nonzero)<10,(case,r)
    if case=='stale':assert 'feedback_stale' in r['errors'] and not r['final_stop_confirmed'],r
    if case=='backlog':assert 'serial_output_backlog' in r['errors'] and r['final_stop_confirmed'],r
    if nonzero:
        last=nonzero[-1][0]
        stops=[t for t,f in serial.tx if t>last and f==STOP_FRAME]
        assert len(stops)>=490 and stops[-1]-stops[0]>=9.98
    assert not serial.is_open
    results.append(case)
print('BENCH_TEST PASS cases='+','.join(results)+' speed='+str(m.PULSE_SPEED)+' duration='+str(m.PULSE_SECONDS)+' zero_tail>=10s')
