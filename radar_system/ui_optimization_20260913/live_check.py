from pathlib import Path
import urllib.request,json,hashlib
D=Path(__file__).resolve().parent
with urllib.request.urlopen('http://192.168.0.170:8088/',timeout=10) as r:
 assert r.status==200
 assert r.read()==(D/'MODIFIED_FILE.html').read_bytes()
frames=[]
with urllib.request.urlopen('http://192.168.0.170:8088/api/stream',timeout=5) as r:
 for line in r:
  if line.startswith(b'data: '):
   frames.append(json.loads(line[6:]))
   if len(frames)==5:break
assert all(d.get('map_width',0)>0 and len(d.get('ranges',[]))>0 for d in frames)
assert len({(d['robot_x'],d['robot_y']) for d in frames})>1
(D/'live-sample.json').write_text(json.dumps(frames[-1]))
print('http=200 html_match=1 sse_frames=5 pose_updates=1')
