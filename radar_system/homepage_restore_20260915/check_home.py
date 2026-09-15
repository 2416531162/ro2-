#!/usr/bin/env python3
import json
import sys
import urllib.request

base = sys.argv[1]
try:
    with urllib.request.urlopen(base+'/', timeout=3) as r:
        status = r.status
        html = r.read()
    with urllib.request.urlopen(base+'/api/stream', timeout=5) as r:
        for _ in range(8):
            line = r.readline()
            if line.startswith(b'data: '):
                data = json.loads(line[6:])
                break
        else:
            raise RuntimeError('no live stream event')
    points = len(data.get('ranges', []))
    assert status == 200 and len(html) > 1000 and points > 0, (status, len(html), points)
    print('PASS homepage: HTTP=200 live_lidar_points='+str(points))
    print(json.dumps({'mapping_status': data.get('mapping_status'), 'hz': data.get('hz'), 'points_count': data.get('points_count')}, ensure_ascii=False))
except Exception as exc:
    print('FAIL homepage: '+str(exc))
    sys.exit(1)
