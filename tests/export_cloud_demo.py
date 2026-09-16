"""Export an offline interactive *synthetic* renderer demo; no ROS/network/control."""
import base64
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'radar_system'))
from cloud_scene import ScenePackets
from scene_fixture import room_scene,fixture_state


def export(path):
    packets=ScenePackets();packets.update(room_scene())
    state=fixture_state(packets.meta)
    raw=packets.get(packets.epoch,packets.revision)
    bootstrap='''<script>
    const fixtureState=STATE,fixtureBytes='DATA';
    window.fetch=async(url,options={})=>{
      if(options.method==='POST')return new Response(JSON.stringify({error:'离线合成演示：不启动建图、不保存地图、不控制车辆'}),{status:400});
      if(url.includes('scene.bin'))return new Response(Uint8Array.from(atob(fixtureBytes),c=>c.charCodeAt(0)));
      return new Response(JSON.stringify(fixtureState));
    };
    </script>'''.replace('STATE',json.dumps(state,ensure_ascii=False)).replace('DATA',base64.b64encode(raw).decode())
    html=(ROOT/'radar_system/templates/cloud_map.html').read_text()
    html=html.replace('<link rel="stylesheet" href="/static/cloud.css">','<style>'+(ROOT/'radar_system/static/cloud.css').read_text()+'</style>')
    html=html.replace('<script src="/static/cloud_viewer.js"></script>',bootstrap+'<script>'+(ROOT/'radar_system/static/cloud_viewer.js').read_text()+'</script>')
    html=html.replace('<script src="/static/cloud_app.js"></script>','<script>'+(ROOT/'radar_system/static/cloud_app.js').read_text()+'</script>')
    html=html.replace('<h1>实时三维点云地图</h1>','<h1>三维点云 · 离线交互演示</h1>')
    html=html.replace('<nav class="links"><a href="/map2d">二维导航地图</a><a href="/control">相机 / 遥控 →</a></nav>',
                      '<nav class="links">合成数据演示 · 不连接机器人</nav>')
    Path(path).write_text(html,encoding='utf-8')

if __name__=='__main__':
    export(sys.argv[1] if len(sys.argv)>1 else 'ro2_3d_demo.html')
