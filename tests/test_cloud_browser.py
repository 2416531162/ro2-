"""Actual Chromium/WebGL with an in-memory synthetic API fixture, never ROS/hardware."""
# Historical optional feature: keep tests, but do not fail collection after removal.
from pathlib import Path as _FeaturePath
import pytest as _feature_pytest
if not (_FeaturePath(__file__).resolve().parents[1] / 'radar_system' / 'cloud_scene.py').exists():
    _feature_pytest.skip('retired feature: cloud_scene.py is not shipped', allow_module_level=True)

import json
import os
import shutil
import sys
import threading
from pathlib import Path
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'radar_system'))
from cloud_scene import ScenePackets
from scene_fixture import room_scene,fixture_state
ROOT=Path(__file__).resolve().parents[1]


@pytest.fixture
def fixture_server():
    packets=ScenePackets();packets.update(room_scene())
    return dict(offline=False,requests=[],state=fixture_state(packets.meta),packets=packets,malformed=False)


def load_fixture(page,control):
    # In-memory transport is deliberate: no external network or browser policy
    # changes. WebGL, events, DOM and application scripts are the real renderer.
    import base64
    def reply(_source,url,options):
        if control['offline']:
            return dict(status=503,body=json.dumps({'error':'test disconnect'}))
        if options.get('method')=='POST':
            body=json.loads(options['body']);control['requests'].append((url,body))
            if url.endswith('/save') and not body.get('name'):
                return dict(status=400,body=json.dumps({'error':'地图名称不能为空'}))
            return dict(status=200,body=json.dumps({'ok':True,'message':'测试请求已接收（无 ROS 动作）'}))
        if 'scene.bin' in url:
            p=control['packets'];data=p.get(p.epoch,p.revision)
            return dict(status=200,binary=True,body=base64.b64encode(b'bad' if control['malformed'] else data).decode())
        return dict(status=200,body=json.dumps(control['state']))
    page.expose_binding('__fixtureReply',reply)
    page.evaluate("""()=>{window.fetch=async(url,options={})=>{
      const r=await window.__fixtureReply(url,options);
      const data=r.binary?Uint8Array.from(atob(r.body),c=>c.charCodeAt(0)):r.body;
      return new Response(data,{status:r.status});
    };}""")
    html=(ROOT/'radar_system/templates/cloud_map.html').read_text()
    html=html.replace('<link rel="stylesheet" href="/static/cloud.css">',
                      '<style>'+(ROOT/'radar_system/static/cloud.css').read_text()+'</style>')
    for name in ['cloud_viewer.js','cloud_app.js']:
        html=html.replace('<script src="/static/'+name+'"></script>',
                          '<script>'+(ROOT/'radar_system/static'/name).read_text()+'</script>')
    page.set_content(html)


def chromium_launch(playwright,**kwargs):
    """启动 Chromium。

    优先用系统里的 chromium/google-chrome;找不到就用 Playwright 自带的那份
    (容器和 CI 里通常只有后者,它不在 PATH 上)。两个都没有才跳过。
    """
    executable=shutil.which('chromium') or shutil.which('google-chrome')
    if executable:
        return playwright.chromium.launch(executable_path=executable,**kwargs)
    try:
        return playwright.chromium.launch(**kwargs)
    except Exception as exc:
        pytest.skip('Chromium unavailable: %s'%exc)


def test_browser_orbit_controls_epoch_disconnect(fixture_server):
    api=pytest.importorskip('playwright.sync_api')
    control=fixture_server
    with api.sync_playwright() as p:
        browser=chromium_launch(p,headless=True,
            args=['--no-sandbox','--enable-unsafe-swiftshader','--use-angle=swiftshader'])
        page=browser.new_page(viewport={'width':1520,'height':1040},device_scale_factor=1)
        errors=[];page.on('pageerror',lambda e:errors.append(str(e)))
        load_fixture(page,control);page.wait_for_function('window.cloudView && cloudView.points.length>10000')
        page.wait_for_function("document.getElementById('connection').textContent==='离线测试数据'")
        if page.evaluate('!!cloudView.gl'):
            assert page.evaluate('cloudView.gl.getError()')==0
        assert page.locator('#color option[value=intensity]').is_disabled()
        start=page.evaluate('cloudView.azimuth')
        box=page.locator('#cloud').bounding_box()
        page.mouse.move(box['x']+300,box['y']+250);page.mouse.down();page.mouse.move(box['x']+370,box['y']+270,steps=8);page.mouse.up()
        assert page.evaluate('cloudView.azimuth')!=start
        d=page.evaluate('cloudView.distance');page.mouse.wheel(0,-200)
        page.wait_for_function(f'cloudView.distance < {d}')
        page.click('#top');assert page.evaluate('cloudView.elevation')==pytest.approx(1.5707963267948966)
        page.click('#orbit');page.click('#fit');page.click('#theme')
        assert page.evaluate('cloudView.light')
        page.click('#theme')
        page.fill('#zlow','0.4');page.locator('#zhigh').focus()
        assert page.evaluate('cloudView.zLow')==pytest.approx(.4)
        page.fill('#zlow','-0.2');page.locator('#zhigh').focus()
        page.select_option('#color','distance');assert page.evaluate('cloudView.mode')=='distance'
        page.select_option('#color','height')
        page.click('#save');page.wait_for_function("document.getElementById('message').textContent.includes('地图名称不能为空')")
        page.fill('#name','office_new');page.click('#save')
        page.wait_for_function("document.getElementById('message').textContent.includes('测试请求已接收')")
        assert control['requests'][-1][1]['name']=='office_new'
        assert page.evaluate('document.documentElement.scrollWidth<=innerWidth')
        page.wait_for_timeout(350)
        # Pixel evidence: colored actual points, not only HTML labels/gray grid.
        count=page.evaluate('''()=>{const gl=cloudView.gl;let p;if(gl){p=new Uint8Array(gl.drawingBufferWidth*gl.drawingBufferHeight*4);gl.readPixels(0,0,gl.drawingBufferWidth,gl.drawingBufferHeight,gl.RGBA,gl.UNSIGNED_BYTE,p);}else{p=cloudView.cpu.getImageData(0,0,cloudView.canvas.width,cloudView.canvas.height).data;}let n=0;for(let i=0;i<p.length;i+=4)if(Math.max(p[i],p[i+1],p[i+2])-Math.min(p[i],p[i+1],p[i+2])>70)n++;return n;}''')
        assert count>10000
        if os.environ.get('CLOUD_SCREENSHOT'):
            page.screenshot(path=os.environ['CLOUD_SCREENSHOT'],full_page=True)
        page.click('#top');page.wait_for_timeout(200)
        if os.environ.get('CLOUD_TOP_SCREENSHOT'):
            page.screenshot(path=os.environ['CLOUD_TOP_SCREENSHOT'],full_page=True)
        # On a map/session change, no old vertices survive even when count becomes zero.
        control['packets'].reset();control['state']=fixture_state(control['packets'].meta)
        control['state']['scene']['live']=False
        page.wait_for_function('cloudView.points.length===0')
        assert page.locator('#empty').inner_text()
        control['offline']=True
        page.wait_for_function("document.getElementById('connection').textContent.includes('连接中断')")
        assert not page.evaluate('cloudView.online')
        page.set_viewport_size({'width':390,'height':844});page.wait_for_timeout(150)
        assert page.evaluate('document.documentElement.scrollWidth<=innerWidth')
        assert not errors,errors
        browser.close()


def test_browser_missing_webgl_falls_back(fixture_server):
    api=pytest.importorskip('playwright.sync_api')
    control=fixture_server
    with api.sync_playwright() as p:
        browser=chromium_launch(p,headless=True,args=['--no-sandbox'])
        page=browser.new_page()
        page.evaluate("()=>{const old=HTMLCanvasElement.prototype.getContext;HTMLCanvasElement.prototype.getContext=function(t,...a){return t==='webgl'?null:old.call(this,t,...a);};}")
        load_fixture(page,control);page.wait_for_function('window.cloudView && cloudView.points.length>10000')
        assert page.evaluate('!!cloudView.cpu')
        assert '软件预览' in page.evaluate('cloudView.warning')
        browser.close()


def test_webgl_driver_when_available(fixture_server):
    api=pytest.importorskip('playwright.sync_api')
    with api.sync_playwright() as p:
        browser=chromium_launch(p,headless=True,args=['--no-sandbox'])
        page=browser.new_page()
        if not page.evaluate("!!document.createElement('canvas').getContext('webgl')"):
            browser.close();pytest.skip('No working WebGL context on this test host; CPU UI test runs separately')
        load_fixture(page,fixture_server)
        page.wait_for_function('window.cloudView && cloudView.points.length>10000')
        # 着色器链接失败不会报错,只会悄悄退到软件渲染。有 WebGL 的机器上
        # 必须真的走 GL 路径,否则等于白装了显卡。
        assert not page.evaluate('!!cloudView.cpu'),page.evaluate('cloudView.warning')
        assert page.evaluate('cloudView.gl.getError()')==0
        browser.close()
