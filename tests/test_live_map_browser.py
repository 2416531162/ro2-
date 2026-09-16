"""Real Chromium UI test with EXPLICIT SYNTHETIC data, no ROS or chassis."""
import base64
import json
import os
import shutil
import sys
import threading
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path

import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'radar_system'))
from live_map_core import encode_grid


def launch_chromium(playwright,**kwargs):
    # 系统里没有 chromium 时用 Playwright 自带的那份(容器/CI 里通常只有它,
    # 而且不在 PATH 上)。两个都没有才跳过。
    executable=shutil.which('chromium') or shutil.which('google-chrome')
    if executable:
        return playwright.chromium.launch(executable_path=executable,**kwargs)
    try:
        return playwright.chromium.launch(**kwargs)
    except Exception as exc:
        pytest.skip('Chromium unavailable: %s'%exc)


def test_map_browser_offline_fixture():
    playwright=pytest.importorskip('playwright.sync_api')
    grid=np.full((180,240),-1,dtype=np.int8)
    grid[15:165,15:225]=0
    grid[15:18,15:225]=100
    grid[162:165,15:225]=100
    grid[15:165,15:18]=100
    grid[15:165,222:225]=100
    grid[15:110,116:119]=100
    grid[138:165,116:119]=100
    grid[65:85,50:80]=100
    grid[65:85,160:190]=100
    png,meta=encode_grid(grid.ravel(),240,180,.05)
    meta.update(origin=[-2.,-2.,0.],revision=1,frame='map',source='TEST_FIXTURE')
    state=dict(map=meta,robot=[2.2,1.4,.5],localized=True,scan_live=True,people=[dict(x=4.,y=2.,locked=True)],
               trajectory=[[-.8,-.4],[0,.1],[.8,.3],[1.6,.7],[2.2,1.4]],scan=[[3.8,1.5],[4.2,2.2]],
               goal=dict(x=2.4,y=1.5,validated=False),plan=[],cloud_revision=1,
               session=dict(mode='mapping',maps=['office_01']),
               error='界面自动化测试：合成数据，不代表实车建图结果')
    # In-memory fixture: no network, no ROS, no relaxed browser security policy.
    with playwright.sync_playwright() as p:
        browser=launch_chromium(p,headless=True,args=['--no-sandbox'])
        page=browser.new_page(viewport=dict(width=1440,height=960),device_scale_factor=1)
        errors=[]
        page.on('pageerror',lambda error:errors.append(str(error)))
        page.evaluate("""data=>{
            window.testData=data; window.testOffline=false;
            window.fetch=async (url,options={})=>{
                if(window.testOffline)return new Response(JSON.stringify({error:'offline'}),{status:503});
                if(options.method==='POST'){
                    const body=JSON.parse(options.body);
                    if(url.endsWith('/save')&&!body.name)return new Response(JSON.stringify({error:'地图名称不能为空'}),{status:400});
                    return new Response(JSON.stringify({ok:true,message:'TEST action accepted'}));
                }
                if(url.endsWith('/cloud'))return new Response(JSON.stringify({revision:1,points:[[3,2,.1],[3,2,.5],[3,2,1.2]]}));
                return new Response(JSON.stringify(window.testData));
            };
        }""",state)
        html=(ROOT/'radar_system/templates/live_map.html').read_text()
        image='data:image/png;base64,'+base64.b64encode(png).decode()+'#v='
        html=html.replace("'/api/live_map/image?v='",json.dumps(image))
        page.set_content(html)
        page.wait_for_function("document.getElementById('connection').textContent==='定位 TF 在线'")
        page.wait_for_function("document.getElementById('empty').textContent===''")
        page.click('#save')
        page.wait_for_function("document.getElementById('message').textContent.includes('地图名称不能为空')")
        page.fill('#name','office_new')
        page.click('#save')
        page.wait_for_function("document.getElementById('message').textContent==='TEST action accepted'")
        page.click('#iso')
        page.click('#top')
        page.click('#zin')
        page.click('#fit')
        assert page.evaluate('document.documentElement.scrollWidth<=innerWidth')
        if os.environ.get('MAP_SCREENSHOT'):
            page.screenshot(path=os.environ['MAP_SCREENSHOT'],full_page=True)
        page.evaluate('window.testOffline=true')
        page.wait_for_function("document.getElementById('connection').textContent.includes('连接中断')")
        assert not errors,errors
        browser.close()
