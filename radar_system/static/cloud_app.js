'use strict';
(() => {
  const $=id=>document.getElementById(id);
  let view;
  try{view=new CloudView($('cloud'),$('overlay'));}
  catch(e){$('empty').textContent='三维渲染初始化失败：'+e.message+'\n可切换到二维导航地图。';return;}
  window.cloudView=view; // diagnostic/test interface; read-only scene, no motor control
  let epoch='',loaded='',pending=false,sequence=0,lastUpdate=0,lastState={},timer;
  const controls=[...document.querySelectorAll('.sidebar button')];
  async function get(url,options={},binary=false){
    const c=new AbortController(),timeout=setTimeout(()=>c.abort(),options.method?20000:4000);
    try{const response=await fetch(url,{cache:'no-store',...options,signal:c.signal});
      if(!response.ok){let text;try{text=(await response.json()).error;}catch(_){}throw Error(text||'请求失败 '+response.status);}
      return binary?await response.arrayBuffer():await response.json();
    }finally{clearTimeout(timeout);}
  }
  function status(s){
    const m=s.scene||{};
    $('connection').textContent=s.test_fixture?'离线测试数据':s.localized?'定位 TF 在线':'定位未就绪';$('connection').className='badge'+(s.localized&&!s.test_fixture?' live':'');
    $('count').textContent=(m.count||0).toLocaleString();$('age').textContent=m.age_s==null?'—':`${m.age_s.toFixed(1)} s`;
    $('voxel').textContent=m.voxel_m?`${Math.round(m.voxel_m*100)} cm`:'—';$('area').textContent=s.map?`${s.map.known_area_m2} m²`:'—';
    $('source-label').textContent={depth:'ASTRA · 深度三维观测',pointcloud:'POINTCLOUD2 · 三维点云',octomap:'OCTOMAP · 三维体素'}[m.source]||'等待三维数据源';
    $('view-info').textContent=`${m.live?'实时':'历史 / 等待'} · ${m.count||0} 点 · map / m`;
    $('pose').textContent=s.robot?`车位 ${s.robot[0].toFixed(2)}, ${s.robot[1].toFixed(2)} m · ${(s.robot[2]*180/Math.PI).toFixed(0)}°`:'地图车位：未知';
    $('diagnostic').textContent=[s.error,m.error].filter(Boolean).join('\n');
    $('history-note').textContent=m.kind!=='recent_observations'?'完整 OctoMap 快照的限量显示；二维导航层独立保留。'
      :m.mode==='persistent'
        ?`长期累积 / 最多 ${m.limit} 点，${m.radius_m} m 内入图。走过的区域不会过期；移动的人会留下残影，不作为避障地图。`
        :`最近 ${m.history_s} s 有限窗口 / 最多 ${m.limit} 点。移动物体可能留短时残影，不作为避障地图。`;
    // 相机外参没确认时深度根本不会入图。不明说的话,现场只会看到一张空地图。
    $('calibration').textContent=m.extrinsics_pending
      ?'等待相机外参标定：深度点不会入图。校准后用 SENSOR_TF_CALIBRATED=1 启动。'
      :m.depth_offline&&m.source==='depth'?'深度相机无新数据（>1.5 s）。':'';
    $('calibration').hidden=!$('calibration').textContent;
    $('fixture').textContent=s.test_fixture||'';
    $('empty').textContent=!(m.count>0)?`等待真实三维点云\n${m.error||'需要深度数据、匹配内参和采样时刻 TF。'}\nN10P 单平面扫描不等于三维建模。`:'';
    const intensity=$('color').options[2];intensity.disabled=!m.intensity_available;intensity.textContent=m.intensity_available?'反射强度（真实通道）':'反射强度（无真实通道）';
    if(!m.intensity_available&&view.mode==='intensity'){$('color').value=view.mode='height';}
    $('color').options[1].disabled=!s.localized;
    if(!s.localized&&view.mode==='distance'){$('color').value=view.mode='height';}
    const maps=s.session?.maps||[],sel=$('maps'),old=sel.value;
    if(sel.dataset.list!==JSON.stringify(maps)){sel.replaceChildren(...maps.map(n=>new Option(n,n)));sel.dataset.list=JSON.stringify(maps);if(maps.includes(old))sel.value=old;}
    legend();
  }
  function legend(){
    const mode=view.mode,m=lastState.scene||{};
    $('color-title').textContent={height:'高度着色 · Z / m',distance:'距机器人 · m',intensity:'真实强度 · 原始单位'}[mode];
    const range=mode==='distance'?[0,10]:mode==='intensity'?(m.intensity_range||[0,1]):[view.zLow,view.zHigh];
    $('low-label').textContent=range[0].toFixed(1)+(mode==='intensity'?'':' m');$('high-label').textContent=range[1].toFixed(1)+(mode==='intensity'?'':' m');
  }
  function loadScene(m){
    const key=m.epoch+':'+m.revision;
    if(pending||key===loaded)return;
    pending=true;const current=sequence;
    get(`/api/live_map/scene.bin?epoch=${encodeURIComponent(m.epoch)}&v=${m.revision}`,{},true)
      .then(buffer=>{
        if(current!==sequence||epoch!==m.epoch)return;
        view.setCloud(new Float32Array(buffer),m);loaded=key;updateFollow();
      }).catch(e=>{if(current===sequence)$('diagnostic').textContent='点云传输未完成：'+e.message;})
      .finally(()=>{pending=false;});
  }
  async function refresh(){
    try{const s=await get('/api/live_map');
      if(!s.scene||s.scene.format!=='xyzi-f32le')throw Error('服务未升级到三维点云版本，请重启 run_web.sh');
      lastState=s;lastUpdate=performance.now();
      if(epoch!==s.scene.epoch){epoch=s.scene.epoch;sequence++;loaded='';view.clear();}
      status(s);view.setState(s,true);loadScene(s.scene);
    }catch(e){view.setState(lastState,false);$('connection').textContent='连接中断 · 历史视图';$('connection').className='badge';$('diagnostic').textContent=e.message;}
    finally{timer=setTimeout(refresh,300);}
  }
  async function action(name,body){
    controls.forEach(b=>b.disabled=true);$('message').textContent='正在处理…';
    try{const r=await get('/api/live_map/'+name,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});$('message').textContent=r.message||'已完成';}
    catch(e){$('message').textContent='未完成：'+e.message;}
    finally{controls.forEach(b=>b.disabled=false);}
  }
  function updateFollow(){$('follow').classList.toggle('active',view.follow);$('follow').textContent=view.follow?'跟随车位：开':'跟随车位：关';}
  for(const name of ['orbit','top','front'])$(name).onclick=()=>{view.view(name);for(const n of ['orbit','top','front'])$(n).classList.toggle('active',n===name);};
  $('fit').onclick=()=>{view.fit();updateFollow();};$('follow').onclick=()=>{view.follow=!view.follow;view.setState(lastState,view.online);updateFollow();};
  $('theme').onclick=()=>{view.light=!view.light;document.body.classList.toggle('light',view.light);$('theme').textContent=view.light?'深色模式':'浅色模式';view.draw();};
  $('zin').onclick=()=>{view.distance=Math.max(1,view.distance/1.25);view.draw();};$('zout').onclick=()=>{view.distance=Math.min(150,view.distance*1.25);view.draw();};
  $('color').onchange=()=>{view.mode=$('color').value;legend();view.draw();};
  for(const id of ['zlow','zhigh'])$(id).onchange=()=>{const a=+$('zlow').value,b=+$('zhigh').value;if(Number.isFinite(a)&&Number.isFinite(b)&&a<b&&Math.abs(a)<1000&&Math.abs(b)<1000){view.zLow=a;view.zHigh=b;legend();view.draw();}else{$('zlow').value=view.zLow;$('zhigh').value=view.zHigh;}};
  $('size').oninput=()=>{view.pointSize=+$('size').value;$('size-label').textContent=view.pointSize+' px';view.draw();};
  for(const [id,key] of [['rings','rings'],['grid','grid'],['scan','showScan'],['path','showPath']])$(id).onchange=()=>{view[key]=$(id).checked;view.draw();};
  $('build').onclick=()=>action('start',{mode:'mapping'});$('stop').onclick=()=>action('stop',{});$('save').onclick=()=>action('save',{name:$('name').value});$('load').onclick=()=>action('start',{mode:'localization',name:$('maps').value});$('initial').onclick=()=>action('initial_pose',{x:+$('px').value,y:+$('py').value,yaw:+$('pa').value*Math.PI/180});
  setInterval(()=>{if(performance.now()-lastUpdate>1800&&view.online){view.setState(lastState,false);$('connection').textContent='数据超时 · 历史视图';$('connection').className='badge';}},300);
  document.addEventListener('visibilitychange',()=>{if(!document.hidden)view.draw();});
  updateFollow();refresh();
})();
