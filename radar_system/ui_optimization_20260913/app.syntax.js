
'use strict';
const $ = id => document.getElementById(id);
const canvas = $('mainCanvas'), ctx = canvas.getContext('2d', {alpha:false});
const mapCache = document.createElement('canvas'), mapCtx = mapCache.getContext('2d', {alpha:false});
const number = (value, fallback=0) => typeof value === 'number' && Number.isFinite(value) ? value : fallback;
const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const validPoints = list => Array.isArray(list) ? list.filter(p => Array.isArray(p) && Number.isFinite(p[0]) && Number.isFinite(p[1])) : [];
const format = (v, digits=2) => typeof v === 'number' && Number.isFinite(v) ? v.toFixed(digits) : '--';
let currentMode='3d', state=null, latestState=null, paused=false, lastReceived=0, lastDisplayed=0, streamStatus='waiting', stream=null;
let viewW=1, viewH=1, dpr=1, dirty=true, cacheDirty=true, mapSignature='', frameCount=0, frameAt=0, lastRender=0;
let camera={zoom:1, panX:0, panY:0, yaw:-Math.PI/5, pitch:0.88}, drag=null, toastTimer=0;
const layers={obstacles:true, free:true, path:true, scan:true, beam:false, grid:true};
let wallHeight=.8;

function normalizeState(d) {
 if (!d || typeof d !== 'object' || Array.isArray(d)) throw new Error('Invalid stream payload');
 return {...d, robot_x:number(d.robot_x),robot_y:number(d.robot_y),robot_yaw:number(d.robot_yaw),map_origin_x:number(d.map_origin_x,-4),map_origin_y:number(d.map_origin_y,-3.5),map_res:Math.max(.001,number(d.map_res,.05)),map_width:clamp(Math.floor(number(d.map_width)),0,100000),map_height:clamp(Math.floor(number(d.map_height)),0,100000),map_obstacles:validPoints(d.map_obstacles),map_frees:validPoints(d.map_frees),trajectory:validPoints(d.trajectory).slice(-2000),ranges:Array.isArray(d.ranges)?d.ranges.map(r=>number(r)):[],map_sample_step:clamp(Math.floor(number(d.map_sample_step,2)),1,16)};
}
function mapStats(d) {
 const step=d.map_sample_step, samples=Math.ceil(d.map_width/step)*Math.ceil(d.map_height/step);
 const count=Math.min(samples,d.map_obstacles.length+d.map_frees.length);
 return {area:Math.min(d.map_width*d.map_height,count*step*step)*d.map_res*d.map_res,coverage:samples?count/samples*100:0};
}
function updateBadge(valueId,badgeId,value) {
 const el=$(badgeId); el.classList.remove('safe','warn','danger','unknown');
 if (!Number.isFinite(value) || value<=0 || value>=50){$(valueId).textContent='--';el.classList.add('unknown');return;}
 $(valueId).textContent=value.toFixed(2);el.classList.add(value<.6?'danger':value<1.2?'warn':'safe');
}
function updateUI(d) {
 $('hzBadge').textContent=format(d.hz,1);
 const valid=d.ranges.filter(r=>r>0).length;
 $('pointsVal').textContent=valid.toLocaleString();$('pointsNote').textContent=`本帧 ${d.ranges.length} 个采样 · 有效 ${d.ranges.length?Math.round(valid/d.ranges.length*100):0}%`;
 const stats=mapStats(d);$('mappedAreaVal').textContent=d.map_width?stats.area.toFixed(1):'--';$('progressVal').textContent=d.map_width?`约 ${stats.coverage.toFixed(1)}% 栅格已知 · 抽样估算`:'等待栅格地图';
 $('resolutionVal').textContent=(d.map_res*100).toFixed(1);$('mapSizeVal').textContent=`${d.map_width} × ${d.map_height} 格 · ${(d.map_width*d.map_res).toFixed(1)} × ${(d.map_height*d.map_res).toFixed(1)} m`;
 $('robotXVal').textContent=format(d.robot_x);$('robotYVal').textContent=format(d.robot_y);$('robotYawVal').textContent=format(d.robot_yaw*180/Math.PI,1);
 $('minDistVal').textContent=Number.isFinite(d.min_dist)&&d.min_dist>0&&d.min_dist<50?`${format(d.min_dist)} m`:'-- m';
 for(const dir of ['front','left','right','back'])updateBadge(dir+'Val',dir+'Badge',d[dir+'_dist']);
 $('dangerBanner').hidden=!(Number.isFinite(d.min_dist)&&d.min_dist>0&&d.min_dist<.6);
 $('exportBtn').disabled=false;$('snapshotBtn').disabled=false;
}
function applyState(d) {
 const signature=JSON.stringify([d.map_width,d.map_height,d.map_origin_x,d.map_origin_y,d.map_res,d.map_sample_step,d.map_obstacles,d.map_frees]);
 if(signature!==mapSignature){mapSignature=signature;cacheDirty=true;}
 state=d;lastDisplayed=lastReceived;dirty=true;updateUI(d);
 $('emptyState').hidden=Boolean(d.map_width || d.ranges.some(r=>r>0));
 if(!$('emptyState').hidden){$('emptyTitle').textContent='数据流已连接，等待地图与扫描';$('emptyHint').textContent='尚未收到有效的空间数据';}
}
function receiveFrame(event) {
 try{latestState=normalizeState(JSON.parse(event.data));lastReceived=performance.now();streamStatus='live';if(!paused)applyState(latestState);updateConnection();}
 catch(error){streamStatus='invalid';updateConnection();}
}
function updateConnection() {
 const age=lastReceived?performance.now()-lastReceived:Infinity;
 const live=streamStatus==='live'&&age<3000;
 $('connection').dataset.status=live?'live':streamStatus==='waiting'?'waiting':'offline';
 $('connectionText').textContent=live?'数据流已连接':streamStatus==='invalid'?'数据格式异常':streamStatus==='waiting'?'正在连接':'数据流中断 · 重连中';
 $('liveLabel').textContent=paused?'PAUSED':live?'LIVE':state?'STALE':'WAIT';
 $('liveLabel').style.color=paused||!live?'#f6c879':'#69dec4';
 $('updateVal').textContent=lastDisplayed?paused?'画面已冻结':`${Math.max(0,(performance.now()-lastDisplayed)/1000).toFixed(1)}s 前更新`:'等待数据';
 if(!live&&age>=3000&&streamStatus==='live'){streamStatus='stale';$('connectionText').textContent='数据流超时 · 等待更新';}
 if(!state&&streamStatus!=='waiting'){ $('emptyTitle').textContent='等待数据连接恢复';$('emptyHint').textContent='保留当前视角，连接恢复后自动显示'; }
}
function connectStream(){stream=new EventSource('/api/stream');stream.onmessage=receiveFrame;stream.onerror=()=>{streamStatus='offline';updateConnection();};}
function setMode(mode) {
 if(!['slam','3d','polar'].includes(mode))return;
 currentMode=mode;document.querySelectorAll('.mode-btn').forEach(b=>{const active=b.dataset.mode===mode;b.classList.toggle('active',active);b.setAttribute('aria-pressed',String(active));});
 const titles={slam:['二维占据栅格','正交俯视 · 原始地图坐标与实时扫描','MAP FRAME / TOP VIEW','拖动平移 · 滚轮缩放 · 0 复位'], '3d':['空间立体示意','2D 栅格挤出 · 高度为显示参数，非实测','MAP FRAME / ISOMETRIC','拖动旋转 · Shift 拖动平移 · 滚轮缩放'],polar:['机体坐标点云','前方朝上 · 左侧朝左 · 距离以环线标尺为准','LASER FRAME / POLAR','滚轮缩放量程 · 0 复位 · 方向固定于机体']};
 const t=titles[mode];$('viewModeTitle').textContent=t[0];$('viewDescription').textContent=t[1];$('viewBadge').textContent=t[2];$('interactionHint').textContent=t[3];$('heightControl').hidden=mode!=='3d';
 for(const id of ['layerObstacles','layerFree','layerPath'])$(id).disabled=mode==='polar';
 resetView(false);
}
function notify(message){$('toast').textContent=message;$('toast').hidden=false;clearTimeout(toastTimer);toastTimer=setTimeout(()=>{$('toast').hidden=true;},3200);}
function resetView(message=true){camera={zoom:1,panX:0,panY:0,yaw:-Math.PI/5,pitch:.88};invalidate();if(message)notify('视角已复位，地图数据保持不变');}
function invalidate(){dirty=true;cacheDirty=true;$('zoomVal').textContent=`${Math.round(camera.zoom*100)}%`;}
function zoomBy(factor){camera.zoom=clamp(camera.zoom*factor,.35,5);invalidate();}
function togglePause(){paused=!paused;$('pauseBtn').setAttribute('aria-pressed',String(paused));$('pauseText').textContent=paused?'继续显示':'暂停画面';$('pauseIcon').setAttribute('href',paused?'#i-play':'#i-pause');if(!paused&&latestState)applyState(latestState);updateConnection();}
function resizeCanvas(){const r=canvas.getBoundingClientRect();viewW=Math.max(1,r.width);viewH=Math.max(1,r.height);dpr=Math.min(window.devicePixelRatio||1,2);canvas.width=Math.round(viewW*dpr);canvas.height=Math.round(viewH*dpr);mapCache.width=canvas.width;mapCache.height=canvas.height;ctx.setTransform(dpr,0,0,dpr,0,0);mapCtx.setTransform(dpr,0,0,dpr,0,0);invalidate();}

// World coordinates remain metric; CSS pixels and physical canvas pixels are separated.
function getBounds(){if(!state||!state.map_width||!state.map_height)return{x:-4,y:-3.5,w:8,h:7};return{x:state.map_origin_x,y:state.map_origin_y,w:Math.max(.2,state.map_width*state.map_res),h:Math.max(.2,state.map_height*state.map_res)};}
function projection(){
 const b=getBounds(),cx=b.x+b.w/2,cy=b.y+b.h/2,cos=Math.cos(camera.yaw),sin=Math.sin(camera.yaw),tilt=Math.sin(camera.pitch),up=Math.cos(camera.pitch);
 const raw=(x,y,z=0)=>{x-=cx;y-=cy;return[(x*cos-y*sin),(x*sin+y*cos)*tilt-z*up];};
 const corners=[];for(const x of [b.x,b.x+b.w])for(const y of [b.y,b.y+b.h])for(const z of [0,wallHeight])corners.push(raw(x,y,z));
 const x0=Math.min(...corners.map(p=>p[0])),x1=Math.max(...corners.map(p=>p[0])),y0=Math.min(...corners.map(p=>p[1])),y1=Math.max(...corners.map(p=>p[1]));
 const availableW=Math.max(80,viewW-76),availableH=Math.max(80,viewH-136);
 const scale=(currentMode==='3d'?Math.min(availableW/(x1-x0),availableH/(y1-y0)):Math.min(availableW/b.w,availableH/b.h))*camera.zoom;
 const point=(x,y,z=0)=>{if(currentMode==='3d'){const p=raw(x,y,z);return[viewW/2+camera.panX+(p[0]-(x0+x1)/2)*scale,viewH/2+12+camera.panY+(p[1]-(y0+y1)/2)*scale];}return[viewW/2+camera.panX+(x-cx)*scale,viewH/2+12+camera.panY-(y-cy)*scale];};
 return {b,scale,point,depth:(x,y)=>x*sin+y*cos};
}
function pathLine(c,points,color,width=1){if(points.length<2)return;c.beginPath();c.moveTo(...points[0]);for(let i=1;i<points.length;i++)c.lineTo(...points[i]);c.strokeStyle=color;c.lineWidth=width;c.stroke();}
function polygon(c,points,color){if(points.length<3)return;c.beginPath();c.moveTo(...points[0]);for(let i=1;i<points.length;i++)c.lineTo(...points[i]);c.closePath();c.fillStyle=color;c.fill();}
function drawGround(c,p){
 const b=p.b,P=p.point;
 polygon(c,[P(b.x,b.y),P(b.x+b.w,b.y),P(b.x+b.w,b.y+b.h),P(b.x,b.y+b.h)],'#15222d');
 if(layers.grid){let step=Math.max(1,Math.pow(10,Math.floor(Math.log10(Math.max(b.w,b.h)/20))));for(let x=Math.ceil(b.x/step)*step;x<=b.x+b.w;x+=step)pathLine(c,[P(x,b.y),P(x,b.y+b.h)],'#263642');for(let y=Math.ceil(b.y/step)*step;y<=b.y+b.h;y+=step)pathLine(c,[P(b.x,y),P(b.x+b.w,y)],'#263642');}
 pathLine(c,[P(b.x,b.y),P(b.x+b.w,b.y),P(b.x+b.w,b.y+b.h),P(b.x,b.y+b.h),P(b.x,b.y)],'#41515f');
}
function drawMapCache(p){
 const c=mapCtx,P=p.point;c.setTransform(dpr,0,0,dpr,0,0);c.fillStyle='#111923';c.fillRect(0,0,viewW,viewH);drawGround(c,p);
 if(!state){cacheDirty=false;return;}
 const size=state.map_res*state.map_sample_step,ox=state.map_origin_x,oy=state.map_origin_y;
 const cell=(pt,z=0)=>{const x=ox+pt[0]*state.map_res,y=oy+pt[1]*state.map_res;return [P(x,y,z),P(x+size,y,z),P(x+size,y+size,z),P(x,y+size,z)];};
 if(layers.free){if(currentMode==='slam'){c.fillStyle='#254446';for(const pt of state.map_frees){const a=P(ox+pt[0]*state.map_res,oy+pt[1]*state.map_res+size);c.fillRect(a[0],a[1],size*p.scale+.4,size*p.scale+.4);}}else{for(const pt of state.map_frees)polygon(c,cell(pt),'#254446');}}
 if(layers.obstacles){
  if(currentMode==='slam'){c.fillStyle='#7be1ce';for(const pt of state.map_obstacles){const a=P(ox+pt[0]*state.map_res,oy+pt[1]*state.map_res+size);c.fillRect(a[0],a[1],Math.max(1.5,size*p.scale),Math.max(1.5,size*p.scale));}}
  else{const cells=state.map_obstacles.slice().sort((a,b)=>p.depth(a[0],a[1])-p.depth(b[0],b[1]));for(const pt of cells){const base=cell(pt),top=cell(pt,wallHeight);const edges=[0,1,2,3].sort((a,b)=>((base[a][1]+base[(a+1)%4][1])-(base[b][1]+base[(b+1)%4][1])));for(const i of edges){const j=(i+1)%4;polygon(c,[base[i],base[j],top[j],top[i]],i%2?'#367d76':'#46988b');}polygon(c,top,'#8cddc6');}}
 }
 cacheDirty=false;
}
function scanPoints(){if(!state)return[];const inc=number(state.angle_increment,Math.PI*2/(state.ranges.length||360)),start=number(state.angle_min);return state.ranges.map((r,i)=>{const angle=start+i*inc;return r>0?{r,angle,x:state.robot_x+r*Math.cos(state.robot_yaw+angle),y:state.robot_y+r*Math.sin(state.robot_yaw+angle)}:null;}).filter(Boolean);}
function drawDynamic(p){
 if(!state)return;const P=p.point;
 if(layers.path&&state.trajectory.length>1){const path=state.trajectory.map(pt=>P(pt[0],pt[1],.035));pathLine(ctx,path,'#1b241de6',4);pathLine(ctx,path,'#f6c879',1.65);}
 const robot=P(state.robot_x,state.robot_y,.07),scan=scanPoints();
 if(layers.beam){ctx.beginPath();for(let i=0;i<scan.length;i+=6){const pt=P(scan[i].x,scan[i].y,.045);ctx.moveTo(...robot);ctx.lineTo(...pt);}ctx.strokeStyle='#79e0d326';ctx.lineWidth=.8;ctx.stroke();}
 if(layers.scan){ctx.fillStyle='#ccfff4';ctx.beginPath();for(const hit of scan){const pt=P(hit.x,hit.y,.055);ctx.rect(pt[0]-1,pt[1]-1,2,2);}ctx.fill();}
 const heading=P(state.robot_x+Math.cos(state.robot_yaw)*.45,state.robot_y+Math.sin(state.robot_yaw)*.45,.07),angle=Math.atan2(heading[1]-robot[1],heading[0]-robot[0]);
 ctx.save();ctx.translate(...robot);ctx.rotate(angle);ctx.beginPath();ctx.arc(0,0,11,0,Math.PI*2);ctx.fillStyle='#142f2cee';ctx.fill();ctx.strokeStyle='#83e6ce';ctx.lineWidth=1.5;ctx.stroke();polygon(ctx,[[16,0],[-5,-6],[-2,0],[-5,6]],'#e1fff6');ctx.restore();
}
function drawAxes(p){const P=p.point,origin=P(p.b.x,p.b.y),x=P(p.b.x+.6,p.b.y),y=P(p.b.x,p.b.y+.6),z=P(p.b.x,p.b.y,.6);pathLine(ctx,[origin,x],'#eeac97',1.5);pathLine(ctx,[origin,y],'#8cd8ba',1.5);ctx.font='10px Consolas, monospace';ctx.fillStyle='#eeac97';ctx.fillText('X',x[0]+5,x[1]+3);ctx.fillStyle='#8cd8ba';ctx.fillText('Y',y[0]-9,y[1]+1);if(currentMode==='3d'){pathLine(ctx,[origin,z],'#9ec8f7',1.5);ctx.fillStyle='#9ec8f7';ctx.fillText('Z',z[0]-4,z[1]-6);}}
function drawPolar(){
 ctx.fillStyle='#111923';ctx.fillRect(0,0,viewW,viewH);
 const cx=viewW/2,cy=viewH/2+12,radius=Math.max(35,Math.min(viewW-88,viewH-150)/2),maxRange=8/camera.zoom;
 if(layers.grid){ctx.font='10px Consolas, monospace';for(let i=1;i<=4;i++){const r=radius*i/4;ctx.beginPath();ctx.arc(cx,cy,r,0,Math.PI*2);ctx.strokeStyle=i===4?'#506574':'#30414e';ctx.lineWidth=1;ctx.stroke();ctx.fillStyle='#a2b6c5';ctx.fillText((maxRange*i/4).toFixed(1)+' m',cx+5,cy-r+12);}pathLine(ctx,[[cx-radius,cy],[cx+radius,cy]],'#334653');pathLine(ctx,[[cx,cy-radius],[cx,cy+radius]],'#334653');}
 ctx.font='10px sans-serif';ctx.textAlign='center';ctx.fillStyle='#c2cdd9';ctx.fillText('前 0°',cx,cy-radius-12);ctx.fillText('后 180°',cx,cy+radius+20);ctx.fillText('左',cx-radius-18,cy+3);ctx.fillText('右',cx+radius+18,cy+3);ctx.textAlign='left';
 const hits=scanPoints();
 if(layers.beam){ctx.beginPath();for(let i=0;i<hits.length;i+=6){const hit=hits[i];if(hit.r>maxRange)continue;ctx.moveTo(cx,cy);ctx.lineTo(cx-Math.sin(hit.angle)*hit.r/maxRange*radius,cy-Math.cos(hit.angle)*hit.r/maxRange*radius);}ctx.strokeStyle='#69dec429';ctx.lineWidth=1;ctx.stroke();}
 if(layers.scan)for(const hit of hits){if(hit.r>maxRange)continue;const x=cx-Math.sin(hit.angle)*hit.r/maxRange*radius,y=cy-Math.cos(hit.angle)*hit.r/maxRange*radius;ctx.fillStyle=hit.r<.6?'#ff8c91':hit.r<1.2?'#f6c879':'#a2eee0';ctx.fillRect(x-1.3,y-1.3,2.6,2.6);}
 polygon(ctx,[[cx,cy-10],[cx-5,cy+6],[cx,cy+3],[cx+5,cy+6]],'#d4fff2');
 $('scaleLine').style.width='48px';$('scaleLabel').textContent=`量程 ${maxRange.toFixed(1)} m`;
}
function renderFrame(now){
 requestAnimationFrame(renderFrame);
 if(document.hidden||!dirty||now-lastRender<1000/30)return;lastRender=now;
 ctx.setTransform(dpr,0,0,dpr,0,0);
 if(currentMode==='polar')drawPolar();else{const p=projection();if(cacheDirty)drawMapCache(p);ctx.drawImage(mapCache,0,0,canvas.width,canvas.height,0,0,viewW,viewH);drawDynamic(p);if(layers.grid)drawAxes(p);const a=p.point(p.b.x,p.b.y),b=p.point(p.b.x+1,p.b.y);const unit=Math.hypot(b[0]-a[0],b[1]-a[1]);const meters=unit>100?.5:unit<25?2:1;$('scaleLine').style.width=`${Math.max(4,unit*meters)}px`;$('scaleLabel').textContent=`${meters} m${currentMode==='3d'?' · 地面 X 方向':''}`;}
 dirty=false;frameCount++;
 if(now-frameAt>=1000){$('frameVal').textContent=`${Math.round(frameCount*1000/(now-frameAt))} FPS`;frameCount=0;frameAt=now;}
}
function mapExport(){if(!state)return null;return{format:'rk3588-sparse-map/v1',captured_at:new Date(Date.now()-(performance.now()-lastDisplayed)).toISOString(),source:'/api/stream',display_paused:paused,coordinate_frame:'map',map:{width:state.map_width,height:state.map_height,resolution:state.map_res,origin:[state.map_origin_x,state.map_origin_y],sample_step:state.map_sample_step,obstacles:state.map_obstacles,free:state.map_frees},robot:{x:state.robot_x,y:state.robot_y,yaw:state.robot_yaw},scan:{ranges:state.ranges,angle_min:number(state.angle_min),angle_increment:number(state.angle_increment,Math.PI*2/(state.ranges.length||360))},trajectory:state.trajectory,display:{mode:currentMode,extrusion_height:wallHeight,extrusion_is_measured:false}};}
function downloadBlob(blob,name){const url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download=name;document.body.appendChild(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(url),1000);}
function exportMap(){const data=mapExport();if(!data)return;downloadBlob(new Blob([JSON.stringify(data,null,2)],{type:'application/json'}),`rk3588-map-${Date.now()}.json`);notify('已生成当前显示帧的 JSON 下载');}
function saveSnapshot(){if(!state)return;canvas.toBlob(blob=>{if(blob){downloadBlob(blob,`rk3588-${currentMode}-${Date.now()}.png`);notify('已生成当前视图的 PNG 下载');}else notify('视图导出失败，请重试');},'image/png');}

document.querySelectorAll('[data-mode]').forEach(button=>button.addEventListener('click',()=>setMode(button.dataset.mode)));
$('pauseBtn').addEventListener('click',togglePause);$('exportBtn').addEventListener('click',exportMap);$('snapshotBtn').addEventListener('click',saveSnapshot);
$('zoomInBtn').addEventListener('click',()=>zoomBy(1.2));$('zoomOutBtn').addEventListener('click',()=>zoomBy(1/1.2));$('fitBtn').addEventListener('click',()=>resetView());
$('focusBtn').addEventListener('click',()=>{const focus=$('workspace').classList.toggle('focus');$('focusBtn').setAttribute('aria-pressed',String(focus));resizeCanvas();});
$('fullscreenBtn').addEventListener('click',async()=>{try{if(document.fullscreenElement)await document.exitFullscreen();else await $('viewer').requestFullscreen();}catch(error){notify('全屏请求未完成，可使用专注视图');}});
document.addEventListener('fullscreenchange',resizeCanvas);
for(const [id,key]of Object.entries({layerObstacles:'obstacles',layerFree:'free',layerPath:'path',layerScan:'scan',layerBeam:'beam',layerGrid:'grid'}))$(id).addEventListener('change',e=>{layers[key]=e.target.checked;invalidate();});
$('wallHeight').addEventListener('input',e=>{wallHeight=clamp(Number(e.target.value),.2,2);$('heightVal').textContent=wallHeight.toFixed(1)+' m';invalidate();});
canvas.addEventListener('wheel',e=>{e.preventDefault();zoomBy(Math.exp(-e.deltaY*.001));},{passive:false});
canvas.addEventListener('pointerdown',e=>{if(e.button!==0||currentMode==='polar')return;drag={id:e.pointerId,x:e.clientX,y:e.clientY};canvas.setPointerCapture(e.pointerId);});
canvas.addEventListener('pointermove',e=>{if(!drag||drag.id!==e.pointerId)return;const dx=e.clientX-drag.x,dy=e.clientY-drag.y;if(currentMode==='3d'&&!e.shiftKey){camera.yaw+=dx*.008;camera.pitch=clamp(camera.pitch+dy*.006,.25,1.3);}else{camera.panX+=dx;camera.panY+=dy;}drag.x=e.clientX;drag.y=e.clientY;invalidate();});
function endDrag(e){if(drag&&drag.id===e.pointerId){if(canvas.hasPointerCapture(e.pointerId))canvas.releasePointerCapture(e.pointerId);drag=null;}}
canvas.addEventListener('pointerup',endDrag);canvas.addEventListener('pointercancel',endDrag);canvas.addEventListener('lostpointercapture',()=>{drag=null;});
canvas.addEventListener('keydown',e=>{if(['+','=','-','0','ArrowLeft','ArrowRight','ArrowUp','ArrowDown'].includes(e.key)){e.preventDefault();if(e.key==='+'||e.key==='=')zoomBy(1.2);else if(e.key==='-')zoomBy(1/1.2);else if(e.key==='0')resetView();else if(currentMode!=='polar'){const dx=e.key==='ArrowLeft'?-1:e.key==='ArrowRight'?1:0,dy=e.key==='ArrowUp'?-1:e.key==='ArrowDown'?1:0;if(currentMode==='3d'&&!e.shiftKey){camera.yaw+=dx*.12;camera.pitch=clamp(camera.pitch+dy*.08,.25,1.3);}else{camera.panX+=dx*20;camera.panY+=dy*20;}invalidate();}}});
document.addEventListener('visibilitychange',()=>{if(!document.hidden){dirty=true;frameAt=performance.now();frameCount=0;}});
window.addEventListener('beforeunload',()=>{if(stream)stream.close();});
new ResizeObserver(resizeCanvas).observe($('canvasStage'));
setInterval(()=>{updateConnection();if(paused||performance.now()-lastReceived>3000)$('frameVal').textContent='IDLE';},500);
resizeCanvas();setMode('3d');connectStream();requestAnimationFrame(renderFrame);
