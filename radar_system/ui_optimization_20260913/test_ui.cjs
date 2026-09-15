const fs=require('fs'),vm=require('vm'),assert=require('assert');
const file=process.argv[2],html=fs.readFileSync(file,'utf8'),sample=JSON.parse(fs.readFileSync(__dirname+'/sample.json','utf8'));
const elements=new Map(),streams=[],raf=[],intervals=[],downloads=[],ops=[];let now=1000;
const context2d=new Proxy({}, {get:(t,k)=>t[k]||(t[k]=(...args)=>{ops.push([k,...args]);if(ops.length>50000)ops.shift();}),set:(t,k,v)=>(t[k]=v,true)});
function el(id='',attrs=''){
 const e={id,dataset:{},style:{},hidden:/\bhidden\b/.test(attrs),disabled:/\bdisabled\b/.test(attrs),checked:/\bchecked\b/.test(attrs),value:attrs.match(/value="([^"]*)"/)?.[1],listeners:{},width:900,height:600,textContent:'',innerText:'',className:attrs.match(/class="([^"]*)"/)?.[1]||'',attrs:{},
 getContext:()=>context2d,getBoundingClientRect:()=>({width:800,height:500,left:0,top:0}),setAttribute(k,v){this.attrs[k]=v;},getAttribute(k){return this.attrs[k];},addEventListener(k,fn){this.listeners[k]=fn;},appendChild(){},remove(){},click(){downloads.push(this.download);},toBlob(fn){fn(new Blob(['test'],{type:'image/png'}));},setPointerCapture(){},hasPointerCapture(){return true},releasePointerCapture(){},requestFullscreen:async()=>{}};
 e.classList={add(...c){e.className=[...new Set([...e.className.split(' ').filter(Boolean),...c])].join(' ')},remove(...c){e.className=e.className.split(' ').filter(x=>!c.includes(x)).join(' ')},contains(c){return e.className.split(' ').includes(c)},toggle(c,on){if(on===undefined)on=!this.contains(c);on?this.add(c):this.remove(c);return on}};
 const mode=attrs.match(/data-mode="([^"]*)"/);if(mode)e.dataset.mode=mode[1];
 return e;
}
for(const match of html.matchAll(/<[^>]*\bid="([^"]+)"[^>]*>/g))elements.set(match[1],el(match[1],match[0]));
const document={getElementById:id=>elements.get(id)||null,querySelectorAll:s=>[...elements.values()].filter(e=>s==='[data-mode]'?e.dataset.mode:s==='.mode-btn'?e.classList.contains('mode-btn'):false),createElement:()=>el(),body:el(),addEventListener(){},hidden:false};
const sandbox={document,window:{devicePixelRatio:2,addEventListener(){}},performance:{now:()=>now},EventSource:class{constructor(){streams.push(this)}close(){this.closed=true}},ResizeObserver:class{constructor(fn){this.fn=fn}observe(){}},requestAnimationFrame:fn=>raf.push(fn),setInterval:fn=>intervals.push(fn),setTimeout:()=>1,clearTimeout(){},Blob,URL:{createObjectURL:()=> 'blob:test',revokeObjectURL(){}},alert(){},console,Date};
const c=vm.createContext(sandbox),run=code=>vm.runInContext(code,c),results=[];
function test(name,fn){try{fn();results.push([name,'PASS'])}catch(e){results.push([name,'FAIL',e.message])}}
const script=html.match(/<script>([\s\S]*?)<\/script>/)[1];
test('script_boots',()=>run(script));
function send(d){streams[0].onmessage({data:JSON.stringify(d)});}
test('sample_stream_updates',()=>{send(sample);assert.match((elements.get('hzBadge').textContent||elements.get('hzBadge').innerText),/9.9/)});
test('direction_layout_preserved',()=>{send(sample);for(const dir of ['front','left','right','back'])assert(elements.get(dir+'Badge').classList.contains(dir));});
test('metric_area_derived',()=>{const s=run('mapStats(normalizeState('+JSON.stringify(sample)+'))');assert(s.area>0);assert(s.coverage>0&&s.coverage<=100);const small={...sample,map_frees:[],map_obstacles:[]};send(small);assert.equal(elements.get('mappedAreaVal').textContent,'0.0');send(sample);});
test('zero_origin_preserved',()=>{run('applyState(normalizeState('+JSON.stringify({...sample,map_origin_x:0,map_origin_y:0})+'))');assert.equal(run('projection().b.x'),0);assert.equal(run('projection().b.y'),0);send(sample)});
test('dpr_canvas',()=>{assert.equal(elements.get('mainCanvas').width,1600);assert.equal(elements.get('mainCanvas').height,1000);assert.match(html,/ResizeObserver/)});
test('uniform_2d_scale',()=>{run("setMode('slam')");const p=run('projection()');const o=p.point(0,0),a=p.point(1,0),b=p.point(0,1);assert(Math.abs(Math.hypot(a[0]-o[0],a[1]-o[1])-Math.hypot(b[0]-o[0],b[1]-o[1]))<1e-6);});
test('pause_freezes_then_resumes',()=>{send(sample);run('togglePause()');send({...sample,robot_x:42});assert.notEqual(run('state.robot_x'),42);run('togglePause()');assert.equal(run('state.robot_x'),42);send(sample);});
test('exports_actual_frame',()=>{const data=run('mapExport()');assert.equal(data.map.obstacles.length,sample.map_obstacles.length);assert.equal(data.scan.ranges.length,360);run('exportMap()');assert(downloads.some(x=>x?.endsWith('.json')));});
test('layer_toggle_event',()=>{const e=elements.get('layerObstacles');e.listeners.change({target:{checked:false}});assert.equal(run('layers.obstacles'),false);e.listeners.change({target:{checked:true}});});
test('three_render_modes',()=>{for(const mode of ['slam','3d','polar']){run(`setMode('${mode}');dirty=true;renderFrame(${now+=100})`);assert.equal(run('currentMode'),mode)}assert(ops.some(x=>x[0]==='drawImage'));});
test('cache_reused_for_pose_only',()=>{send(sample);run("setMode('3d');renderFrame(5000)");assert.equal(run('cacheDirty'),false);send({...sample,robot_x:.42});assert.equal(run('cacheDirty'),false);});
test('zoom_clamped_and_reset',()=>{run('zoomBy(100)');assert.equal(run('camera.zoom'),5);run('zoomBy(.0001)');assert.equal(run('camera.zoom'),.35);run('resetView(false)');assert.equal(run('camera.zoom'),1);});
test('stream_error_and_recovery',()=>{streams[0].onerror();assert.equal(elements.get('connection').dataset.status,'offline');send(sample);assert.equal(elements.get('connection').dataset.status,'live');});
test('invalid_payload_keeps_frame',()=>{const before=run('state');streams[0].onmessage({data:'null'});assert.equal(run('state'),before);assert.equal(run('streamStatus'),'invalid');send(sample)});
test('sparse_invalid_ranges',()=>{send({...sample,ranges:[0,null,NaN,2]});assert.equal(run('scanPoints().length'),1);send(sample);});
test('truthful_3d_and_exports',()=>{assert.match(html,/非实测/);assert.match(html,/抽样估算/);assert(!html.includes("alert('已重置"));assert(!html.includes('已保存至 /root/maps'));});
test('responsive_desktop_phone',()=>{assert.match(html,/@media\(max-width:520px\)/);assert.match(html,/@media\(min-width:821px\)/);assert.match(html,/prefers-reduced-motion/);});
test('polar_left_handedness',()=>{send({...sample,robot_x:0,robot_y:0,robot_yaw:0,ranges:[0,2,0,0]});run("setMode('polar')");ops.length=0;run('drawPolar()');const dots=ops.filter(op=>op[0]==='fillRect'&&op[3]===2.6);assert.equal(dots.length,1);assert(dots[0][1]<400);send(sample);});
test('pointer_and_keyboard_controls',()=>{run("setMode('3d')");const c=elements.get('mainCanvas'),before=run('camera.yaw');c.listeners.pointerdown({button:0,pointerId:1,clientX:100,clientY:100});c.listeners.pointermove({pointerId:1,clientX:150,clientY:100,shiftKey:false});assert.notEqual(run('camera.yaw'),before);c.listeners.pointerup({pointerId:1});assert.equal(run('drag'),null);c.listeners.keydown({key:'0',preventDefault(){}});assert.equal(run('camera.zoom'),1);});
test('png_export',()=>{run('saveSnapshot()');assert(downloads.some(x=>x?.endsWith('.png')));});
test('stale_stream_flag',()=>{now+=4000;run('updateConnection()');assert.equal(elements.get('connection').dataset.status,'offline');send(sample);});
test('canvas_intrinsic_size_isolated',()=>{assert.match(html,/#mainCanvas\{position:absolute;inset:0/);});
const fail=results.filter(x=>x[1]==='FAIL').length;
fs.writeFileSync(file+'.test-results.json',JSON.stringify(results,null,2));
console.log(`checks=${results.length} passed=${results.length-fail} failed=${fail}`);
// Full per-check results are retained in the adjacent JSON report.
process.exit(fail?1:0);
