'use strict';
// Run with: node --test tests/test_display_interaction.cjs
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const { test } = require('node:test');
const script = fs.readFileSync(path.join(__dirname, '../radar_system/static/display/display.js'), 'utf8');

function page() {
  let now = 1000;
  const events = new Map(), windowEvents = new Map(), documentEvents = new Map(), elements = new Map();
  function element() {
    return { textContent: '', hidden: false, style: {}, dataset: {}, classList: { remove() {}, add() {} },
      getContext: () => null, setPointerCapture() {}, clientWidth: 1320, clientHeight: 1080,
      addEventListener: (name, fn) => events.set(name, fn) };
  }
  const context = vm.createContext({ performance: { now: () => now },
    document: { addEventListener: (name, fn, options) => documentEvents.set(name, {fn, options}), getElementById: id => { if (!elements.has(id)) elements.set(id, element()); return elements.get(id); } },
    window: { devicePixelRatio: 1, addEventListener: (name, fn) => windowEvents.set(name, fn) },
    requestAnimationFrame() {}, setInterval() {}, EventSource: class {}, console: { error() {}, warn() {} } });
  const api = vm.runInContext(script + '\n({SceneCamera, Renderer, M4, loop, state, onMessage, buildScene})', context);
  const setRenderer = renderer => { context.testRenderer = renderer; vm.runInContext('renderer = testRenderer; renderFailure = null;', context); };
  const dispatch = (name, props = {}) => events.get(name)({ pointerType: 'touch', button: 0, preventDefault() {}, ...props });
  return { ...api, context, events, windowEvents, documentEvents, setRenderer, dispatch, time: t => { now = t; } };
}
function scene() {
  return { body: { front: .67, rear: .08, length: .75, width: .67 }, bounds: {x: 0, y: 0, radius: 3},
    route: [], reference: [], trail: [], displayVehicle: {x: 0, y: 0, yawDeg: 0, preview: true, hidden: false},
    vehicle: null, origin: null, steerDeg: 0, roll: 0 };
}
function camera(p) { const c = new p.SceneCamera(); c.frame(scene(), 1.2, 1/60); return c; }

test('fixed camera follows position and heading at the same distance and pitch', () => {
  const p=page(), c=new p.SceneCamera(), s=scene();
  s.displayVehicle.preview=false; s.vehicle=s.displayVehicle;
  c.frame(s,1.2,1/60);
  const distance=c.distance, pitch=c.pitch;
  s.displayVehicle.x=6; s.displayVehicle.y=3; s.displayVehicle.yawDeg=90;
  for(let i=0;i<180;i++) c.frame(s,1.2,1/60);
  assert.ok(Math.abs(c.x-6)<.001);
  assert.ok(Math.abs(c.y-(3+Math.max(.95,s.body.length*1.25)))<.001);
  assert.ok(Math.abs(c.yaw-90*Math.PI/180)<.001);
  assert.equal(c.distance,distance); assert.equal(c.pitch,pitch);
  assert.equal(c.frame(s,1.2,1/60).moving,false);
  for(const control of ['pan','orbit','scale']) assert.equal(typeof c[control],'undefined');
});

test('camera settles consistently at different refresh rates', () => {
  const p=page(), a=new p.SceneCamera(), b=new p.SceneCamera(), s=scene();
  s.displayVehicle.preview=false; s.vehicle=s.displayVehicle;
  a.frame(s,1.2,1/60); b.frame(s,1.2,1/60);
  s.displayVehicle.x=5;
  for(let i=0;i<30;i++) a.frame(s,1.2,1/30);
  for(let i=0;i<60;i++) b.frame(s,1.2,1/60);
  assert.ok(Math.abs(a.x-b.x)<1e-10);
});

test('no fix shows a stable showcase; a route without a pose is framed as an overview', () => {
  const p=page(), c=new p.SceneCamera(), s=scene();
  const f=c.frame(s,1.2,1/60);
  assert.ok(f.distance>=3.2); assert.equal(f.moving,false);
  s.route=[[0,0],[20,20]]; s.bounds={x:10,y:10,radius:15};
  for(let i=0;i<240;i++) c.frame(s,1.2,1/60);
  assert.ok(Math.abs(c.x-10)<.001 && Math.abs(c.y-10)<.001);
  assert.ok(c.distance>30);
});

test('native pinch, wheel and page zoom shortcuts cannot scale the whole page', () => {
  const p=page();
  for(const type of ['touchstart','touchmove','gesturestart','gesturechange','gestureend','wheel']) {
    const {fn,options}=p.documentEvents.get(type); let prevented=false;
    assert.equal(options.passive,false);
    fn({cancelable:true,preventDefault(){prevented=true;}});
    assert.equal(prevented,true);
  }
  const key=p.documentEvents.get('keydown').fn;
  for(const value of ['+','-','=','0']) {
    let prevented=false; key({ctrlKey:true,key:value,preventDefault(){prevented=true;}});
    assert.equal(prevented,true);
  }
  assert.equal(p.events.has('pointerdown'),false);
  assert.equal(p.events.has('touchstart'),false);
});

test('moving camera draws every animation frame, idle view skips rendering', () => {
  const p=page(), s=scene(); let frames=0;
  const renderer={camera:camera(p),active:s,moving:true,render:()=>{frames++;return true;}};
  p.setRenderer(renderer); p.context.testScene=s;
  vm.runInContext('buildScene=()=>testScene; renderHud=()=>{}; renderPanels=()=>{}; dirty=false;',p.context);
  for(let i=0;i<60;i++) p.loop(1000+i*1000/60);
  assert.equal(frames,60);
  renderer.moving=false; p.loop(2100); assert.equal(frames,60);
});

test('follow-camera movement reuses shadows; moving vehicle and wheels invalidate them', () => {
  const p=page(), s=scene(); let shadows=0, visible=0;
  s.displayVehicle.preview=false; s.vehicle=s.displayVehicle;
  const r=Object.create(p.Renderer.prototype);
  Object.assign(r,{canvas:{clientWidth:1320,clientHeight:1080,width:1320,height:1080},
    gl:new Proxy({},{get:()=>()=>{}}),lit:{u:n=>n},depth:{u:n=>n},shadowFbo:{},
    shadowKey:null,lightVp:p.M4.identity(),active:s,anchor:[0,0],camera:camera(p),lineWidth:.1,
    paint(){},project(){},drawVehicle(_s,shadow){if(shadow) shadows++;else visible++;}});
  r.render(s,1000); assert.equal(shadows,1);
  for(let i=1;i<30;i++) r.render(s,1000+i*1000/60);
  assert.equal(shadows,1); assert.equal(visible,30);
  s.displayVehicle.x=1; r.render(s,1600); assert.equal(shadows,2);
  s.steerDeg=10; r.render(s,1620); assert.equal(shadows,3);
  s.roll=.3; r.render(s,1640); assert.equal(shadows,4);
  r.anchor=[2,0]; r.render(s,1660); assert.equal(shadows,5);
});

function telemetry(fix='FLOAT', x=100, y=200, quality='RTK_FIXED', usable=true, origin=[30,120]) {
  return {release:'test',car:{front_m:.67,rear_m:.08,half_width_m:.335,wheelbase_m:.52},
    tel:{rtk:{stale:false,fix},drive:{speed:0,yaw_rate:0,manual:false},line:{stale:false,origin,phase:'IDLE',
      antenna:{x:x+.3,y:y+.4},pose:{x,y,yaw_deg:25,speed:0,quality,usable,reasons:[]}}}};
}

test('float and single fixes never move the default model or record a trail', () => {
  const p=page(), c=new p.SceneCamera(); let first;
  for(let i=0;i<30;i++) {
    p.time(1000+i*100); p.onMessage(telemetry(i%2?'SINGLE':'FLOAT',i*5,100-i));
    const s=p.buildScene(1000+i*100), f=c.frame(s,1.2,.1);
    assert.equal(s.displayVehicle.preview,true); assert.equal(s.displayVehicle.x,0); assert.equal(s.displayVehicle.y,0);
    assert.equal(s.antennaPoint,null); assert.equal(s.trail.length,0);
    if(!first) first=f; else assert.deepEqual(f.eye,first.eye);
  }
});

test('only a fresh fixed and usable pose establishes location; loss freezes and recovery follows', () => {
  const p=page(), c=new p.SceneCamera();
  p.onMessage(telemetry('FIXED',10,20));
  const fixed=p.buildScene(1000); const frame=c.frame(fixed,1.2,.03);
  assert.equal(fixed.displayVehicle.preview,false); assert.equal(fixed.vehicle.usable,true);
  const trailCount=fixed.trail.length;
  p.time(1100); p.onMessage(telemetry('FLOAT',500,800));
  const held=p.buildScene(1100);
  assert.equal(held.displayVehicle.x,10); assert.equal(held.displayVehicle.y,20);
  assert.equal(held.vehicle.usable,false); assert.equal(held.positionHeld,true); assert.equal(held.trail.length,trailCount);
  assert.deepEqual(c.frame(held,1.2,.5).eye,frame.eye);
  const offline=p.buildScene(5000); assert.equal(offline.positionHeld,true);
  assert.equal(offline.displayVehicle.x,10);
  p.time(5100); p.onMessage(telemetry('FIXED',12,22));
  const resumed=p.buildScene(5100); assert.equal(resumed.displayVehicle.x,12); assert.equal(resumed.positionHeld,false);
  assert.equal(c.frame(resumed,1.2,.03).moving,true);
});

test('stale, invalid and uninitialized heading do not establish a real vehicle pose', () => {
  for(const change of [m=>m.tel.rtk.stale=true,m=>m.tel.line.stale=true,m=>m.tel.line.pose.usable=false,
    m=>m.tel.line.pose.quality='RTK_FLOAT',m=>m.tel.line.pose.reasons=['heading_uninitialized'],m=>m.tel.line.pose.x=NaN]) {
    const p=page(), msg=telemetry('FIXED'); change(msg); p.onMessage(msg);
    assert.equal(p.buildScene(1000).displayVehicle.preview,true);
  }
});

test('a new coordinate origin clears the retained position until another fixed pose arrives', () => {
  const p=page(); p.onMessage(telemetry('FIXED',10,20));
  p.time(1100); p.onMessage(telemetry('FLOAT',500,800,'RTK_FIXED',true,[31,121]));
  const next=p.buildScene(1100); assert.equal(next.displayVehicle.preview,true);
  assert.equal(next.displayVehicle.x,0); assert.equal(next.trail.length,0);
});
