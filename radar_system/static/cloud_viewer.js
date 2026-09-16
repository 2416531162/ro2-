/* Dependency-free WebGL 1 point-cloud renderer. No CDN, Three.js or fake geometry. */
'use strict';
(() => {
  const ramp = [[.64,.18,.85],[.14,.43,1],[.04,.82,.86],[.23,.86,.40],[1,.86,.20],[1,.31,.17]];
  const vertex = `
    attribute vec4 point;
    uniform vec3 eye, rightV, upV, forwardV, robot;
    uniform float aspect, pointSize, zLow, zHigh, intensityLow, intensityHigh;
    uniform int colorMode, primitive;
    uniform vec3 lineColor;
    varying vec3 color;
    varying float hidden;
    vec3 palette(float v) {
      float t=clamp(v,0.,1.)*5.;
      if(t<1.)return mix(vec3(.64,.18,.85),vec3(.14,.43,1.),t);
      if(t<2.)return mix(vec3(.14,.43,1.),vec3(.04,.82,.86),t-1.);
      if(t<3.)return mix(vec3(.04,.82,.86),vec3(.23,.86,.40),t-2.);
      if(t<4.)return mix(vec3(.23,.86,.40),vec3(1.,.86,.20),t-3.);
      return mix(vec3(1.,.86,.20),vec3(1.,.31,.17),t-4.);
    }
    void main(){
      vec3 rel=point.xyz-eye;
      float d=dot(rel,forwardV);
      gl_Position=vec4(dot(rel,rightV)/(.4142135624*aspect),dot(rel,upV)/.4142135624,
                       1.00040008*d-.200040008,d);
      gl_PointSize=pointSize;
      hidden=0.;
      if(primitive==1){color=lineColor;}
      else{
        hidden=(point.z<zLow||point.z>zHigh)?1.:0.;
        float v=(point.z-zLow)/max(zHigh-zLow,.001);
        if(colorMode==1)v=distance(point.xyz,robot)/10.;
        if(colorMode==2)v=(point.w-intensityLow)/max(intensityHigh-intensityLow,.001);
        color=(colorMode==2&&point.w<0.)?vec3(.55,.58,.62):palette(v);
      }
    }`;
  const fragment = `
    precision mediump float;
    varying vec3 color; varying float hidden;
    uniform int primitive; uniform float opacity;
    void main(){
      if(hidden>.5)discard;
      if(primitive==0&&distance(gl_PointCoord,vec2(.5))>.5)discard;
      gl_FragColor=vec4(color,opacity);
    }`;

  class CloudView {
    constructor(canvas,overlay) {
      this.canvas=canvas; this.overlay=overlay; this.hud=overlay.getContext('2d');
      this.target=[0,0,.5]; this.distance=14; this.azimuth=-2.1; this.elevation=.9;
      this.points=new Float32Array(); this.meta={}; this.state={}; this.online=false;
      this.follow=true; this.light=false; this.mode='height'; this.zLow=-.2; this.zHigh=3;
      this.pointSize=2.5; this.rings=true; this.grid=true; this.showScan=true; this.showPath=true;
      this.viewName='orbit'; this.framePending=false; this.lost=false;
      this.gl=canvas.getContext('webgl',{antialias:false,alpha:false,depth:true,preserveDrawingBuffer:true});
      if(this.gl) this.initGL();
      else {this.cpu=canvas.getContext('2d'); this.warning='WebGL 不可用 · 软件预览（最多 10000 点）';}
      this.bind(); this.draw();
    }
    initGL() {
      const gl=this.gl;
      const compile=(type,text)=>{
        const shader=gl.createShader(type); gl.shaderSource(shader,text); gl.compileShader(shader);
        if(!gl.getShaderParameter(shader,gl.COMPILE_STATUS))throw Error(gl.getShaderInfoLog(shader));
        return shader;
      };
      this.program=gl.createProgram();
      const vs=compile(gl.VERTEX_SHADER,vertex),fs=compile(gl.FRAGMENT_SHADER,fragment);
      gl.attachShader(this.program,vs);gl.attachShader(this.program,fs);gl.linkProgram(this.program);
      gl.deleteShader(vs);gl.deleteShader(fs);
      if(!gl.getProgramParameter(this.program,gl.LINK_STATUS))throw Error(gl.getProgramInfoLog(this.program));
      this.loc={};
      for(const name of ['eye','rightV','upV','forwardV','robot','aspect','pointSize','zLow','zHigh','intensityLow','intensityHigh','colorMode','primitive','lineColor','opacity'])this.loc[name]=gl.getUniformLocation(this.program,name);
      this.attribute=gl.getAttribLocation(this.program,'point');
      this.buffer=gl.createBuffer();this.lines=gl.createBuffer();
      this.upload(); gl.enable(gl.DEPTH_TEST); gl.enable(gl.BLEND); gl.blendFunc(gl.SRC_ALPHA,gl.ONE_MINUS_SRC_ALPHA);
      this.warning='';
    }
    upload(){if(!this.gl||this.lost)return;this.gl.bindBuffer(this.gl.ARRAY_BUFFER,this.buffer);this.gl.bufferData(this.gl.ARRAY_BUFFER,this.points,this.gl.DYNAMIC_DRAW);}
    setCloud(points,meta){
      if(!(points instanceof Float32Array)||points.length!==meta.count*4||meta.stride!==16||meta.format!=='xyzi-f32le'||meta.count>120000)throw Error('点云数据格式或长度错误');
      for(let i=0;i<points.length;i++)if(!Number.isFinite(points[i]))throw Error('非有限点云数据');
      const first=!this.points.length;
      this.points=points;this.meta=meta;this.upload();
      if(first&&meta.bounds)this.fit();this.draw();
    }
    clear(){this.points=new Float32Array();this.meta={};this.state={};this.upload();this.draw();}
    setState(state,online=true){
      this.state=state;this.online=online;
      if(this.follow&&online&&state.localized&&state.robot)this.target=[state.robot[0],state.robot[1],.4];
      this.draw();
    }
    fit(){
      if(!this.meta.bounds)return;
      const [a,b]=this.meta.bounds;
      this.target=a.map((v,i)=>(v+b[i])/2);
      this.distance=Math.max(4,Math.hypot(b[0]-a[0],b[1]-a[1],b[2]-a[2])*1.4);
      this.follow=false;this.draw();
    }
    view(name){
      this.viewName=name;
      if(name==='top'){this.elevation=Math.PI/2;this.azimuth=-Math.PI/2;}
      else if(name==='front'){this.elevation=.18;this.azimuth=-Math.PI/2;}
      else{this.elevation=.9;this.azimuth=-2.1;}
      this.draw();
    }
    basis(){
      const e=this.elevation,a=this.azimuth,d=this.distance;
      const dir=[Math.cos(e)*Math.cos(a),Math.cos(e)*Math.sin(a),Math.sin(e)];
      return {eye:this.target.map((v,i)=>v+d*dir[i]),right:[-Math.sin(a),Math.cos(a),0],
              up:[-Math.sin(e)*Math.cos(a),-Math.sin(e)*Math.sin(a),Math.cos(e)],forward:dir.map(v=>-v)};
    }
    project(p){
      const b=this.basis(),rel=p.map((v,i)=>v-b.eye[i]),dot=v=>v.reduce((s,x,i)=>s+x*rel[i],0),depth=dot(b.forward),f=this.canvas.clientHeight/(2*Math.tan(Math.PI/8));
      if(depth<=.1)return null;
      return [this.canvas.clientWidth/2+dot(b.right)*f/depth,this.canvas.clientHeight/2-dot(b.up)*f/depth,depth];
    }
    bind(){
      const c=this.canvas;let drag=null;
      c.addEventListener('contextmenu',e=>e.preventDefault());
      c.addEventListener('pointerdown',e=>{drag={x:e.clientX,y:e.clientY,az:this.azimuth,el:this.elevation,target:[...this.target],pan:e.button!==0||e.shiftKey||this.viewName==='top'};c.setPointerCapture(e.pointerId);});
      c.addEventListener('pointermove',e=>{
        if(!drag)return;const dx=e.clientX-drag.x,dy=e.clientY-drag.y;
        if(drag.pan){const b=this.basis(),s=this.distance*2*Math.tan(Math.PI/8)/c.clientHeight;
          this.target=drag.target.map((v,i)=>v-dx*s*b.right[i]+dy*s*b.up[i]);this.follow=false;
        }else{this.azimuth=drag.az-dx*.006;this.elevation=Math.max(.08,Math.min(Math.PI/2,drag.el+dy*.006));}
        this.draw();
      });
      for(const ev of ['pointerup','pointercancel','lostpointercapture'])c.addEventListener(ev,()=>drag=null);
      c.addEventListener('wheel',e=>{e.preventDefault();this.distance=Math.max(1,Math.min(150,this.distance*Math.exp(e.deltaY*.001)));this.draw();},{passive:false});
      c.addEventListener('webglcontextlost',e=>{e.preventDefault();this.lost=true;this.warning='WebGL 上下文丢失，等待恢复';this.draw();});
      c.addEventListener('webglcontextrestored',()=>{this.lost=false;this.initGL();this.draw();});
      this.resizeObserver=new ResizeObserver(()=>this.draw());this.resizeObserver.observe(c);
    }
    draw(){if(!this.framePending){this.framePending=true;requestAnimationFrame(()=>{this.framePending=false;this.render();});}}
    lineGroups(){
      const groups=[];const add=(color,pts)=>groups.push({color,pts});
      if(this.grid){const pts=[],cx=Math.round(this.target[0]),cy=Math.round(this.target[1]);
        for(let i=-10;i<=10;i++){pts.push([cx+i,cy-10,-.03],[cx+i,cy+10,-.03],[cx-10,cy+i,-.03],[cx+10,cy+i,-.03]);}
        add(this.light?[.77,.82,.86]:[.16,.22,.29],pts);
      }
      const s=this.state,valid=this.online&&s.localized&&s.robot;
      const segments=(a,z=.035)=>{const p=[];for(let i=1;i<a.length;i++)p.push([a[i-1][0],a[i-1][1],z],[a[i][0],a[i][1],z]);return p;};
      if(this.showPath){add([.09,.67,.8],segments(s.trajectory||[]));if(valid)add([.66,.47,.98],segments(s.plan||[],.045));}
      if(!valid)return groups;
      const [x,y,a]=s.robot;
      if(this.rings){const pts=[];for(const r of [1,2,3,5])for(let i=0;i<96;i++)pts.push([x+r*Math.cos(i*Math.PI/48),y+r*Math.sin(i*Math.PI/48),.02],[x+r*Math.cos((i+1)*Math.PI/48),y+r*Math.sin((i+1)*Math.PI/48),.02]);
        add(this.light?[.45,.53,.60]:[.50,.60,.66],pts);}
      const local=(u,v,z)=>[x+u*Math.cos(a)-v*Math.sin(a),y+u*Math.sin(a)+v*Math.cos(a),z];
      const corners=[[-.18,-.335],[.67,-.335],[.67,.335],[-.18,.335]],body=[];
      for(let i=0;i<4;i++)body.push(local(...corners[i],.05),local(...corners[(i+1)%4],.05));
      body.push(local(0,0,.06),local(.9,0,.06),local(.9,0,.06),local(.7,.13,.06),local(.9,0,.06),local(.7,-.13,.06));
      add([.96,.83,.21],body);
      for(const p of s.people||[]){const pts=[],r=p.locked?.23:.16;for(let i=0;i<32;i++)pts.push([p.x+r*Math.cos(i*Math.PI/16),p.y+r*Math.sin(i*Math.PI/16),.05],[p.x+r*Math.cos((i+1)*Math.PI/16),p.y+r*Math.sin((i+1)*Math.PI/16),.05]);add([1,.49,.19],pts);}
      if(s.goal){const g=s.goal;add([1,.55,.2],[[g.x-.15,g.y-.15,.06],[g.x+.15,g.y+.15,.06],[g.x-.15,g.y+.15,.06],[g.x+.15,g.y-.15,.06]]);}
      if(this.showScan){const pts=[];for(const p of s.scan||[])pts.push([p[0]-.015,p[1],.025],[p[0]+.015,p[1],.025]);add([.1,.9,.9],pts);}
      return groups;
    }
    render(){
      const c=this.canvas,w=c.clientWidth,h=c.clientHeight;if(!w||!h)return;
      const dpr=Math.min(window.devicePixelRatio||1,2);
      for(const node of [c,this.overlay])if(node.width!==Math.round(w*dpr)||node.height!==Math.round(h*dpr)){node.width=Math.round(w*dpr);node.height=Math.round(h*dpr);}
      const groups=this.lineGroups();
      if(this.gl&&!this.lost){const gl=this.gl,b=this.basis(),s=this.state;
        gl.viewport(0,0,c.width,c.height);gl.clearColor(...(this.light?[.92,.945,.965,1]:[.059,.086,.125,1]));gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);gl.useProgram(this.program);
        for(const [name,val] of Object.entries({eye:b.eye,rightV:b.right,upV:b.up,forwardV:b.forward,robot:s.robot?[s.robot[0],s.robot[1],0]:[0,0,0]}))gl.uniform3fv(this.loc[name],val);
        for(const [name,val] of Object.entries({aspect:w/h,pointSize:this.pointSize*dpr,zLow:this.zLow,zHigh:this.zHigh,intensityLow:this.meta.intensity_range?.[0]||0,intensityHigh:this.meta.intensity_range?.[1]||1,opacity:(s.scene?.live&&this.online)?1:.60}))gl.uniform1f(this.loc[name],val);
        gl.uniform1i(this.loc.colorMode,['height','distance','intensity'].indexOf(this.mode));
        gl.uniform1i(this.loc.primitive,0);gl.bindBuffer(gl.ARRAY_BUFFER,this.buffer);gl.vertexAttribPointer(this.attribute,4,gl.FLOAT,false,16,0);gl.enableVertexAttribArray(this.attribute);gl.drawArrays(gl.POINTS,0,this.points.length/4);
        gl.uniform1i(this.loc.primitive,1);gl.uniform1f(this.loc.opacity,1);
        gl.bindBuffer(gl.ARRAY_BUFFER,this.lines);gl.vertexAttribPointer(this.attribute,4,gl.FLOAT,false,16,0);
        for(const group of groups){if(!group.pts.length)continue;const a=new Float32Array(group.pts.length*4);group.pts.forEach((p,i)=>a.set([...p,0],i*4));gl.bufferData(gl.ARRAY_BUFFER,a,gl.DYNAMIC_DRAW);gl.uniform3fv(this.loc.lineColor,group.color);gl.drawArrays(gl.LINES,0,group.pts.length);}
      }else if(this.cpu){this.renderCPU(w,h,dpr,groups);}
      this.drawHud(w,h,dpr);
    }
    renderCPU(w,h,dpr,groups){
      const ctx=this.cpu;ctx.setTransform(dpr,0,0,dpr,0,0);ctx.fillStyle=this.light?'#ebf1f6':'#0f1620';ctx.fillRect(0,0,w,h);
      const pts=[],step=Math.max(1,Math.ceil(this.points.length/4/10000));
      for(let i=0;i<this.points.length;i+=4*step){const p=Array.from(this.points.subarray(i,i+4));if(p[2]<this.zLow||p[2]>this.zHigh)continue;const q=this.project(p.slice(0,3));if(q)pts.push({p,q});}
      pts.sort((a,b)=>b.q[2]-a.q[2]);
      for(const {p,q} of pts){let v=(p[2]-this.zLow)/(this.zHigh-this.zLow);if(this.mode==='distance'){const r=this.state.robot||[0,0];v=Math.hypot(p[0]-r[0],p[1]-r[1],p[2])/10;}if(this.mode==='intensity')v=(p[3]-(this.meta.intensity_range?.[0]||0))/Math.max(.001,(this.meta.intensity_range?.[1]||1)-(this.meta.intensity_range?.[0]||0));let t=Math.max(0,Math.min(1,v))*5,j=Math.min(4,Math.floor(t)),rgb=ramp[j].map((v,k)=>Math.round((v*(1-t+j)+ramp[j+1][k]*(t-j))*255));if(this.mode==='intensity'&&p[3]<0)rgb=[140,148,158];ctx.fillStyle=`rgb(${rgb})`;ctx.fillRect(q[0],q[1],this.pointSize,this.pointSize);}
      for(const {color,pts} of groups){ctx.strokeStyle=`rgb(${color.map(v=>Math.round(v*255))})`;ctx.lineWidth=1;ctx.beginPath();for(let i=0;i<pts.length;i+=2){const a=this.project(pts[i]),b=this.project(pts[i+1]);if(a&&b){ctx.moveTo(...a.slice(0,2));ctx.lineTo(...b.slice(0,2));}}ctx.stroke();}
    }
    drawHud(w,h,dpr){
      const ctx=this.hud;ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,w,h);ctx.font='12px system-ui';ctx.fillStyle=this.light?'#41596d':'#a9bdcd';
      const s=this.state,valid=this.online&&s.localized&&s.robot;
      if(this.rings&&valid){for(const r of [1,2,3,5]){const p=this.project([s.robot[0]+r,s.robot[1],.03]);if(p)ctx.fillText(`${r} m`,p[0]+4,p[1]-4);}}
      if(valid){const p=this.project([s.robot[0],s.robot[1],.05]);if(p){ctx.fillStyle='#f4c936';ctx.fillText('ROBOT',p[0]+12,p[1]-10);}for(const person of s.people||[]){const p=this.project([person.x,person.y,.05]);if(p){ctx.fillStyle='#ff9338';ctx.fillText(person.locked?'跟随目标':'人体观测',p[0]+10,p[1]-12);}}}
      ctx.fillStyle=this.light?'#41596d':'#a9bdcd';ctx.fillText('map / m  ·  网格 1 m  ·  '+(this.gl?'WebGL':'CPU'),16,h-18);
      ctx.fillText('左键旋转 · 右键/Shift 平移 · 滚轮缩放',16,h-38);
      if(this.warning){ctx.fillStyle='#eb8f38';ctx.fillText(this.warning,16,h-60);}
    }
  }
  window.CloudView=CloudView;
})();
