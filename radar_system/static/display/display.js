// 车载屏幕只读显示。地图是 app（ui/scene/GridSceneRenderer.kt 等）的 WebGL 移植：同样的网格地面、路线色带、
// 车辆模型、阴影和跟车镜头；右侧是卫星和连接状态，文字与 app 一致。只收 /display/stream，不向车发任何指令。
// 跟着平板走：车上的路线（平板改完自动存到车上）、平板正在预览的图纸（导入还没应用）、平板点开始后的作业。
'use strict';

// ======================= 小工具 =======================
const $ = (id) => document.getElementById(id);
const fin = (v) => typeof v === 'number' && Number.isFinite(v);
const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
const fmt = (v, d = 1) => (fin(v) ? v.toFixed(d) : '—');
function setText(el, text, cls) {
  if (el.textContent !== text) el.textContent = text;
  if (cls !== undefined) tone(el, cls);
}

// ======================= 矩阵（列主序，与 android.opengl.Matrix 相同） =======================
const M4 = {
  identity() { const m = new Float32Array(16); m[0] = m[5] = m[10] = m[15] = 1; return m; },
  mul(a, b) {
    const o = new Float32Array(16);
    for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) {
      let s = 0; for (let k = 0; k < 4; k++) s += a[k * 4 + r] * b[c * 4 + k]; o[c * 4 + r] = s;
    }
    return o;
  },
  perspective(fovyDeg, aspect, near, far) {
    const f = 1 / Math.tan(fovyDeg * Math.PI / 360), m = new Float32Array(16);
    m[0] = f / aspect; m[5] = f; m[10] = (far + near) / (near - far); m[11] = -1; m[14] = 2 * far * near / (near - far);
    return m;
  },
  ortho(l, r, b, t, n, f) {
    const m = new Float32Array(16);
    m[0] = 2 / (r - l); m[5] = 2 / (t - b); m[10] = -2 / (f - n);
    m[12] = -(r + l) / (r - l); m[13] = -(t + b) / (t - b); m[14] = -(f + n) / (f - n); m[15] = 1;
    return m;
  },
  lookAt(ex, ey, ez, cx, cy, cz, ux, uy, uz) {
    let fx = cx - ex, fy = cy - ey, fz = cz - ez;
    const fl = Math.hypot(fx, fy, fz) || 1; fx /= fl; fy /= fl; fz /= fl;
    let sx = fy * uz - fz * uy, sy = fz * ux - fx * uz, sz = fx * uy - fy * ux;
    const sl = Math.hypot(sx, sy, sz) || 1; sx /= sl; sy /= sl; sz /= sl;
    const ux2 = sy * fz - sz * fy, uy2 = sz * fx - sx * fz, uz2 = sx * fy - sy * fx;
    const m = new Float32Array(16);
    m[0] = sx; m[4] = sy; m[8] = sz; m[1] = ux2; m[5] = uy2; m[9] = uz2; m[2] = -fx; m[6] = -fy; m[10] = -fz; m[15] = 1;
    m[12] = -(sx * ex + sy * ey + sz * ez); m[13] = -(ux2 * ex + uy2 * ey + uz2 * ez); m[14] = fx * ex + fy * ey + fz * ez;
    return m;
  },
  // 平移 × 绕 z 转（度）× 缩放，与 GridSceneRenderer.transform() 相同
  model(x = 0, y = 0, z = 0, yawDeg = 0, sx = 1, sy = sx, sz = sx) {
    const a = yawDeg * Math.PI / 180, c = Math.cos(a), s = Math.sin(a), m = new Float32Array(16);
    m[0] = c * sx; m[1] = s * sx; m[4] = -s * sy; m[5] = c * sy; m[10] = sz; m[12] = x; m[13] = y; m[14] = z; m[15] = 1;
    return m;
  },
  apply(m, x, y, z) {
    return [m[0] * x + m[4] * y + m[8] * z + m[12], m[1] * x + m[5] * y + m[9] * z + m[13],
      m[2] * x + m[6] * y + m[10] * z + m[14], m[3] * x + m[7] * y + m[11] * z + m[15]];
  },
};

// ======================= 网格（ui/scene/SceneMesh.kt） =======================
const STRIDE = 10; // 位置 3、法线 3、RGBA 4
class MeshBuilder {
  constructor() { this.v = []; }
  vertex(p, n, c) { this.v.push(p[0], p[1], p[2], n[0], n[1], n[2], c[0], c[1], c[2], c.length > 3 ? c[3] : 1); }
  triangle(a, b, c, color, normal) {
    let n = normal;
    if (!n) {
      const u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]], w = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
      n = [u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]];
      const l = Math.hypot(n[0], n[1], n[2]) || 1; n = [n[0] / l, n[1] / l, n[2] / l];
    }
    this.vertex(a, n, color); this.vertex(b, n, color); this.vertex(c, n, color);
  }
  quad(a, b, c, d, color) { this.triangle(a, b, c, color); this.triangle(a, c, d, color); }
  box(x0, y0, z0, x1, y1, z1, color) {
    const a = [x0, y0, z0], b = [x1, y0, z0], c = [x1, y1, z0], d = [x0, y1, z0];
    const e = [x0, y0, z1], f = [x1, y0, z1], g = [x1, y1, z1], h = [x0, y1, z1];
    this.quad(e, f, g, h, color); this.quad(d, c, b, a, color); this.quad(a, b, f, e, color);
    this.quad(b, c, g, f, color); this.quad(c, d, h, g, color); this.quad(d, a, e, h, color);
  }
  cylinder(start, end, radius, color, sides = 24) {
    let ax = [end[0] - start[0], end[1] - start[1], end[2] - start[2]];
    const al = Math.hypot(...ax) || 1; ax = ax.map((v) => v / al);
    const ref = Math.abs(ax[2]) < 0.8 ? [0, 0, 1] : [0, 1, 0];
    let u = [ax[1] * ref[2] - ax[2] * ref[1], ax[2] * ref[0] - ax[0] * ref[2], ax[0] * ref[1] - ax[1] * ref[0]];
    const ul = Math.hypot(...u) || 1; u = u.map((v) => v / ul);
    const v = [ax[1] * u[2] - ax[2] * u[1], ax[2] * u[0] - ax[0] * u[2], ax[0] * u[1] - ax[1] * u[0]];
    const at = (p, n, r) => [p[0] + n[0] * r, p[1] + n[1] * r, p[2] + n[2] * r];
    for (let i = 0; i < sides; i++) {
      const t = 2 * Math.PI * i / sides, t1 = 2 * Math.PI * (i + 1) / sides;
      const n = [0, 1, 2].map((k) => u[k] * Math.cos(t) + v[k] * Math.sin(t));
      const n1 = [0, 1, 2].map((k) => u[k] * Math.cos(t1) + v[k] * Math.sin(t1));
      const a = at(start, n, radius), b = at(start, n1, radius), c = at(end, n1, radius), d = at(end, n, radius);
      this.vertex(a, n, color); this.vertex(b, n1, color); this.vertex(c, n1, color);
      this.vertex(a, n, color); this.vertex(c, n1, color); this.vertex(d, n, color);
      this.triangle(start, b, a, color, ax.map((x) => -x)); this.triangle(end, d, c, color, ax);
    }
  }
  mesh() { return new Float32Array(this.v); }
}

const COLORS = {
  white: [0.96, 0.97, 0.98, 1], routeBorder: [0.11, 0.16, 0.19, 1], blue: [0.22, 0.66, 0.84, 1],
  focus: [0.66, 0.91, 0.98, 1], amber: [0.98, 0.67, 0.27, 1], teal: [0.33, 0.90, 0.55, 1],
};
const UP = [0, 0, 1];
const Meshes = {
  ground(e) { const b = new MeshBuilder(); b.quad([-e, -e, 0], [e, -e, 0], [e, e, 0], [-e, e, 0], COLORS.white); return b.mesh(); },
  // 先在 double 精度下减去显示原点，大坐标不会在 GPU 中损失厘米级细节
  ribbon(points, anchor, width, z, color, dashed = false) {
    const b = new MeshBuilder(); let along = 0;
    for (let i = 1; i < points.length; i++) {
      const a = points[i - 1], c = points[i];
      const dx = c[0] - a[0], dy = c[1] - a[1], len = Math.hypot(dx, dy);
      if (!(len > 1e-6)) continue;
      const nx = -dy / len * width / 2, ny = dx / len * width / 2;
      const strip = (from, to) => {
        const x0 = a[0] - anchor[0] + dx * from / len, y0 = a[1] - anchor[1] + dy * from / len;
        const x1 = a[0] - anchor[0] + dx * to / len, y1 = a[1] - anchor[1] + dy * to / len;
        b.quad([x0 + nx, y0 + ny, z], [x0 - nx, y0 - ny, z], [x1 - nx, y1 - ny, z], [x1 + nx, y1 + ny, z], color);
      };
      if (!dashed) strip(0, len);
      else {
        const period = Math.max(0.8, len / 1024); let t = 0;
        while (t < len) {
          const phase = (along + t) % period, end = Math.min(len, t + period - phase);
          if (phase < period * 0.58) strip(t, Math.min(end, t + period * 0.58 - phase));
          t = Math.max(end, t + 1e-6);
        }
      }
      along += len;
    }
    return b.mesh();
  },
  ring(radius, width, color) {
    const pts = []; for (let i = 0; i <= 64; i++) pts.push([radius * Math.cos(i * 2 * Math.PI / 64), radius * Math.sin(i * 2 * Math.PI / 64)]);
    return Meshes.ribbon(pts, [0, 0], width, 0.02, color);
  },
  arrow(front) {
    const b = new MeshBuilder(), z = 0.028;
    b.box(front + 0.12, -0.038, z, front + 0.55, 0.038, z + 0.006, COLORS.blue);
    b.triangle([front + 0.8, 0, z], [front + 0.5, 0.16, z], [front + 0.5, -0.16, z], COLORS.blue, UP);
    return b.mesh();
  },
  // 模型加载失败时的通用示意车（NavigationVehicleMesh）
  vehicle(body, wheelbase, steerDeg) {
    const b = new MeshBuilder(), l = body.length, w = body.width;
    const r = clamp(Math.min(w * 0.19, l * 0.17), 0.06, 0.22);
    const dark = [0.105, 0.135, 0.15, 1], steel = [0.43, 0.49, 0.50, 1], shell = [0.89, 0.92, 0.89, 1];
    const frontAxle = clamp(fin(wheelbase) ? wheelbase : l * 0.65, l * 0.2, Math.max(body.front, l * 0.2));
    const wheelW = w * 0.17, wheelY = w * 0.5 - wheelW * 0.5;
    for (const x of [0, frontAxle]) for (const side of [-1, 1]) {
      const ang = x === frontAxle ? steerDeg * Math.PI / 180 : 0, axle = [-Math.sin(ang), Math.cos(ang), 0];
      const c = [x, side * wheelY, r];
      const off = (k) => [[c[0] - axle[0] * k, c[1] - axle[1] * k, c[2]], [c[0] + axle[0] * k, c[1] + axle[1] * k, c[2]]];
      b.cylinder(...off(wheelW * 0.5), r, dark); b.cylinder(...off(wheelW * 0.52), r * 0.51, steel);
      b.cylinder(...off(wheelW * 0.54), r * 0.18, COLORS.blue, 12);
    }
    const z = r * 1.05;
    b.box(-body.rear, -w * 0.36, z, body.front, w * 0.36, z + r * 0.8, dark);
    b.box(-body.rear * 0.75, -w * 0.31, z + r * 0.7, body.front * 0.94, w * 0.31, z + r * 1.6, shell);
    b.box(body.front * 0.56, -w * 0.315, z + r * 0.78, body.front * 0.97, w * 0.315, z + r * 1.2, COLORS.blue);
    b.box(-body.rear * 0.3, -w * 0.25, z + r * 1.6, body.front * 0.56, w * 0.25, z + r * 2.12, shell);
    b.box(body.front * 0.6, -w * 0.2, z + r * 1.6, body.front * 0.88, w * 0.2, z + r * 1.84, COLORS.blue);
    b.box(body.front * 0.96, -w * 0.22, z + r * 1.28, body.front, w * 0.22, z + r * 1.5, COLORS.white);
    return b.mesh();
  },
};

// ======================= 车辆模型（ui/scene/RiggedVehicle.kt 的 .wrmesh） =======================
function parseVehicle(buffer) {
  const view = new DataView(buffer);
  const magic = new TextDecoder('ascii').decode(new Uint8Array(buffer, 0, 8));
  if (magic !== 'WRIG0001') throw new Error('未知的车辆模型');
  const size = view.getInt32(8, true);
  const header = JSON.parse(new TextDecoder('utf-8').decode(new Uint8Array(buffer, 12, size)));
  if (header.version !== 1 || header.forward !== '+X' || header.up !== '+Z') throw new Error('车辆模型坐标约定不对');
  let offset = 12 + size;
  const bones = header.bones.map((b) => ({ name: b.name, parent: b.parent, rest: new Float32Array(b.rest) }));
  const parts = header.parts.map((p) => {
    const floats = p.vertices * STRIDE;
    const data = new Float32Array(buffer.slice(offset, offset + floats * 4)); // 文件是小端序，与浏览器相同
    offset += floats * 4;
    return { bone: p.bone, data };
  });
  const rig = { name: header.name, wheelRadius: header.wheelRadius, wheelbase: header.wheelbase, bones, parts };
  // 中立外廓：用实际顶点算，与 app 相同
  const rest = poseRig(rig, 0, 0), lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
  for (const part of parts) {
    const m = rest[part.bone], v = part.data;
    for (let i = 0; i < v.length; i += STRIDE) for (let k = 0; k < 3; k++) {
      const p = m[k] * v[i] + m[4 + k] * v[i + 1] + m[8 + k] * v[i + 2] + m[12 + k];
      if (p < lo[k]) lo[k] = p; if (p > hi[k]) hi[k] = p;
    }
  }
  rig.dimensions = [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]];
  return rig;
}
function poseRig(rig, steerDeg, roll) {
  const out = [];
  rig.bones.forEach((bone, i) => {
    const rot = M4.identity(), steer = bone.name.startsWith('steer_'), wheel = bone.name.startsWith('wheel_');
    if (steer || wheel) {
      const a = steer ? steerDeg * Math.PI / 180 : roll, c = Math.cos(a), s = Math.sin(a);
      if (steer) { rot[0] = c; rot[1] = s; rot[4] = -s; rot[5] = c; } else { rot[0] = c; rot[2] = -s; rot[8] = s; rot[10] = c; }
    }
    const local = M4.mul(bone.rest, rot);
    out[i] = bone.parent < 0 ? local : M4.mul(out[bone.parent], local);
  });
  return out;
}
const displayScale = (rig, body) => { const k = body.length / rig.dimensions[0]; return [k, body.width / rig.dimensions[1], k]; };

// ======================= 着色器（与 GridSceneRenderer 相同） =======================
const VERTEX = `
attribute vec3 aPosition; attribute vec3 aNormal; attribute vec4 aColor;
uniform mat4 uModel; uniform mat4 uVp; uniform mat4 uLightVp;
varying vec3 vPosition; varying vec3 vNormal; varying vec4 vColor; varying vec4 vShadow;
void main() {
  vec4 p=uModel*vec4(aPosition,1.0);
  mat3 normalMatrix=mat3(cross(uModel[1].xyz,uModel[2].xyz),cross(uModel[2].xyz,uModel[0].xyz),cross(uModel[0].xyz,uModel[1].xyz));
  vPosition=p.xyz; vNormal=normalMatrix*aNormal; vColor=aColor;
  vShadow=uLightVp*p; gl_Position=uVp*p;
}`;
const FRAGMENT = `
precision highp float;
uniform sampler2D uGrid; uniform sampler2D uShadow; uniform sampler2D uSurface;
uniform float uMaterial; uniform float uHasShadow; uniform vec3 uEye;
uniform vec4 uVehicle; uniform vec4 uFootprint;
uniform float uGridStep; uniform vec2 uGridOffset; uniform float uFogDistance;
uniform vec2 uSurfaceOffset; uniform vec3 uGroundFocus;
varying vec3 vPosition; varying vec3 vNormal; varying vec4 vColor; varying vec4 vShadow;
float unpackDepth(vec4 c) { return dot(c,vec4(1.0,1.0/255.0,1.0/65025.0,1.0/16581375.0)); }
#ifdef SHARP_GRID
float gridLine(vec2 p, float width) {
  vec2 footprint=max(fwidth(p),vec2(0.000001));
  vec2 pixels=abs(fract(p+0.5)-0.5)/footprint;
  vec2 coverage=1.0-smoothstep(vec2(width*0.5-0.5),vec2(width*0.5+0.5),pixels);
  coverage*=1.0-smoothstep(vec2(0.12),vec2(0.35),footprint);
  return max(coverage.x,coverage.y);
}
#endif
float shadow() {
  if(uHasShadow<0.5) return 1.0;
  vec3 p=vShadow.xyz/vShadow.w*0.5+0.5;
  if(p.x<0.0||p.x>1.0||p.y<0.0||p.y>1.0||p.z>1.0) return 1.0;
  float light=0.0;
  for(int x=-1;x<=1;x++) for(int y=-1;y<=1;y++) {
    float d=unpackDepth(texture2D(uShadow,p.xy+vec2(float(x),float(y))*0.0018));
    light+=step(p.z-0.0018,d);
  }
  return 0.48+0.52*light/9.0;
}
void main() {
  vec3 n=normalize(vNormal); vec3 color=vColor.rgb; float rough=0.72; float ao=1.0;
  if(uMaterial>0.5 && uMaterial<1.5) {
    vec2 surfaceUv=vPosition.xy/8.0+uSurfaceOffset;
    vec2 finish=texture2D(uSurface,surfaceUv).rg-0.5;
    float grain=texture2D(uSurface,surfaceUv*8.0).b-0.5;
    float detail=1.0-smoothstep(12.0,35.0,uGroundFocus.z);
    color=vec3(0.255,0.29,0.31)+(finish.x*0.022+finish.y*0.012+grain*0.018)*detail;
    vec2 groundDelta=(vPosition.xy-uGroundFocus.xy)/max(3.5,uGroundFocus.z*1.25);
    color+=vec3(0.055,0.058,0.058)*exp(-dot(groundDelta,groundDelta));
    #ifdef SHARP_GRID
    vec2 coordinates=vPosition.xy/uGridStep+uGridOffset;
    vec2 grid=vec2(gridLine(coordinates,1.1),gridLine(coordinates/5.0,1.7));
    #else
    vec2 grid=texture2D(uGrid,(vPosition.xy/uGridStep+uGridOffset)/5.0).rg;
    #endif
    float fade=1.0-smoothstep(uFogDistance*0.25,uFogDistance*0.85,length(uEye-vPosition));
    color=mix(color,vec3(0.58,0.63,0.66),grid.r*0.42*fade);
    color=mix(color,vec3(0.64,0.69,0.72),grid.g*0.52*fade);
    vec2 delta=vPosition.xy-uVehicle.xy;
    vec2 local=vec2(dot(delta,uVehicle.zw),dot(delta,vec2(-uVehicle.w,uVehicle.z)));
    float mid=(uFootprint.x-uFootprint.y)*0.5;
    vec2 q=vec2((local.x-mid)/(0.2+(uFootprint.x+uFootprint.y)*0.65),local.y/(uFootprint.z+0.13));
    ao=1.0-0.13*exp(-dot(q,q)*1.8)*uFootprint.w;
  }
  vec3 lightDir=normalize(vec3(-4.0,-3.0,7.0));
  float diffuse=max(dot(n,lightDir),0.0);
  float shade=shadow();
  vec3 result=color*(0.60+0.45*diffuse*shade)*ao;
  if(uMaterial<0.5) result=color;
  if(uMaterial>0.5 && uMaterial<1.5) result=color*(0.70+0.30*shade)*ao;
  if(uMaterial>1.5) {
    vec3 halfDir=normalize(lightDir+normalize(uEye-vPosition));
    result+=vec3(0.16)*pow(max(dot(n,halfDir),0.0),mix(42.0,8.0,rough))*shade;
  }
  float fog=uMaterial>0.5 && uMaterial<1.5 ? smoothstep(uFogDistance*0.5,uFogDistance,length(uEye-vPosition)) : 0.0;
  gl_FragColor=vec4(mix(result,vec3(0.19,0.23,0.265),fog),1.0);
}`;
const DEPTH_VERTEX = `attribute vec3 aPosition; uniform mat4 uModel; uniform mat4 uVp;
void main() { gl_Position=uVp*uModel*vec4(aPosition,1.0); }`;
const DEPTH_FRAGMENT = `precision highp float;
void main() {
  vec4 c=fract(gl_FragCoord.z*vec4(1.0,255.0,65025.0,16581375.0));
  c-=c.yzww*vec4(1.0/255.0,1.0/255.0,1.0/255.0,0.0);
  gl_FragColor=c;
}`;

// ======================= 镜头（ui/scene/SceneData.kt 的 SceneCamera） =======================
const FOV_RAD = 48 * Math.PI / 180;
const GRID_STEPS = [0.2, 0.5, 1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000];
const gridStepFor = (mpp) => GRID_STEPS.find((s) => s >= mpp * 40) || GRID_STEPS[GRID_STEPS.length - 1];
const metresLabel = (v) => (v < 1 ? v.toFixed(1) : String(Math.trunc(v)));
// 沿车身中线的固定跟车镜头：观察距离、俯角和相对车头的方位不受触屏/滚轮影响。
class SceneCamera {
  constructor() {
    this.x = 0; this.y = 0; this.yaw = 0; this.pitch = 30 * Math.PI / 180;
    this.distance = 3.2; this.ready = false;
  }
  reset() { this.ready = false; }
  frame(scene, aspect, dt) {
    const display = scene.displayVehicle, p = display.preview ? null : display, bounds = scene.bounds;
    const showcase = display.preview && scene.route.length === 0;
    const follow = p !== null, body = scene.body;
    const vehicleHeading = display.yawDeg * Math.PI / 180;
    const heading = showcase || follow ? vehicleHeading : Math.PI / 2;
    const lead = Math.max(0.95, body.length * 1.25);
    const tx = showcase || follow ? display.x + Math.cos(vehicleHeading) * lead : bounds.x;
    const ty = showcase || follow ? display.y + Math.sin(vehicleHeading) * lead : bounds.y;
    const base = clamp(showcase || follow ? Math.max(3.2, body.length * 4.3) / Math.min(1, aspect)
        : bounds.radius / Math.sin(Math.atan(Math.tan(FOV_RAD / 2) * Math.min(1, aspect))) * 1.18, 2, 30000);
    const wantedPitch = (showcase || follow ? 30 : 57) * Math.PI / 180;
    if (!this.ready) { this.x = tx; this.y = ty; this.yaw = heading; this.pitch = wantedPitch; this.distance = base; this.ready = true; }
    const k = 1 - Math.exp(-clamp(dt, 0, 0.5) / 0.24);
    if (!scene.positionHeld) {
      this.x += (tx - this.x) * k; this.y += (ty - this.y) * k;
      this.yaw = wrap(this.yaw + wrap(heading - this.yaw) * k);
      this.distance += (base - this.distance) * k; this.pitch += (wantedPitch - this.pitch) * k;
    }
    const d = this.distance, horizontal = Math.cos(this.pitch) * d, th = 0.12;
    return { eye: [this.x - Math.cos(this.yaw) * horizontal, this.y - Math.sin(this.yaw) * horizontal, Math.sin(this.pitch) * d + th],
      target: [this.x, this.y, th], heading: this.yaw, distance: d,
      moving: !scene.positionHeld && (Math.abs(tx - this.x) + Math.abs(ty - this.y) > 0.002 || Math.abs(wrap(heading - this.yaw)) > 0.0005 ||
        Math.abs(base - this.distance) > 0.002 || Math.abs(wantedPitch - this.pitch) > 0.0005) };
  }
}

// 车辆附近那一小段路线（SceneRouteFocus.nearby）：只为高亮，不代表控制器选段
function nearbyRoute(points, x, y) {
  let nearest = Infinity, centre = 0, along = 0;
  for (let i = 1; i < points.length; i++) {
    const a = points[i - 1], b = points[i], dx = b[0] - a[0], dy = b[1] - a[1], len = Math.hypot(dx, dy);
    if (len < 1e-8) continue;
    const t = clamp(((x - a[0]) * dx + (y - a[1]) * dy) / (len * len), 0, 1);
    const dist = Math.hypot(x - a[0] - t * dx, y - a[1] - t * dy);
    if (dist < nearest) { nearest = dist; centre = along + t * len; }
    along += len;
  }
  if (nearest > 1.5) return [];
  const from = Math.max(0, centre - 1.5), to = centre + 2, out = []; along = 0;
  for (let i = 1; i < points.length; i++) {
    const a = points[i - 1], b = points[i], len = Math.hypot(b[0] - a[0], b[1] - a[1]);
    if (len < 1e-8) continue;
    if (along + len >= from && along <= to) {
      const at = (t) => [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t];
      if (!out.length) out.push(at(clamp((from - along) / len, 0, 1)));
      out.push(at(clamp((to - along) / len, 0, 1)));
    }
    along += len;
    if (along > to) break;
  }
  return out;
}

function boundsOf(points) {
  const v = points.filter((p) => p && fin(p[0]) && fin(p[1]));
  if (!v.length) return { x: 0, y: 0, width: 12, height: 12, radius: Math.hypot(12, 12) / 2 };
  let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
  for (const [x, y] of v) { x0 = Math.min(x0, x); x1 = Math.max(x1, x); y0 = Math.min(y0, y); y1 = Math.max(y1, y); }
  const w = x1 - x0, h = y1 - y0;
  return { x: (x0 + x1) / 2, y: (y0 + y1) / 2, width: w, height: h, radius: Math.max(3, Math.hypot(w, h) / 2) };
}

// ======================= 渲染器（GridSceneRenderer 的 WebGL 版） =======================
const SHADOW_SIZE = 1024;
class Renderer {
  constructor(canvas) {
    this.canvas = canvas;
    const gl = canvas.getContext('webgl', { antialias: true, alpha: false, preserveDrawingBuffer: false });
    if (!gl) throw new Error('这台设备不支持 WebGL');
    this.gl = gl;
    this.sharp = !!gl.getExtension('OES_standard_derivatives');
    const header = this.sharp ? '#extension GL_OES_standard_derivatives : enable\n#define SHARP_GRID\n' : '';
    this.lit = this.program(VERTEX, header + FRAGMENT);
    this.depth = this.program(DEPTH_VERTEX, DEPTH_FRAGMENT);
    this.meshes = new Map();
    this.gridTexture = this.createGridTexture();
    this.surfaceTexture = this.createSurfaceTexture();
    this.createShadowBuffer();
    this.put('ground', Meshes.ground(1));
    this.put('antennaRing', Meshes.ring(0.18, 0.035, COLORS.blue));
    this.camera = new SceneCamera();
    this.anchor = [0, 0];
    this.lineWidth = 0.10;
    this.lastTime = 0;
    this.rig = null; this.bones = [];
    this.vehicleKey = null;
    this.shadowKey = null; this.lightVp = M4.identity();
    this.projection = { heading: 0, ppm: 0, gridStep: 1, labels: [] };
    gl.enable(gl.DEPTH_TEST); gl.disable(gl.CULL_FACE);
  }

  program(vs, fs) {
    const gl = this.gl;
    const compile = (type, src) => {
      const s = gl.createShader(type); gl.shaderSource(s, src); gl.compileShader(s);
      if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(s));
      return s;
    };
    const p = gl.createProgram();
    gl.attachShader(p, compile(gl.VERTEX_SHADER, vs)); gl.attachShader(p, compile(gl.FRAGMENT_SHADER, fs));
    gl.bindAttribLocation(p, 0, 'aPosition'); gl.bindAttribLocation(p, 1, 'aNormal'); gl.bindAttribLocation(p, 2, 'aColor');
    gl.linkProgram(p);
    if (!gl.getProgramParameter(p, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(p));
    const uniforms = new Map();
    return { id: p, u: (n) => { if (!uniforms.has(n)) uniforms.set(n, gl.getUniformLocation(p, n)); return uniforms.get(n); },
      normal: gl.getAttribLocation(p, 'aNormal'), color: gl.getAttribLocation(p, 'aColor') };
  }

  put(name, data) {
    const gl = this.gl, old = this.meshes.get(name);
    if (old) gl.deleteBuffer(old.buffer);
    const buffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, buffer); gl.bufferData(gl.ARRAY_BUFFER, data, gl.STATIC_DRAW);
    this.meshes.set(name, { buffer, count: data.length / STRIDE });
  }

  draw(name, program) {
    const gl = this.gl, mesh = this.meshes.get(name);
    if (!mesh || !mesh.count) return;
    gl.bindBuffer(gl.ARRAY_BUFFER, mesh.buffer);
    gl.enableVertexAttribArray(0); gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 40, 0);
    if (program.normal >= 0) { gl.enableVertexAttribArray(program.normal); gl.vertexAttribPointer(program.normal, 3, gl.FLOAT, false, 40, 12); }
    if (program.color >= 0) { gl.enableVertexAttribArray(program.color); gl.vertexAttribPointer(program.color, 4, gl.FLOAT, false, 40, 24); }
    gl.drawArrays(gl.TRIANGLES, 0, mesh.count);
    gl.disableVertexAttribArray(0);
    if (program.normal >= 0) gl.disableVertexAttribArray(program.normal);
    if (program.color >= 0) gl.disableVertexAttribArray(program.color);
  }

  paint(name, material, model) {
    const gl = this.gl;
    gl.uniformMatrix4fv(this.lit.u('uModel'), false, model || M4.identity());
    gl.uniform1f(this.lit.u('uMaterial'), material || 0);
    this.draw(name, this.lit);
  }

  depthPaint(name, model) {
    this.gl.uniformMatrix4fv(this.depth.u('uModel'), false, model);
    this.draw(name, this.depth);
  }

  texture(size, fill, mipmap = true) {
    const gl = this.gl, bytes = new Uint8Array(size * size * 4);
    fill(bytes);
    const t = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, t);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, size, size, 0, gl.RGBA, gl.UNSIGNED_BYTE, bytes);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, mipmap ? gl.LINEAR_MIPMAP_LINEAR : gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.REPEAT);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.REPEAT);
    if (mipmap) gl.generateMipmap(gl.TEXTURE_2D);
    return t;
  }

  createGridTexture() {
    const size = 512, cov = (d, r) => clamp(r + 0.5 - d, 0, 1) * 255;
    return this.texture(size, (b) => {
      for (let y = 0; y < size; y++) for (let x = 0; x < size; x++) {
        const gx = (x + 0.5) * 5 / size, gy = (y + 0.5) * 5 / size;
        const minor = Math.min(Math.abs(gx - Math.round(gx)), Math.abs(gy - Math.round(gy))) * size / 5;
        const major = Math.min(Math.min(x + 0.5, size - x - 0.5), Math.min(y + 0.5, size - y - 0.5));
        const i = (y * size + x) * 4; b[i] = cov(minor, 0.55); b[i + 1] = cov(major, 1.4); b[i + 2] = 0; b[i + 3] = 255;
      }
    });
  }

  createSurfaceTexture() {
    // 周期噪声只生成一次，纹理固定在实地坐标里，不随车辆滑动
    let seed = 982;
    const random = () => { seed = (seed + 0x6D2B79F5) | 0; let t = Math.imul(seed ^ (seed >>> 15), 1 | seed); t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t; return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
    const size = 256, coarse = Array.from({ length: 64 }, random), fine = Array.from({ length: 1024 }, random);
    const sample = (values, cells, x, y) => {
      const px = x * cells / size, py = y * cells / size, ix = Math.trunc(px), iy = Math.trunc(py);
      const fx = px - ix, fy = py - iy, sx = fx * fx * (3 - 2 * fx), sy = fy * fy * (3 - 2 * fy);
      const at = (dx, dy) => values[((iy + dy) % cells) * cells + (ix + dx) % cells];
      const a = at(0, 0) * (1 - sx) + at(1, 0) * sx, c = at(0, 1) * (1 - sx) + at(1, 1) * sx;
      return a * (1 - sy) + c * sy;
    };
    return this.texture(size, (b) => {
      for (let y = 0; y < size; y++) for (let x = 0; x < size; x++) {
        const i = (y * size + x) * 4;
        b[i] = sample(coarse, 8, x, y) * 255; b[i + 1] = sample(fine, 32, x, y) * 255; b[i + 2] = random() * 256; b[i + 3] = 255;
      }
    });
  }

  createShadowBuffer() {
    const gl = this.gl;
    this.shadowTexture = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, this.shadowTexture);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, SHADOW_SIZE, SHADOW_SIZE, 0, gl.RGBA, gl.UNSIGNED_BYTE, null);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
    const depth = gl.createRenderbuffer();
    gl.bindRenderbuffer(gl.RENDERBUFFER, depth);
    gl.renderbufferStorage(gl.RENDERBUFFER, gl.DEPTH_COMPONENT16, SHADOW_SIZE, SHADOW_SIZE);
    this.shadowFbo = gl.createFramebuffer();
    gl.bindFramebuffer(gl.FRAMEBUFFER, this.shadowFbo);
    gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, this.shadowTexture, 0);
    gl.framebufferRenderbuffer(gl.FRAMEBUFFER, gl.DEPTH_ATTACHMENT, gl.RENDERBUFFER, depth);
    if (gl.checkFramebufferStatus(gl.FRAMEBUFFER) !== gl.FRAMEBUFFER_COMPLETE) this.shadowFbo = null;
    gl.bindFramebuffer(gl.FRAMEBUFFER, null);
  }

  setVehicle(rig) {
    this.rig = rig;
    rig.parts.forEach((p, i) => this.put('rig_' + i, p.data));
    this.bones = poseRig(rig, 0, 0);
    this.vehicleKey = null;
    this.shadowKey = null;
  }

  // 数据变了才重建路线色带等网格
  update(scene) {
    const old = this.active, display = scene.displayVehicle;
    const nextAnchor = display.preview ? [display.x, display.y] : (scene.route[0] || [display.x, display.y]);
    const rebase = !old || old.origin !== scene.origin || (old.displayVehicle.preview !== display.preview) ||
      Math.hypot(nextAnchor[0] - this.anchor[0], nextAnchor[1] - this.anchor[1]) > 2000;
    if (rebase) { this.anchor = nextAnchor; this.camera.reset(); }
    if (rebase || old.route !== scene.route || old.reference !== scene.reference || old.trail !== scene.trail) this.rebuildPaths(scene, rebase ? null : old);
    this.rebuildFocus(scene);
    if (this.rig) this.bones = poseRig(this.rig, display.preview ? 0 : scene.steerDeg, display.preview ? 0 : scene.roll);
    const key = JSON.stringify([scene.body, display.preview ? 0 : Math.round(scene.steerDeg)]);
    if (this.vehicleKey !== key) {
      if (!this.rig) this.put('vehicle', Meshes.vehicle(scene.body, scene.wheelbase, display.preview ? 0 : Math.round(scene.steerDeg)));
      this.put('arrow', Meshes.arrow(scene.body.front));
      this.vehicleKey = key;
    }
    this.active = scene;
  }

  rebuildPaths(scene, old) {
    const a = this.anchor, w = this.lineWidth;
    if (!old || old.route !== scene.route) {
      this.put('routeEdge', Meshes.ribbon(scene.route, a, w * 1.35, 0.009, COLORS.routeBorder));
      this.put('route', Meshes.ribbon(scene.route, a, w, 0.015, COLORS.blue));
    }
    if (!old || old.reference !== scene.reference) this.put('reference', Meshes.ribbon(scene.reference, a, w * 0.48, 0.025, COLORS.focus, true));
    if (!old || old.trail !== scene.trail) this.put('trail', Meshes.ribbon(scene.trail, a, w * 1.18, 0.055, COLORS.teal));
  }

  rebuildFocus(scene) {
    const v = scene.vehicle;
    const pts = v && v.usable ? nearbyRoute(scene.route, v.x, v.y) : [];
    const key = JSON.stringify(pts) + this.lineWidth;
    if (key !== this.focusKey) { this.put('focus', Meshes.ribbon(pts, this.anchor, this.lineWidth, 0.03, COLORS.focus)); this.focusKey = key; }
  }

  render(scene, now) {
    const gl = this.gl, canvas = this.canvas;
    const w = Math.max(1, Math.round(canvas.clientWidth * (window.devicePixelRatio || 1)));
    const h = Math.max(1, Math.round(canvas.clientHeight * (window.devicePixelRatio || 1)));
    if (canvas.width !== w || canvas.height !== h) { canvas.width = w; canvas.height = h; }
    if (scene !== this.active) this.update(scene);
    const dt = this.lastTime ? (now - this.lastTime) / 1000 : 0.033;
    this.lastTime = now;
    const frame = this.camera.frame(scene, w / h, dt), a = this.anchor, eye = frame.eye, target = frame.target;
    const view = M4.lookAt(eye[0] - a[0], eye[1] - a[1], eye[2], target[0] - a[0], target[1] - a[1], target[2], 0, 0, 1);
    const far = Math.min(60000, Math.max(500, frame.distance * 4 + scene.bounds.radius * 2));
    const vp = M4.mul(M4.perspective(48, w / h, Math.max(0.05, far / 100000), far), view);
    const mpp = frame.distance * Math.tan(FOV_RAD / 2) * 2 / h;
    const gridStep = gridStepFor(mpp);
    const visible = clamp(mpp * 2.8, 0.10, 2.0);
    if (Math.abs(visible - this.lineWidth) > Math.max(0.02, this.lineWidth * 0.25)) {
      this.lineWidth = visible; this.rebuildPaths(scene, null); this.rebuildFocus(scene);
    }
    const p = scene.displayVehicle, drawVehicle = !p.hidden;
    // 阴影不依赖观察镜头。平移、旋转、缩放时复用，只在车辆/轮子/显示原点变化后重画。
    if (this.shadowFbo && drawVehicle) {
      const x = p.x - a[0], y = p.y - a[1], r = Math.max(3, Math.max(scene.body.length, scene.body.width) * 2);
      const key = JSON.stringify([x, y, p.yawDeg, scene.body, p.preview ? 0 : scene.steerDeg, p.preview ? 0 : scene.roll]);
      if (key !== this.shadowKey) {
        this.lightVp = M4.mul(M4.ortho(-r, r, -r, r, 0.1, 15 * r), M4.lookAt(x - 4 * r, y - 3 * r, 7 * r, x, y, 0, 0, 0, 1));
        gl.bindFramebuffer(gl.FRAMEBUFFER, this.shadowFbo);
        gl.viewport(0, 0, SHADOW_SIZE, SHADOW_SIZE);
        gl.clearColor(1, 1, 1, 1); gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
        gl.useProgram(this.depth.id);
        gl.uniformMatrix4fv(this.depth.u('uVp'), false, this.lightVp);
        gl.enable(gl.POLYGON_OFFSET_FILL); gl.polygonOffset(2, 3);
        this.drawVehicle(scene, true);
        gl.disable(gl.POLYGON_OFFSET_FILL);
        this.shadowKey = key;
      }
    }
    gl.bindFramebuffer(gl.FRAMEBUFFER, null);
    gl.viewport(0, 0, w, h);
    gl.clearColor(0.19, 0.23, 0.265, 1); gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    const L = this.lit; gl.useProgram(L.id);
    gl.uniformMatrix4fv(L.u('uVp'), false, vp);
    gl.uniformMatrix4fv(L.u('uLightVp'), false, this.lightVp);
    gl.uniform3f(L.u('uEye'), eye[0] - a[0], eye[1] - a[1], eye[2]);
    gl.activeTexture(gl.TEXTURE0); gl.bindTexture(gl.TEXTURE_2D, this.gridTexture); gl.uniform1i(L.u('uGrid'), 0);
    gl.uniform1f(L.u('uGridStep'), gridStep);
    gl.uniform2f(L.u('uGridOffset'), (a[0] % (gridStep * 5)) / gridStep, (a[1] % (gridStep * 5)) / gridStep);
    gl.uniform1f(L.u('uFogDistance'), Math.max(20, frame.distance * 5));
    gl.uniform3f(L.u('uGroundFocus'), target[0] - a[0], target[1] - a[1], frame.distance);
    gl.activeTexture(gl.TEXTURE2); gl.bindTexture(gl.TEXTURE_2D, this.surfaceTexture); gl.uniform1i(L.u('uSurface'), 2);
    gl.uniform2f(L.u('uSurfaceOffset'), (a[0] % 8) / 8, (a[1] % 8) / 8);
    gl.activeTexture(gl.TEXTURE1); gl.bindTexture(gl.TEXTURE_2D, this.shadowTexture); gl.uniform1i(L.u('uShadow'), 1);
    gl.uniform1f(L.u('uHasShadow'), this.shadowFbo && drawVehicle ? 1 : 0);
    const yaw = p.yawDeg * Math.PI / 180;
    gl.uniform4f(L.u('uVehicle'), p.x - a[0], p.y - a[1], Math.cos(yaw), Math.sin(yaw));
    gl.uniform4f(L.u('uFootprint'), scene.body.front, scene.body.rear, scene.body.width / 2, drawVehicle ? 1 : 0);
    // 地面只覆盖镜头附近，避免巨大三角形插值损失近处网格精度
    this.paint('ground', 1, M4.model(target[0] - a[0], target[1] - a[1], 0, 0, Math.min(30000, Math.max(600, frame.distance * 12))));
    // 共面的导航叠层按顺序画，远视角下也不闪
    gl.disable(gl.DEPTH_TEST); gl.depthMask(false);
    for (const name of ['routeEdge', 'route', 'reference', 'focus', 'trail']) this.paint(name);
    if (drawVehicle && !p.preview) this.paint('arrow', 0, M4.model(p.x - a[0], p.y - a[1], 0, p.yawDeg));
    if (scene.antennaPoint && !scene.vehicle) this.paint('antennaRing', 0, M4.model(scene.antennaPoint[0] - a[0], scene.antennaPoint[1] - a[1]));
    gl.depthMask(true); gl.enable(gl.DEPTH_TEST);
    if (drawVehicle) this.drawVehicle(scene, false);
    this.project(scene, frame, vp, w, h, gridStep);
    return frame.moving;
  }

  drawVehicle(scene, shadow) {
    const p = scene.displayVehicle, a = this.anchor, rig = this.rig;
    if (rig) {
      const s = displayScale(rig, scene.body);
      const vehicle = M4.mul(M4.model(p.x - a[0], p.y - a[1], 0, p.yawDeg), M4.model(0, 0, 0, 0, s[0], s[1], s[2]));
      rig.parts.forEach((part, i) => {
        const m = M4.mul(vehicle, this.bones[part.bone]);
        if (shadow) this.depthPaint('rig_' + i, m); else this.paint('rig_' + i, 2, m);
      });
    } else {
      const m = M4.model(p.x - a[0], p.y - a[1], 0, p.yawDeg);
      if (shadow) this.depthPaint('vehicle', m); else this.paint('vehicle', 2, m);
    }
  }

  project(scene, frame, vp, w, h, gridStep) {
    const a = this.anchor, css = window.devicePixelRatio || 1;
    const at = (x, y, z) => {
      const c = M4.apply(vp, x - a[0], y - a[1], z);
      if (c[3] <= 0 || Math.abs(c[2]) > c[3]) return null;
      const sx = (c[0] / c[3] + 1) * w / 2, sy = (1 - c[1] / c[3]) * h / 2;
      return sx >= 0 && sx <= w && sy >= 0 && sy <= h ? [sx / css, sy / css] : null;
    };
    const labels = [];
    (scene.controlPoints || []).forEach((point, index) => {
      let n = index + 1, name = '';
      while (n > 0) { n--; name = String.fromCharCode(65 + n % 26) + name; n = Math.floor(n / 26); }
      const q = at(point[0], point[1], 0.1);
      if (q) labels.push({ text: '⊕ ' + name, x: q[0], y: q[1] });
    });
    const p = scene.displayVehicle;
    if (frame.distance > 24 && !p.preview && !p.hidden) { const q = at(p.x, p.y, 0.6); if (q) labels.push({ text: '车辆', x: q[0], y: q[1] }); }
    const t = frame.target, zero = at(t[0], t[1], 0), metre = at(t[0] + Math.sin(frame.heading), t[1] - Math.cos(frame.heading), 0);
    const ppm = zero && metre ? Math.hypot(zero[0] - metre[0], zero[1] - metre[1]) : 0;
    this.projection = { heading: frame.heading, ppm, gridStep, labels };
  }
}

// ======================= 数据：事件流 → 场景 =======================
const state = {
  version: 0, msg: null, lastAt: 0, release: undefined, trail: [], trailKey: null, lastPhase: 'IDLE',
  plan: [], planRoute: null, route: [], roll: 0, prevPose: null, prevPoseAt: 0, steerDeg: 0,
  fixedPose: null, fixedOrigin: null,
  view: null,   // 平板正在预览的图纸：{name, points, kind, closed, count, origin}
};

function drawnRoute(route) {
  if (!route) return [];
  if (route.kind === 'circle') {
    if (!route.center) return [];
    const [cx, cy] = route.center, r = route.radius || 3;
    return Array.from({ length: 129 }, (_, i) => [cx + r * Math.cos(2 * Math.PI * i / 128), cy + r * Math.sin(2 * Math.PI * i / 128)]);
  }
  const pts = (route.points || []).filter((p) => fin(p[0]) && fin(p[1]));
  return route.closed && pts.length >= 3 ? pts.concat([pts[0]]) : pts;
}

function online(now) { return state.msg !== null && now - state.lastAt < 2500; }

function confirmedPose(msg) {
  const rtk = msg && msg.tel.rtk, line = msg && msg.tel.line, p = line && line.pose;
  return rtk && !rtk.stale && rtk.fix === 'FIXED' && line && !line.stale && line.origin &&
    p && p.usable && p.quality === 'RTK_FIXED' && !(p.reasons || []).includes('heading_uninitialized') &&
    fin(p.x) && fin(p.y) && fin(p.yaw_deg) ? p : null;
}

function onMessage(msg) {
  const now = performance.now();
  if (state.release === undefined) state.release = msg.release;
  else if (msg.release && msg.release !== state.release) { location.reload(); return; } // 车端换了发布：用新页面
  state.msg = msg; state.lastAt = now;
  const line = msg.tel.line || {};
  if (msg.plan) state.plan = msg.plan.points || [];
  if (msg.view) state.view = msg.view.file;
  const route = line.stale ? null : line.route;
  const routeKey = route ? JSON.stringify([route.kind, route.points, route.closed, route.center, route.radius, route.laps]) : null;
  if (routeKey !== state.planRoute) { state.planRoute = routeKey; state.route = route && route.kind === 'path' ? drawnRoute(route) : (route ? state.plan : []); }
  if (route && route.kind !== 'path') state.route = state.plan;
  // 轨迹：开始新一趟路线作业时清空；连线路线记车身中心（要求车身中心压线），其余记后轴中心
  const phase = line.phase || 'IDLE';
  if (phase !== 'IDLE' && state.lastPhase === 'IDLE') state.trail = [];
  state.lastPhase = phase;
  const originKey = JSON.stringify(line.origin || null);
  if (originKey !== state.trailKey) {
    state.trailKey = originKey; state.trail = []; state.prevPose = null; state.prevPoseAt = 0;
  }
  if (state.fixedPose && line.origin && originKey !== state.fixedOrigin) {
    state.fixedPose = null; state.fixedOrigin = null; state.roll = 0; state.steerDeg = 0;
  }
  const pose = confirmedPose(msg), car = msg.car;
  if (pose) {
    state.fixedPose = { ...pose, reasons: [...(pose.reasons || [])] }; state.fixedOrigin = originKey;
    const yaw = pose.yaw_deg * Math.PI / 180, c = route && route.kind === 'path' ? 0.5 * (car.front_m - car.rear_m) : 0;
    const q = [pose.x + c * Math.cos(yaw), pose.y + c * Math.sin(yaw)], last = state.trail[state.trail.length - 1];
    if (!last || Math.hypot(q[0] - last[0], q[1] - last[1]) > 0.05) {
      state.trail = state.trail.concat([q]);
      if (state.trail.length > 2048) state.trail = state.trail.slice(-2048);
    }
    // 轮子滚动：只按收到的位置增量（VehicleWheelRoll），停车、失联不自己转
    const prev = state.prevPose, speed = pose.speed || 0;
    if (prev && prev.usable && now - state.prevPoseAt < 1000 && Math.abs(speed) > 0.015) {
      const dx = pose.x - prev.x, dy = pose.y - prev.y, dist = Math.hypot(dx, dy);
      if (dist <= Math.min(0.3, Math.max(0.05, Math.abs(speed) * (now - state.prevPoseAt) / 1000 * 3))) {
        const my = prev.yaw_deg * Math.PI / 180 + wrap((pose.yaw_deg - prev.yaw_deg) * Math.PI / 180) / 2;
        const travel = dx * Math.cos(my) + dy * Math.sin(my);
        if (travel * speed >= 0 && renderer && renderer.rig) state.roll = (state.roll + travel / (renderer.rig.wheelRadius * displayScale(renderer.rig, bodyOf(car))[0])) % (2 * Math.PI);
      }
    }
    // 前轮转角：手动时车端直接给；自动时按车速和角速度反算（只用于画轮子）
    const drive = msg.tel.drive || {};
    if (drive.manual && fin(drive.steer_deg)) state.steerDeg = drive.steer_deg;
    else if (fin(drive.speed) && fin(drive.yaw_rate) && Math.abs(drive.speed) > 0.05) {
      state.steerDeg = clamp(Math.atan(car.wheelbase_m * drive.yaw_rate / drive.speed) * 180 / Math.PI, -25, 25);
    } else if (!fin(drive.speed) || Math.abs(drive.speed) <= 0.05) state.steerDeg *= 0.9;
  }
  state.prevPose = pose; state.prevPoseAt = now;
  const r3 = (v) => (fin(v) ? Math.round(v * 1000) : null);
  const shown = state.fixedPose;
  const sig = JSON.stringify([shown && [r3(shown.x), r3(shown.y), r3(shown.yaw_deg)], !!pose,
    line.stale, line.origin, msg.tel.rtk && [msg.tel.rtk.fix, msg.tel.rtk.stale], state.planRoute, state.trail.length,
    !!msg.plan, !!msg.view, Math.round(state.steerDeg * 10), Math.round(state.roll * 100)]);
  if (sig !== state.sig) { state.sig = sig; state.version++; dirty = true; }
}

const bodyOf = (car) => ({ front: car.front_m, rear: car.rear_m, width: 2 * car.half_width_m, length: car.front_m + car.rear_m });

let cachedScene = null, sceneKey = null;
function buildScene(now) {
  const msg = state.msg, live = online(now);
  const car = msg ? msg.car : { front_m: 0.67, rear_m: 0.08, half_width_m: 0.335, wheelbase_m: 0.52 };
  const line = (msg && msg.tel.line) || { stale: true };
  const registered = live && !line.stale && !!line.origin;
  const fixed = live && confirmedPose(msg) !== null;
  // 只保留最后确认的固定解，浮点、单点、失联数据不会改变车身或镜头位置。
  const vehicle = state.fixedPose ? { ...state.fixedPose, usable: fixed, speed: fixed ? state.fixedPose.speed : null } : null;
  const key = state.version + '/' + live;          // 新数据来了或连接状态变了才重建场景
  if (cachedScene && sceneKey === key) return cachedScene;
  const file = live ? state.view : null;
  if (file) {
    // 平板在预览导入的图纸（还没应用到车）：与 app 一样只画图纸，不画车的实时位置和轨迹
    const controlPoints = file.control_points || [];
    const bounds = boundsOf(file.points.concat(controlPoints));
    cachedScene = { route: file.points, reference: [], trail: [], vehicle: null, antennaPoint: null, origin: 'file:' + file.name,
      controlPoints,
      body: bodyOf(car), wheelbase: car.wheelbase_m, steerDeg: 0, roll: 0, bounds, file,
      displayVehicle: { x: bounds.x, y: bounds.y, yawDeg: 0, preview: true, hidden: file.points.length > 0 } };
    sceneKey = key;
    return cachedScene;
  }
  const route = registered ? state.route : [], reference = registered && line.route && line.route.kind === 'path' ? state.plan : [];
  const bounds = boundsOf(route.concat(vehicle ? [[vehicle.x, vehicle.y]] : []));
  // 尚无固定解时默认静止；不使用漂动的原始天线坐标给展示模型定位置。
  const displayVehicle = vehicle ? { x: vehicle.x, y: vehicle.y, yawDeg: vehicle.yaw_deg, preview: false, hidden: false }
    : { x: bounds.x, y: bounds.y, yawDeg: 0, preview: true, hidden: route.length > 0 };
  cachedScene = { route, reference, trail: registered ? state.trail : [], vehicle, antennaPoint: null,
    origin: state.fixedOrigin || (line.origin ? JSON.stringify(line.origin) : null), positionHeld: !!vehicle && !fixed,
    body: bodyOf(car), wheelbase: car.wheelbase_m, steerDeg: state.steerDeg, roll: state.roll, bounds, displayVehicle };
  sceneKey = key;
  return cachedScene;
}

// ======================= 文字：与 app 一致 =======================
const FIX = { FIXED: '固定', FLOAT: '浮点', SINGLE: '单点' };
const fixLabel = (f) => FIX[f] || '无解';
const QUALITY = { RTK_FIXED: 'RTK 固定解', RTK_FLOAT: 'RTK 浮点解', DEAD_RECKONING: '航位推算' };
const REASONS = {
  too_far_from_line: '离路线太远(>0.5 m)', heading_not_aligned: '车头没对准路线方向(>30°)', too_far_behind_a: '在路线起点后方太远(>3 m)',
  already_past_b: '已经过了路线终点', no_line: '还没有路线', pose_unusable: '定位不可用', no_pose: '还没有定位',
  off_line: '偏离路线太远(横偏>0.5 m 或车头偏>45°)', operator: '已手动停止', pose_recovered_waiting_operator: '定位已恢复',
  heading_uninitialized: '航向还没初始化(双天线航向未固定)', heading_unavailable: '双天线航向无效', course_heading: '按行驶轨迹推算航向',
  gyro_only: '只靠陀螺推算航向', rtk_fixed_timeout: 'RTK 固定解中断', fix_unavailable: '没有有效定位', position_outlier: '定位跳变已剔除',
  rtk_float: 'RTK 浮点解', position_uncertain: '位置误差偏大', heading_uncertain: '航向误差偏大', imu_stale: 'IMU 数据中断',
  wheel_stale: '轮速数据中断', position_reset: '定位刚重置,稍等', not_stopped_at_cusp: '换向点没停稳',
};
const REASON_ORDER = Object.keys(REASONS).sort((a, b) => b.length - a.length);
const humanize = (t) => { let o = t || ''; for (const k of REASON_ORDER) o = o.split(k).join(REASONS[k]); return o; };
const BLOCKING = ['heading_uninitialized', 'imu_stale', 'wheel_stale', 'rtk_fixed_timeout', 'position_reset', 'heading_uncertain', 'position_uncertain'];
function poseProblem(p) {
  if (!p || p.usable) return null;
  let b = BLOCKING.filter((r) => (p.reasons || []).includes(r));
  if (b.includes('heading_uninitialized')) b = b.filter((r) => r !== 'heading_uncertain' && r !== 'position_uncertain');
  return (b.length ? b : (p.reasons || [])).map(humanize).join('、') || '定位不可用';
}
function rtkLabel(r) {
  if (!r || r.stale) return ['RTK --', 'idle'];
  if (!r.receiver_ok) return ['RTK 无数据', 'danger'];
  if (r.fix === 'FIXED') return [`RTK 固定 ${r.sats ?? '-'}星`, 'ok'];
  if (r.fix === 'FLOAT') return ['RTK 浮点', 'warn'];
  if (r.fix === 'SINGLE') return ['RTK 单点', 'danger'];
  return ['RTK 无解', 'danger'];
}
const MODE = { IDLE: ['待命', 'idle'], MANUAL: ['手动驾驶', 'intent'], NAVIGATION: ['自动行驶中', 'intent'], FOLLOW: ['跟随模式', 'warn'],
  ESTOP: ['急停已锁存', 'danger'], FAULT: ['底盘故障', 'danger'] };

function tone(el, cls) {
  if (el.dataset.tone === cls) return;
  el.classList.remove('ok', 'warn', 'danger', 'idle', 'intent');
  if (cls) el.classList.add(cls);
  el.dataset.tone = cls;
}

// 状态栏一行：名称、值（按状态着色）、灰色小字补充
function row(dl, key, value, cls, small) {
  let dt = dl.querySelector(`dt[data-k="${key}"]`), dd;
  if (!dt) {
    dt = document.createElement('dt'); dt.dataset.k = key; dt.textContent = key;
    dd = document.createElement('dd'); dl.append(dt, dd);
  } else dd = dt.nextElementSibling;
  const sig = value + '\u0000' + (small || '');
  if (dd.dataset.sig !== sig) {
    dd.textContent = value;
    if (small) { const s = document.createElement('small'); s.textContent = small; dd.append(s); }
    dd.dataset.sig = sig;
  }
  tone(dd, cls);
}

function renderPanels(now) {
  const msg = state.msg, live = online(now);
  const tel = live ? msg.tel : null, rtk = tel ? tel.rtk : null, line = tel ? tel.line : null;
  if (msg) setText($('name'), msg.car.name || '小车');
  setText($('subtitle'), live ? '只读显示 · 车端数据实时' : '只读显示');
  // ---- 连接状态 ----
  const L = $('links');
  row(L, '车端服务', live ? '已连接' : '连接中断', live ? 'ok' : 'danger', live ? null : '等待车端网页服务');
  const apps = live ? msg.apps : null;
  row(L, '平板 app', apps == null ? '—' : apps > 0 ? `已连接 ${apps} 台` : '未连接', apps > 0 ? 'ok' : apps == null ? 'idle' : 'warn');
  const net = msg ? msg.net : null, addrs = net && net.addresses ? net.addresses.map((a) => a.ip) : [];
  row(L, '网络', net && net.wifi ? `WiFi ${net.wifi}` : addrs.length ? '有线 / 热点' : '没有网络', net && (net.wifi || addrs.length) ? 'ok' : 'warn',
    addrs.length ? addrs.slice(0, 2).join(' · ') : null);
  const n = rtk && !rtk.stale ? rtk.ntrip : null;
  row(L, 'CORS 差分', !n ? '—' : n.connected ? '已连接' : n.configured ? '未连接' : '未配置', !n ? 'idle' : n.connected ? 'ok' : 'warn',
    n && n.connected ? `改正数 ${Math.round((n.bytes || 0) / 1024)} KB` : n && n.configured && n.error ? String(n.error).slice(0, 28) : null);
  const link = tel ? tel.link : null, motion = tel ? tel.motion : null;
  const [modeText, modeTone] = motion ? (MODE[motion.mode] || [motion.mode, 'idle']) : ['—', 'idle'];
  row(L, '底盘', !link ? '—' : link.chassis_ok ? `在线 · ${modeText}` : '离线', !link ? 'idle' : !link.chassis_ok ? 'danger' : modeTone === 'idle' ? 'ok' : modeTone);
  row(L, 'RTK 程序', !tel ? '—' : rtk.stale ? '没有数据' : rtk.receiver_ok ? '运行中' : '接收机无数据', !tel ? 'idle' : rtk.stale || !rtk.receiver_ok ? 'danger' : 'ok');
  // ---- 卫星状态（AntennaStatusView） ----
  const [label, tone] = rtkLabel(rtk);
  setText($('rtk').firstElementChild, label, tone);
  setText($('rtk').lastElementChild, rtk && !rtk.stale ? `差分龄期 ${fmt(rtk.diff_age)} s · 水平精度 ${fin(rtk.sigma_h) ? (rtk.sigma_h * 100).toFixed(1) + ' cm' : '—'}` : live ? 'RTK 程序没有数据' : '连接中断');
  const r = rtk && !rtk.stale ? rtk : null;
  for (const name of ['ant1', 'ant2']) {
    const el = $(name), a = r && r.antennas ? r.antennas[name] : null, v = a && a.available ? a : null;
    setText(el.querySelector('.sats'), v && v.sats_used != null ? `解算 ${v.sats_used} / 跟踪 ${v.sats_tracked ?? '—'}` : '卫星 —');
    setText(el.querySelector('.fix'), !v ? '定位：—' : fin(v.offset_m) ? `定位：${fixLabel(v.fix)} · 距主天线 ${v.offset_m.toFixed(2)} m` : `定位：${fixLabel(v.fix)}`,
      v ? (v.fix === 'FIXED' ? 'ok' : 'warn') : 'idle');
    setText(el.querySelector('.fresh'), !live ? '连接中断' : !r ? '遥测已过期' : !a || a.age_s == null ? '等待天线数据' : !v ? '天线数据已过期' : `${a.source} · ${fmt(a.age_s)} s`);
  }
  const h = r ? r.heading : null;
  setText($('heading'), !h ? `双天线测向：${r ? '无数据' : '—'}` :
    `双天线测向：${fixLabel(h.fix)} · ${h.sats_used != null ? `${h.sats_used} / ${h.sats_tracked ?? '—'} 星` : '— 星'} · ${fin(h.length_m) ? `两天线 ${h.length_m.toFixed(2)} m` : '两天线 —'}` +
    (!r.heading_ok && fin(h.ref_m) ? `（两天线实时距离 ${h.ref_m.toFixed(2)} m）` : ''), r && r.heading_ok ? 'ok' : h ? 'warn' : 'idle');
  // ---- 定位与作业 ----
  const W = $('work'), pose = line && !line.stale ? line.pose : null;
  // 图纸：平板在预览导入的文件就写文件名；否则是车上存的路线（平板改完 0.6 s 自动存到车上）
  const file = live ? state.view : null, route = line && !line.stale ? line.route : null;
  const KIND = { path: (r) => `${r.closed ? '连线闭环' : '连线路线'} · ${r.points.length} 个点`, circle: () => '圆形绕行',
    points: (r) => `路点路线 · ${r.points.length} 个点` };
  if (file) row(W, '图纸', file.name || '导入的图纸', 'warn', '平板预览 · 未应用到车辆');
  else if (route && (route.points.length || route.center)) {
    const cover = route.cover;
    row(W, '图纸', (KIND[route.kind] || KIND.points)(route), '', [fin(route.length) ? `${route.length.toFixed(1)} m` : null,
      route.laps > 1 ? `${route.laps} 圈` : null, cover && cover.corners ? `${cover.corners} 处掉头补角` : null].filter(Boolean).join(' · ') || null);
  } else row(W, '图纸', live ? '车上没有路线' : '—', 'idle');
  const why = !live ? '连接中断' : !line || line.stale ? '车端 RTK 程序没有运行' : pose && pose.usable ? '定位可用' :
    !rtk || rtk.stale || !rtk.receiver_ok ? '等待 RTK 数据' : rtk.fix !== 'FIXED' ? '等待 RTK 固定解' :
      !pose && !rtk.heading_ok ? '等待双天线测向' : !pose ? '等待融合初始化' : poseProblem(pose) || '定位不可用于控制';
  row(W, '融合定位', pose && pose.usable ? (QUALITY[pose.quality] || '融合定位') : why, pose && pose.usable ? 'ok' : live ? 'warn' : 'idle',
    pose && pose.usable && fin(pose.sigma_yaw_deg) ? `航向 1σ ${pose.sigma_yaw_deg.toFixed(2)}°` : null);
  const drive = tel ? tel.drive : null, bat = tel ? tel.battery : null;
  row(W, '车速', drive && fin(drive.speed) ? `${drive.speed.toFixed(2)} m/s` : '—', drive ? '' : 'idle',
    bat && fin(bat.v) ? `电池 ${bat.v.toFixed(1)} V` : null);
  let job = '—', jobTone = 'idle', jobSmall = null;
  if (line && !line.stale) {
    const tr = line.track || {};
    if (line.phase === 'IDLE') { job = '待命'; jobSmall = line.message ? humanize(line.message).slice(0, 26) : null; }
    else if (line.phase === 'SELECTING') { job = '申请控制权'; jobTone = 'intent'; }
    else {
      job = line.task === 'route' ? '路线作业中' : '走直线中'; jobTone = 'intent';
      jobSmall = `已走 ${fmt(tr.s)} / ${fmt(tr.length)} m` + (tr.switching ? ' · 换向中' : tr.gear === 'reverse' ? ' · 掉头倒车' : '');
    }
  }
  row(W, '作业', job, jobTone, jobSmall);
}

// ---- 地图上的状态、比例尺、指北针、图例（SceneHud） ----
function renderHud(scene, now) {
  const live = online(now), line = state.msg && state.msg.tel.line;
  const registered = scene.origin !== null, v = scene.vehicle;
  const usable = live && registered && v && v.usable;
  const status = renderFailure ? renderFailure : !state.msg ? '等待车辆数据' : !live ? '连接中断 · 等待实时定位' :
    scene.file ? `文件预览 · 未应用到车辆 · ${scene.file.name || '导入的图纸'}` :
    !line || line.stale ? '车端 RTK 程序没有运行' : scene.positionHeld ? '定位未固定或不可用 · 保持最后固定位置' :
      !v ? (state.msg.tel.rtk && !state.msg.tel.rtk.stale && state.msg.tel.rtk.fix === 'FIXED' ? 'RTK 已固定 · 等待可靠位姿' : '等待 RTK 固定解 · 车辆保持静止') : !registered ? '等待车的 RTK 定位' :
      !v ? (scene.antennaPoint ? '航向待定 · 仅显示主天线' : '等待车辆位姿') : !v.usable ? (poseProblem(v) || '定位不可用') :
        `${QUALITY[v.quality] || '融合定位'} · 定位可用`;
  const chip = $('status');
  setText(chip.lastElementChild, status);
  tone(chip, usable ? 'ok' : 'warn');
  const proj = renderer ? renderer.projection : null;
  const compass = $('compass');
  compass.hidden = !(proj && v);
  if (!compass.hidden) compass.firstElementChild.style.transform = `rotate(${(proj.heading - Math.PI / 2) * 180 / Math.PI}deg)`;
  const scale = $('scale');
  scale.hidden = !(proj && proj.ppm > 0);
  if (!scale.hidden) {
    const preferred = 110 / proj.ppm, list = [0.2, 0.5, 1, 2, 5, 10, 20, 50, 100, 200, 500];
    let metres = 0.2; for (const m of list) if (m <= preferred) metres = m;
    setText(scale.children[0], `每小格 ${metresLabel(proj.gridStep)} m`);
    setText(scale.children[1], `比例尺 ${metresLabel(metres)} m`);
    scale.children[2].style.width = `${Math.round(metres * proj.ppm)}px`;
  }
  const items = [];
  if (scene.route.length) items.push([scene.file ? '导入的图纸' : '计划路线', '#38A8D6', false]);
  if (scene.reference.length) items.push(['车身中心要走的线', '#A8E8FA', true]);
  if (scene.route.length && v) items.push(['邻近路段', '#A8E8FA', false]);
  if (scene.trail.length) items.push(['已走轨迹', '#54E68C', false]);
  const legend = $('legend'), html = items.map(([t, c, d]) => `<span class="${d ? 'dashed' : ''}" style="--c:${c}">${t}</span>`).join('');
  legend.hidden = !items.length;
  if (legend.dataset.html !== html) { legend.innerHTML = html; legend.dataset.html = html; }
  const labels = $('labels'), lhtml = (proj ? proj.labels : []).map((l) => `<b style="left:${l.x}px;top:${l.y - 8}px">${l.text}</b>`).join('');
  if (labels.dataset.html !== lhtml) { labels.innerHTML = lhtml; labels.dataset.html = lhtml; }
}

// ======================= 主循环 =======================
let renderer = null, renderFailure = null, dirty = true, lastPanel = 0;
try { renderer = new Renderer($('scene')); } catch (e) { renderFailure = '3D 显示暂不可用'; console.error(e); }
if (renderer) {
  fetch('/display/vehicle.bin').then((r) => { if (!r.ok) throw new Error(r.status); return r.arrayBuffer(); })
    .then((buf) => { renderer.setVehicle(parseVehicle(buf)); dirty = true; })
    .catch((e) => console.warn('车辆模型加载失败，用通用示意车', e));
  $('scene').addEventListener('webglcontextlost', () => location.reload());
}
if (renderFailure) { $('banner').hidden = false; $('banner').textContent = renderFailure; }

// 固定视角只读屏：阻止浏览器的整页捏合、双击和滚轮缩放，右侧状态栏始终保持原大小。
// WebKitGTK 的原生捏合独立于 PointerEvent，必须拦截 TouchEvent 的默认行为。
const canvas = $('scene');
function preventPageGesture(e) { if (e.cancelable) e.preventDefault(); }
for (const type of ['touchstart', 'touchmove', 'gesturestart', 'gesturechange', 'gestureend', 'wheel']) {
  document.addEventListener(type, preventPageGesture, { passive: false });
}
document.addEventListener('keydown', (e) => {
  if ((e.ctrlKey || e.metaKey) && ['+', '-', '=', '0'].includes(e.key)) e.preventDefault();
});
window.addEventListener('resize', () => { dirty = true; });

function loop(now) {
  requestAnimationFrame(loop);
  // 跟屏幕刷新同步；定位更新时自动跟车，静止时不重画。
  const scene = buildScene(now);
  let painted = false;
  if (renderer && (dirty || scene !== renderer.active || renderer.moving)) {
    try { renderer.moving = renderer.render(scene, now); } catch (e) { renderFailure = '3D 显示暂不可用'; console.error(e); renderer = null; }
    dirty = false;
    painted = true;
  } else if (renderer) renderer.lastTime = now;
  const updatePanel = now - lastPanel > 200;
  if (painted || updatePanel) renderHud(scene, now);
  if (updatePanel) { renderPanels(now); lastPanel = now; }
}
requestAnimationFrame(loop);

setInterval(() => {
  const d = new Date();
  $('clock').textContent = [d.getHours(), d.getMinutes(), d.getSeconds()].map((v) => String(v).padStart(2, '0')).join(':');
}, 500);

// 事件流：浏览器断线会自己重连；只读，不发任何东西
const source = new EventSource('/display/stream');
source.onmessage = (e) => { try { onMessage(JSON.parse(e.data)); } catch (err) { console.error(err); } };
source.onerror = () => { dirty = true; };

// 「回到桌面」：只在车载窗口里显示（car_display_window.py 注册了 carDesktop 消息），电脑浏览器预览时没有这个按钮。
// 只是让车载窗口自己隐藏、显示 Ubuntu 桌面，不向车发任何指令。
(() => {
  const handlers = window.webkit && window.webkit.messageHandlers;
  const desktop = handlers && handlers.carDesktop;
  if (!desktop) return;
  const button = $('toDesktop');
  button.hidden = false;
  button.addEventListener('click', () => desktop.postMessage('desktop'));
})();
