"""Interactive Three.js 3D Studio for the Accurate T-Slot + MT0134 + Sideways 4632-370 + TD-8125MG Carriage.

Features:
  - Stationary Robot Body + TD-8125MG Servo Housing (no wire twist!)
  - Rotating 90 deg Carriage: exact MT0134 65.5x65.5x3 mm plate, 4x Delrin V-Wheels (Ø24x10.95)
    with 6 mm spacers + 1 mm shims, sideways 4632-370 cuboid pod, 25T Aluminum Horn
  - Spinning 14T Climber Pinion (100 RPM) seated inside the CNC4551 2020 T-Slot Rail + In-Slot Rack
  - Per-part Show / Ghost / X-Ray / Hide toggles & Exploded View

Usage:
  python3 sim/view_t_slot_swivel_carriage.py
"""

import http.server
import os
import socketserver
import threading
import webbrowser

HERE = os.path.dirname(os.path.abspath(__file__))
MESH_DIR = os.path.join(HERE, "build", "meshes_t_slot_carriage")

HTML = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>T-Slot 2020 + MT0134 + Sideways 4632-370 + TD-8125MG Studio</title>
<style>
  * { box-sizing: border-box; }
  body { margin: 0; overflow: hidden; font-family: system-ui, -apple-system, sans-serif; background: #12161f; color: #e6ebf2; }
  #canvas-wrap { position: fixed; inset: 0; }
  #panel {
    position: fixed; top: 14px; left: 14px; width: 330px; max-height: calc(100vh - 28px);
    overflow-y: auto; background: rgba(20, 25, 34, 0.92); backdrop-filter: blur(10px);
    border: 1px solid rgba(255,255,255,0.12); border-radius: 12px; padding: 14px;
    box-shadow: 0 12px 32px rgba(0,0,0,0.45); font-size: 12.5px;
  }
  h1 { font-size: 14.5px; margin: 0 0 2px; font-weight: 650; }
  .sub { font-size: 11px; color: #94a3b8; margin-bottom: 10px; }
  .sec { margin-top: 10px; padding-top: 9px; border-top: 1px solid rgba(255,255,255,0.10); }
  .sec-title { font-size: 10.5px; text-transform: uppercase; letter-spacing: 0.06em; color: #94a3b8; margin-bottom: 7px; font-weight: 600; }
  .row { display: flex; align-items: center; justify-content: space-between; margin: 5px 0; gap: 6px; }
  .btn-group { display: flex; gap: 4px; }
  button {
    background: #232b3a; color: #dce4f0; border: 1px solid rgba(255,255,255,0.14);
    border-radius: 6px; padding: 4px 7px; font-size: 11px; cursor: pointer; transition: 0.12s;
  }
  button:hover { background: #2f3a4f; }
  button.active { background: #2563eb; border-color: #3b82f6; color: #fff; }
  button.ok.active { background: #16a34a; border-color: #22c55e; }
  input[type=range] { width: 100%; accent-color: #3b82f6; cursor: pointer; }
  .dot { display: inline-block; width: 8px; height: 8px; border-radius: 50%; margin-right: 5px; }
</style>
<script type="importmap">
{
  "imports": {
    "three": "https://unpkg.com/three@0.160.0/build/three.module.js",
    "three/addons/": "https://unpkg.com/three@0.160.0/examples/jsm/"
  }
}
</script>
</head>
<body>
<div id="canvas-wrap"></div>

<div id="panel">
  <h1>T-Slot 2020 Swivel Carriage</h1>
  <div class="sub">MT0134 + Delrin V-Wheels + Sideways 4632-370 · 0.000 mm³ Clash</div>

  <div class="sec">
    <div class="sec-title">1. Leg Carriage Swivel (TD-8125MG · 0°↔90°)</div>
    <div class="btn-group" style="width:100%">
      <button id="s-0" style="flex:1" onclick="setSwivel(0)">0° Rail X</button>
      <button id="s-90" style="flex:1" onclick="setSwivel(90)">90° Rail Y</button>
      <button id="s-auto" class="active ok" style="flex:1" onclick="setSwivelAuto()">Auto 0↔90</button>
    </div>
    <div class="row" style="margin-top:6px">
      <span>Carriage Angle</span><span id="sw-val">0.0°</span>
    </div>
    <input id="sw-slider" type="range" min="0" max="90" step="1" value="0" oninput="setSwivel(Number(this.value))"/>
  </div>

  <div class="sec">
    <div class="sec-title">2. Climb Pinion (4632-370 · 100 RPM)</div>
    <div class="btn-group" style="width:100%">
      <button id="m-rev" style="flex:1" onclick="setMotor('rev')">◀ -100</button>
      <button id="m-stop" style="flex:1" onclick="setMotor('stop')">■ Stop</button>
      <button id="m-fwd" class="active ok" style="flex:1" onclick="setMotor('fwd')">+100 ▶</button>
    </div>
    <div class="row" style="margin-top:6px">
      <span>Pinion Speed</span><span id="rpm-val">100 RPM (110 mm/s)</span>
    </div>
    <input id="rpm-slider" type="range" min="-100" max="100" step="5" value="100" oninput="setRpm(this.value)"/>
  </div>

  <div class="sec">
    <div class="sec-title">3. Layer Visibility</div>
    <div class="row">
      <span><span class="dot" style="background:#33b873"></span>Body Clamp (Fixed)</span>
      <div class="btn-group">
        <button id="v-body-solid" class="active" onclick="setVis('body','solid')">Solid</button>
        <button id="v-body-xray" onclick="setVis('body','xray')">Ghost</button>
        <button id="v-body-off" onclick="setVis('body','off')">Hide</button>
      </div>
    </div>
    <div class="row">
      <span><span class="dot" style="background:#c41e24"></span>TD-8125MG + 25T Horn</span>
      <div class="btn-group">
        <button id="v-servo-solid" class="active" onclick="setVis('servo','solid')">Show</button>
        <button id="v-servo-xray" onclick="setVis('servo','xray')">X-Ray</button>
        <button id="v-servo-off" onclick="setVis('servo','off')">Hide</button>
      </div>
    </div>
    <div class="row">
      <span><span class="dot" style="background:#2e8ee6"></span>Sideways Pod + Turntable</span>
      <div class="btn-group">
        <button id="v-pod-solid" onclick="setVis('pod','solid')">Solid</button>
        <button id="v-pod-xray" class="active" onclick="setVis('pod','xray')">Ghost</button>
        <button id="v-pod-off" onclick="setVis('pod','off')">Hide</button>
      </div>
    </div>
    <div class="row">
      <span><span class="dot" style="background:#b4bcc8"></span>4632-370 Motor</span>
      <div class="btn-group">
        <button id="v-motor-solid" class="active" onclick="setVis('motor','solid')">Show</button>
        <button id="v-motor-off" onclick="setVis('motor','off')">Hide</button>
      </div>
    </div>
    <div class="row">
      <span><span class="dot" style="background:#eb7a24"></span>14T Pinion + Retainer</span>
      <div class="btn-group">
        <button id="v-pinion-solid" class="active" onclick="setVis('pinion','solid')">Show</button>
        <button id="v-pinion-off" onclick="setVis('pinion','off')">Hide</button>
      </div>
    </div>
    <div class="row">
      <span><span class="dot" style="background:#3a3d45"></span>MT0134 + 4× V-Wheels</span>
      <div class="btn-group">
        <button id="v-gantry-solid" class="active" onclick="setVis('gantry','solid')">Show</button>
        <button id="v-gantry-off" onclick="setVis('gantry','off')">Hide</button>
      </div>
    </div>
    <div class="row">
      <span><span class="dot" style="background:#94a3b8"></span>2020 T-Slot Rail + Rack</span>
      <div class="btn-group">
        <button id="v-rail-solid" class="active" onclick="setVis('rail','solid')">Show</button>
        <button id="v-rail-xray" onclick="setVis('rail','xray')">Ghost</button>
        <button id="v-rail-off" onclick="setVis('rail','off')">Hide</button>
      </div>
    </div>
  </div>

  <div class="sec">
    <div class="row"><span>Exploded View</span><span id="exp-val">0%</span></div>
    <input id="exp-slider" type="range" min="0" max="100" value="0" oninput="setExplode(this.value)"/>
    <div class="row" style="margin-top:6px"><button style="width:100%" onclick="resetCam()">Reset Camera</button></div>
  </div>
</div>

<script type="module">
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x12161f);

const camera = new THREE.PerspectiveCamera(40, window.innerWidth / window.innerHeight, 0.5, 1400);
camera.position.set(135, 95, 140);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
document.getElementById('canvas-wrap').appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.target.set(18, 18, 0);
controls.update();

scene.add(new THREE.HemisphereLight(0xdbeafe, 0x1e293b, 1.3));
const key = new THREE.DirectionalLight(0xffffff, 1.6);
key.position.set(120, 180, 110);
key.castShadow = true;
scene.add(key);

const grid = new THREE.GridHelper(360, 36, 0x334155, 0x1e293b);
grid.position.y = -25;
scene.add(grid);

const worldRoot = new THREE.Group();
worldRoot.rotation.x = -Math.PI / 2;
scene.add(worldRoot);

// Fixed Body + Servo + T-Slot Rail
const fixedGroup = new THREE.Group();
worldRoot.add(fixedGroup);

// Rotating Carriage (swivels 0 <-> 90 deg around Z)
const carriagePivot = new THREE.Group();
worldRoot.add(carriagePivot);

// Spinning Pinion (inside carriagePivot at X=52.5, Y=0, Z=5.325, rotates around Y)
const pinionPivot = new THREE.Group();
pinionPivot.position.set(52.5, 0, 5.325);
carriagePivot.add(pinionPivot);

const loader = new STLLoader();
const meshes = {};
const mats = {
  body:         new THREE.MeshStandardMaterial({ color: 0x33b873, roughness: 0.42 }),
  servo_body:   new THREE.MeshStandardMaterial({ color: 0xc41e24, metalness: 0.5, roughness: 0.35 }),
  servo_spline: new THREE.MeshStandardMaterial({ color: 0xd4ad52, metalness: 0.85, roughness: 0.25 }),
  alum_horn:    new THREE.MeshStandardMaterial({ color: 0xe04040, metalness: 0.75, roughness: 0.30 }),
  carriage_pod: new THREE.MeshStandardMaterial({ color: 0x2e8ee6, roughness: 0.35, transparent: true, opacity: 0.38, depthWrite: false }),
  motor_housing:new THREE.MeshStandardMaterial({ color: 0xb4bcc8, metalness: 0.65, roughness: 0.35 }),
  motor_shaft:  new THREE.MeshStandardMaterial({ color: 0xd8dee6, metalness: 0.85, roughness: 0.2 }),
  pinion:       new THREE.MeshStandardMaterial({ color: 0xeb7a24, roughness: 0.42 }),
  retainer:     new THREE.MeshStandardMaterial({ color: 0x333942, roughness: 0.45 }),
  gantry_plate: new THREE.MeshStandardMaterial({ color: 0x22262d, metalness: 0.5, roughness: 0.45 }),
  v_wheels:     new THREE.MeshStandardMaterial({ color: 0x3c414a, roughness: 0.5 }),
  t_rail:       new THREE.MeshStandardMaterial({ color: 0xcbd5e1, metalness: 0.7, roughness: 0.3 }),
  t_rack:       new THREE.MeshStandardMaterial({ color: 0xf59e0b, roughness: 0.4 }),
};

function addStl(key, parent) {
  loader.load('/meshes/' + key + '.stl', (geom) => {
    geom.computeVertexNormals();
    const m = new THREE.Mesh(geom, mats[key]);
    m.castShadow = true;
    parent.add(m);
    meshes[key] = m;
  });
}

addStl('body_mount',   fixedGroup);
addStl('servo_body',   fixedGroup);
addStl('t_rail',       fixedGroup);
addStl('t_rack',       fixedGroup);
addStl('servo_spline', carriagePivot);
addStl('alum_horn',    carriagePivot);
addStl('carriage_pod', carriagePivot);
addStl('motor_housing',carriagePivot);
addStl('gantry_plate', carriagePivot);
addStl('v_wheels',     carriagePivot);
addStl('motor_shaft',  pinionPivot);
addStl('pinion',       pinionPivot);
addStl('retainer',     pinionPivot);

let rpm = 100;
let swAuto = true;
let swTarget = 0;
let swCur = 0;
let climbAng = 0;

window.setMotor = (m) => {
  rpm = m === 'rev' ? -100 : (m === 'stop' ? 0 : 100);
  document.getElementById('rpm-slider').value = rpm;
  ['rev','stop','fwd'].forEach(id => document.getElementById('m-' + id).classList.toggle('active', id === m));
};
window.setRpm = (v) => {
  rpm = Number(v);
  ['rev','stop','fwd'].forEach(id => document.getElementById('m-' + id).classList.remove('active'));
};
window.setSwivel = (deg) => {
  swAuto = false;
  swTarget = deg;
  document.getElementById('sw-slider').value = deg;
  ['s-0','s-90','s-auto'].forEach(id => document.getElementById(id).classList.remove('active'));
  if (deg === 0 || deg === 90) document.getElementById('s-' + deg).classList.add('active');
};
window.setSwivelAuto = () => {
  swAuto = true;
  ['s-0','s-90','s-auto'].forEach(id => document.getElementById(id).classList.toggle('active', id === 's-auto'));
};

const groups = {
  body:   ['body_mount'],
  servo:  ['servo_body', 'servo_spline', 'alum_horn'],
  pod:    ['carriage_pod'],
  motor:  ['motor_housing', 'motor_shaft'],
  pinion: ['pinion', 'retainer'],
  gantry: ['gantry_plate', 'v_wheels'],
  rail:   ['t_rail', 't_rack'],
};
window.setVis = (name, state) => {
  (groups[name] || []).forEach(k => {
    const m = meshes[k];
    if (!m) return;
    if (state === 'off') m.visible = false;
    else if (state === 'xray') { m.visible = true; m.material.transparent = true; m.material.opacity = 0.35; m.material.depthWrite = false; }
    else { m.visible = true; m.material.transparent = false; m.material.opacity = 1.0; m.material.depthWrite = true; }
  });
  ['solid','xray','off'].forEach(s => {
    const el = document.getElementById(`v-${name}-${s}`);
    if (el) el.classList.toggle('active', s === state);
  });
};

window.setExplode = (pct) => {
  document.getElementById('exp-val').textContent = pct + '%';
  const k = Number(pct) / 100;
  if (meshes.t_rail)       meshes.t_rail.position.z       = -22 * k;
  if (meshes.t_rack)       meshes.t_rack.position.z       = -14 * k;
  if (meshes.v_wheels)     meshes.v_wheels.position.z     = -8 * k;
  if (meshes.carriage_pod) meshes.carriage_pod.position.z = 12 * k;
  if (meshes.alum_horn)    meshes.alum_horn.position.z    = 20 * k;
  if (meshes.servo_body)   meshes.servo_body.position.z   = 28 * k;
  if (meshes.body_mount)   meshes.body_mount.position.z   = 40 * k;
};

window.resetCam = () => {
  camera.position.set(135, 95, 140);
  controls.target.set(18, 18, 0);
  controls.update();
};

window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});

const clock = new THREE.Clock();
let t = 0;

function animate() {
  requestAnimationFrame(animate);
  const dt = Math.min(clock.getDelta(), 0.05);
  t += dt;

  const omega = rpm * 2 * Math.PI / 60;
  climbAng += omega * dt;
  pinionPivot.rotation.y = -climbAng;

  if (swAuto) {
    const ph = t % 5.0;
    swTarget = ph < 2.5 ? 0 : 90;
  }
  swCur += (swTarget - swCur) * Math.min(1, dt * 7.0);
  carriagePivot.rotation.z = swCur * Math.PI / 180;

  document.getElementById('rpm-val').textContent = `${rpm} RPM (${(omega * 10.5).toFixed(0)} mm/s)`;
  document.getElementById('sw-val').textContent = `${swCur.toFixed(1)}°`;

  controls.update();
  renderer.render(scene, camera);
}
animate();
</script>
</body>
</html>
"""


class Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/?"):
            body = HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path.startswith("/meshes/"):
            fname = os.path.basename(self.path.split("?", 1)[0])
            fpath = os.path.join(MESH_DIR, fname)
            if os.path.isfile(fpath):
                with open(fpath, "rb") as f:
                    data = f.read()
                self.send_response(200)
                self.send_header("Content-Type", "application/octet-stream")
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(data)
                return

        self.send_error(404)


def main():
    socketserver.TCPServer.allow_reuse_address = True
    for port in range(8825, 8840):
        try:
            srv = http.server.ThreadingHTTPServer(("127.0.0.1", port), Handler)
            break
        except OSError:
            continue
    else:
        raise RuntimeError("No free port in 8825..8839")

    url = f"http://127.0.0.1:{port}/"
    print(f"T-Slot Swivel Carriage Studio running at {url}  (Ctrl+C to stop)")
    threading.Timer(0.4, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        srv.shutdown()


if __name__ == "__main__":
    main()
