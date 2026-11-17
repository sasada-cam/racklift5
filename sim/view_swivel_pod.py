"""Interactive Three.js Dual-Motor Rig Studio for the Complete Rail-Drive + 90-deg Swivel Pod.

Rigged actuators:
  1. 4632-370 Worm Gear Motor (100 RPM): drives the 14T rail climber pinion below the pod
  2. TD-8125MG 25KG Digital Servo (0 <-> 90 deg): rotates the leg-mount swivel hub OR rotates
     the entire rail-drive pod 90 deg relative to a fixed robot leg ("Frame" toggle)

Usage:
  python3 sim/view_swivel_pod.py
"""

import http.server
import os
import socketserver
import threading
import webbrowser

HERE = os.path.dirname(os.path.abspath(__file__))
MESH_DIR = os.path.join(HERE, "build", "meshes_swivel_pod")

HTML = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>Rail-Drive + 90° Swivel Pod — Dual-Motor Rig Studio</title>
<style>
  * { box-sizing: border-box; }
  body { margin: 0; overflow: hidden; font-family: system-ui, -apple-system, sans-serif; background: #12161f; color: #e6ebf2; }
  #canvas-wrap { position: fixed; inset: 0; }
  #panel {
    position: fixed; top: 14px; left: 14px; width: 320px; max-height: calc(100vh - 28px);
    overflow-y: auto; background: rgba(20, 25, 34, 0.92); backdrop-filter: blur(10px);
    border: 1px solid rgba(255,255,255,0.12); border-radius: 12px; padding: 14px;
    box-shadow: 0 12px 32px rgba(0,0,0,0.45); font-size: 12.5px;
  }
  h1 { font-size: 15px; margin: 0 0 2px; font-weight: 650; }
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
  <h1>Rail-Drive + 90° Swivel Pod</h1>
  <div class="sub">4632-370 (100 RPM) + TD-8125MG (0°↔90°) · 0.000 mm³ Clash</div>

  <div class="sec">
    <div class="sec-title">1. Climb Motor (4632-370 · 100 RPM)</div>
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
    <div class="sec-title">2. Swivel Servo (TD-8125MG · 0°↔90°)</div>
    <div class="btn-group" style="width:100%">
      <button id="s-0" style="flex:1" onclick="setSwivel(0)">0° Vert</button>
      <button id="s-90" style="flex:1" onclick="setSwivel(90)">90° Horiz</button>
      <button id="s-auto" class="active ok" style="flex:1" onclick="setSwivelAuto()">Auto 0↔90</button>
    </div>
    <div class="row" style="margin-top:6px">
      <span>Swivel Angle</span><span id="sw-val">0.0°</span>
    </div>
    <input id="sw-slider" type="range" min="0" max="90" step="1" value="0" oninput="setSwivel(Number(this.value))"/>
    <div class="row" style="margin-top:6px">
      <span>Reference Frame</span>
      <div class="btn-group">
        <button id="f-pod" class="active" onclick="setFrame('pod')">Fixed Pod</button>
        <button id="f-leg" onclick="setFrame('leg')">Fixed Leg</button>
      </div>
    </div>
  </div>

  <div class="sec">
    <div class="sec-title">3. Layer Visibility</div>
    <div class="row">
      <span><span class="dot" style="background:#33b873"></span>25T Swivel Leg Hub</span>
      <div class="btn-group">
        <button id="v-leg-solid" class="active" onclick="setVis('leg','solid')">Show</button>
        <button id="v-leg-off" onclick="setVis('leg','off')">Hide</button>
      </div>
    </div>
    <div class="row">
      <span><span class="dot" style="background:#c41e24"></span>TD-8125MG Servo</span>
      <div class="btn-group">
        <button id="v-servo-solid" class="active" onclick="setVis('servo','solid')">Show</button>
        <button id="v-servo-xray" onclick="setVis('servo','xray')">X-Ray</button>
        <button id="v-servo-off" onclick="setVis('servo','off')">Hide</button>
      </div>
    </div>
    <div class="row">
      <span><span class="dot" style="background:#2e8ee6"></span>Upper Cover + Tower</span>
      <div class="btn-group">
        <button id="v-cover-solid" onclick="setVis('cover','solid')">Solid</button>
        <button id="v-cover-xray" class="active" onclick="setVis('cover','xray')">Ghost</button>
        <button id="v-cover-off" onclick="setVis('cover','off')">Hide</button>
      </div>
    </div>
    <div class="row">
      <span><span class="dot" style="background:#b4bcc8"></span>4632-370 Motor</span>
      <div class="btn-group">
        <button id="v-motor-solid" class="active" onclick="setVis('motor','solid')">Show</button>
        <button id="v-motor-xray" onclick="setVis('motor','xray')">X-Ray</button>
        <button id="v-motor-off" onclick="setVis('motor','off')">Hide</button>
      </div>
    </div>
    <div class="row">
      <span><span class="dot" style="background:#475569"></span>Base Cradle</span>
      <div class="btn-group">
        <button id="v-cradle-solid" class="active" onclick="setVis('cradle','solid')">Show</button>
        <button id="v-cradle-off" onclick="setVis('cradle','off')">Hide</button>
      </div>
    </div>
    <div class="row">
      <span><span class="dot" style="background:#eb7a24"></span>14T Pinion + Retainer</span>
      <div class="btn-group">
        <button id="v-pinion-solid" class="active" onclick="setVis('pinion','solid')">Show</button>
        <button id="v-pinion-off" onclick="setVis('pinion','off')">Hide</button>
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

const camera = new THREE.PerspectiveCamera(40, window.innerWidth / window.innerHeight, 0.5, 1200);
camera.position.set(115, 95, 125);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
document.getElementById('canvas-wrap').appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.target.set(-5, 32, 0);
controls.update();

scene.add(new THREE.HemisphereLight(0xdbeafe, 0x1e293b, 1.25));
const key = new THREE.DirectionalLight(0xffffff, 1.6);
key.position.set(110, 160, 110);
key.castShadow = true;
scene.add(key);

const grid = new THREE.GridHelper(260, 26, 0x334155, 0x1e293b);
grid.position.y = -20;
scene.add(grid);

// World Z-up group
const worldRoot = new THREE.Group();
worldRoot.rotation.x = -Math.PI / 2;
scene.add(worldRoot);

// Pod pivot centered at the 25T swivel axis (10, 0, 0) so "Fixed Leg" rotates the entire pod!
const podFrame = new THREE.Group();
podFrame.position.set(10, 0, 0);
worldRoot.add(podFrame);

const podBody = new THREE.Group();
podBody.position.set(-10, 0, 0);
podFrame.add(podBody);

const climbPivot = new THREE.Group();
climbPivot.position.set(12.5, 0, 0);
podBody.add(climbPivot);

const swivelPivot = new THREE.Group();
swivelPivot.position.set(10.0, 0, 68.0);
podBody.add(swivelPivot);

const loader = new STLLoader();
const meshes = {};
const mats = {
  motor:   new THREE.MeshStandardMaterial({ color: 0xb4bcc8, metalness: 0.65, roughness: 0.35 }),
  shaft:   new THREE.MeshStandardMaterial({ color: 0xd8dee6, metalness: 0.85, roughness: 0.2 }),
  pinion:  new THREE.MeshStandardMaterial({ color: 0xeb7a24, roughness: 0.42 }),
  retainer:new THREE.MeshStandardMaterial({ color: 0x333942, roughness: 0.45 }),
  cradle:  new THREE.MeshStandardMaterial({ color: 0x3b4454, roughness: 0.55 }),
  cover:   new THREE.MeshStandardMaterial({ color: 0x2e8ee6, roughness: 0.35, transparent: true, opacity: 0.38, depthWrite: false }),
  servo:   new THREE.MeshStandardMaterial({ color: 0xc41e24, metalness: 0.5, roughness: 0.35 }),
  spline:  new THREE.MeshStandardMaterial({ color: 0xd4ad52, metalness: 0.85, roughness: 0.25 }),
  leg:     new THREE.MeshStandardMaterial({ color: 0x33b873, roughness: 0.40 }),
};

function addStl(key, file, parent, mat) {
  loader.load('/meshes/' + file, (geom) => {
    geom.computeVertexNormals();
    const m = new THREE.Mesh(geom, mat);
    m.castShadow = true;
    parent.add(m);
    meshes[key] = m;
  });
}

addStl('motor',   'motor_housing.stl', podBody, mats.motor);
addStl('cradle',  'base_cradle.stl',   podBody, mats.cradle);
addStl('cover',   'upper_cover.stl',   podBody, mats.cover);
addStl('servo',   'servo_body.stl',    podBody, mats.servo);
addStl('shaft',   'motor_shaft.stl',   climbPivot, mats.shaft);
addStl('pinion',  'pinion.stl',        climbPivot, mats.pinion);
addStl('retainer','retainer.stl',      climbPivot, mats.retainer);
addStl('spline',  'servo_spline.stl',  swivelPivot, mats.spline);
addStl('leg',     'swivel_leg_hub.stl',swivelPivot, mats.leg);

let rpm = 100;
let swAuto = true;
let swTarget = 0;
let swCur = 0;
let refFrame = 'pod';
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
window.setFrame = (f) => {
  refFrame = f;
  document.getElementById('f-pod').classList.toggle('active', f === 'pod');
  document.getElementById('f-leg').classList.toggle('active', f === 'leg');
};

const groups = {
  leg: ['leg', 'spline'],
  servo: ['servo'],
  cover: ['cover'],
  motor: ['motor', 'shaft'],
  cradle: ['cradle'],
  pinion: ['pinion', 'retainer'],
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
  if (meshes.retainer) meshes.retainer.position.z = -28 * k;
  if (meshes.pinion)   meshes.pinion.position.z   = -16 * k;
  if (meshes.cradle)   meshes.cradle.position.z   = -8 * k;
  if (meshes.cover)    meshes.cover.position.z    = 14 * k;
  if (meshes.servo)    meshes.servo.position.z    = 28 * k;
  if (meshes.leg)      meshes.leg.position.z      = 38 * k;
};

window.resetCam = () => {
  camera.position.set(115, 95, 125);
  controls.target.set(-5, 32, 0);
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
  climbPivot.rotation.z = climbAng;

  if (swAuto) {
    const ph = t % 5.0;
    swTarget = ph < 2.5 ? 0 : 90;
  }
  swCur += (swTarget - swCur) * Math.min(1, dt * 7.0);
  const rad = swCur * Math.PI / 180;

  if (refFrame === 'pod') {
    podFrame.rotation.z = 0;
    swivelPivot.rotation.z = rad;
  } else {
    podFrame.rotation.z = -rad;
    swivelPivot.rotation.z = rad;
  }

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
    for port in range(8795, 8810):
        try:
            srv = http.server.ThreadingHTTPServer(("127.0.0.1", port), Handler)
            break
        except OSError:
            continue
    else:
        raise RuntimeError("No free port in 8795..8809")

    url = f"http://127.0.0.1:{port}/"
    print(f"Swivel Pod Dual-Motor Studio running at {url}  (Ctrl+C to stop)")
    threading.Timer(0.4, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        srv.shutdown()


if __name__ == "__main__":
    main()
