"""Interactive Three.js 3D Studio & Rigging Viewer for the 4632-370 Worm Gear Motor Kit.

Features custom controls in the same window:
  - Toggle / X-Ray / Hide individual parts: Motor+Gearbox, Top Cover, Bottom Cradle, 14T Pinion, Shaft+Retainer
  - Exploded view slider (0% - 100%)
  - Live rigged 100 RPM drive: Forward / Stop / Reverse / Auto-Cycle + RPM slider + live angle & linear rail speed

Usage:
  python3 sim/view_4632_370.py
"""

import http.server
import os
import socketserver
import threading
import webbrowser

HERE = os.path.dirname(os.path.abspath(__file__))
MESH_DIR = os.path.join(HERE, "build", "meshes_4632_370")

HTML = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>4632-370 Worm Gear Motor — 3D Rig Studio</title>
<style>
  * { box-sizing: border-box; }
  body { margin: 0; overflow: hidden; font-family: system-ui, -apple-system, sans-serif; background: #141820; color: #e6ebf2; }
  #canvas-wrap { position: fixed; inset: 0; }
  #panel {
    position: fixed; top: 14px; left: 14px; width: 300px; max-height: calc(100vh - 28px);
    overflow-y: auto; background: rgba(22, 27, 36, 0.90); backdrop-filter: blur(10px);
    border: 1px solid rgba(255,255,255,0.12); border-radius: 12px; padding: 14px;
    box-shadow: 0 12px 32px rgba(0,0,0,0.45); font-size: 13px;
  }
  h1 { font-size: 15px; margin: 0 0 2px; font-weight: 650; }
  .sub { font-size: 11px; color: #94a3b8; margin-bottom: 12px; }
  .sec { margin-top: 12px; padding-top: 10px; border-top: 1px solid rgba(255,255,255,0.10); }
  .sec-title { font-size: 11px; text-transform: uppercase; letter-spacing: 0.06em; color: #94a3b8; margin-bottom: 8px; font-weight: 600; }
  .row { display: flex; align-items: center; justify-content: space-between; margin: 6px 0; gap: 8px; }
  .btn-group { display: flex; gap: 4px; }
  button {
    background: #232b3a; color: #dce4f0; border: 1px solid rgba(255,255,255,0.14);
    border-radius: 6px; padding: 5px 8px; font-size: 11.5px; cursor: pointer; transition: 0.12s;
  }
  button:hover { background: #2f3a4f; }
  button.active { background: #2563eb; border-color: #3b82f6; color: #fff; }
  button.warn.active { background: #dc2626; border-color: #ef4444; }
  button.ok.active { background: #16a34a; border-color: #22c55e; }
  input[type=range] { width: 100%; accent-color: #3b82f6; cursor: pointer; }
  .stat-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 6px; margin-top: 8px; }
  .stat { background: rgba(255,255,255,0.05); border-radius: 7px; padding: 6px 8px; }
  .stat .k { font-size: 10px; color: #94a3b8; }
  .stat .v { font-size: 13px; font-weight: 600; font-variant-numeric: tabular-nums; margin-top: 1px; }
  .dot { display: inline-block; width: 8px; height: 8px; border-radius: 50%; margin-right: 6px; }
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
  <h1>4632-370 Worm Gear Motor</h1>
  <div class="sub">12V 100 RPM · 14T Rail Pinion &amp; PETG Mount</div>

  <div class="sec">
    <div class="sec-title">Motor Rig (100 RPM Max)</div>
    <div class="btn-group" style="width:100%">
      <button id="btn-rev" style="flex:1" onclick="setMode('rev')">◀ -100</button>
      <button id="btn-stop" style="flex:1" onclick="setMode('stop')">■ Stop</button>
      <button id="btn-fwd" style="flex:1" onclick="setMode('fwd')">+100 ▶</button>
      <button id="btn-auto" class="active ok" style="flex:1" onclick="setMode('auto')">Auto</button>
    </div>
    <div class="row" style="margin-top:8px">
      <span>Speed</span>
      <span id="rpm-val" style="font-variant-numeric:tabular-nums">100 RPM</span>
    </div>
    <input id="rpm-slider" type="range" min="-100" max="100" step="5" value="100" oninput="setManualRpm(this.value)"/>

    <div class="stat-grid">
      <div class="stat"><div class="k">Shaft Angle</div><div class="v" id="st-ang">0.0°</div></div>
      <div class="stat"><div class="k">Rail Speed (R=10.5)</div><div class="v" id="st-lin">0.0 mm/s</div></div>
    </div>
  </div>

  <div class="sec">
    <div class="sec-title">Parts Visibility</div>

    <div class="row">
      <span><span class="dot" style="background:#b0b8c4"></span>Motor &amp; Gearbox</span>
      <div class="btn-group">
        <button id="p-housing-solid" class="active" onclick="setPart('housing','solid')">Show</button>
        <button id="p-housing-xray" onclick="setPart('housing','xray')">X-Ray</button>
        <button id="p-housing-off" onclick="setPart('housing','off')">Hide</button>
      </div>
    </div>

    <div class="row">
      <span><span class="dot" style="background:#3898e8"></span>Top PETG Cover</span>
      <div class="btn-group">
        <button id="p-cover-solid" onclick="setPart('cover','solid')">Solid</button>
        <button id="p-cover-xray" class="active" onclick="setPart('cover','xray')">Ghost</button>
        <button id="p-cover-off" onclick="setPart('cover','off')">Hide</button>
      </div>
    </div>

    <div class="row">
      <span><span class="dot" style="background:#475569"></span>Bottom Cradle</span>
      <div class="btn-group">
        <button id="p-cradle-solid" class="active" onclick="setPart('cradle','solid')">Show</button>
        <button id="p-cradle-off" onclick="setPart('cradle','off')">Hide</button>
      </div>
    </div>

    <div class="row">
      <span><span class="dot" style="background:#eb7a24"></span>14T Pinion Head</span>
      <div class="btn-group">
        <button id="p-pinion-solid" class="active" onclick="setPart('pinion','solid')">Show</button>
        <button id="p-pinion-off" onclick="setPart('pinion','off')">Hide</button>
      </div>
    </div>

    <div class="row">
      <span><span class="dot" style="background:#cbd5e1"></span>Shaft &amp; Retainer</span>
      <div class="btn-group">
        <button id="p-shaft-solid" class="active" onclick="setPart('shaft','solid')">Show</button>
        <button id="p-shaft-off" onclick="setPart('shaft','off')">Hide</button>
      </div>
    </div>
  </div>

  <div class="sec">
    <div class="sec-title">Assembly View</div>
    <div class="row">
      <span>Exploded View</span>
      <span id="exp-val">0%</span>
    </div>
    <input id="exp-slider" type="range" min="0" max="100" value="0" oninput="setExplode(this.value)"/>
    <div class="row" style="margin-top:8px">
      <button style="width:100%" onclick="resetCam()">Reset Camera</button>
    </div>
  </div>
</div>

<script type="module">
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';

const wrap = document.getElementById('canvas-wrap');
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x141820);

const camera = new THREE.PerspectiveCamera(40, window.innerWidth / window.innerHeight, 0.5, 1000);
camera.position.set(85, 68, 95);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
wrap.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.target.set(-10, 16, 0);
controls.update();

// Studio lighting
scene.add(new THREE.HemisphereLight(0xdbeafe, 0x1e293b, 1.1));
const key = new THREE.DirectionalLight(0xffffff, 1.6);
key.position.set(90, 140, 100);
key.castShadow = true;
key.shadow.mapSize.set(1024, 1024);
scene.add(key);
const rim = new THREE.DirectionalLight(0x93c5fd, 0.7);
rim.position.set(-100, 60, -80);
scene.add(rim);

// Ground grid (mm)
const grid = new THREE.GridHelper(220, 22, 0x334155, 0x1e293b);
grid.position.y = -3.0;
scene.add(grid);

// Root CAD group (Z-up -> Y-up)
const root = new THREE.Group();
root.rotation.x = -Math.PI / 2;
scene.add(root);

const shaftPivot = new THREE.Group();
shaftPivot.position.set(12.5, 0, 22.5);
root.add(shaftPivot);

const loader = new STLLoader();
const meshes = {};
const mats = {
  housing: new THREE.MeshStandardMaterial({ color: 0xb4bcc8, metalness: 0.65, roughness: 0.35 }),
  cover:   new THREE.MeshStandardMaterial({ color: 0x2e8ee6, roughness: 0.35, transparent: true, opacity: 0.38, depthWrite: false }),
  cradle:  new THREE.MeshStandardMaterial({ color: 0x3b4454, roughness: 0.55 }),
  shaft:   new THREE.MeshStandardMaterial({ color: 0xd8dee6, metalness: 0.85, roughness: 0.2 }),
  pinion:  new THREE.MeshStandardMaterial({ color: 0xeb7a24, roughness: 0.42 }),
  retainer:new THREE.MeshStandardMaterial({ color: 0x333942, roughness: 0.45 }),
};

const baseZ = { housing: 0, cover: 0, cradle: 0, shaft: 0, pinion: 5.0, retainer: 16.0 };

function addStl(key, file, parent, mat) {
  loader.load('/meshes/' + file, (geom) => {
    geom.computeVertexNormals();
    const m = new THREE.Mesh(geom, mat);
    m.castShadow = true;
    m.receiveShadow = true;
    m.position.z = baseZ[key];
    parent.add(m);
    meshes[key] = m;
  });
}

addStl('housing',  'housing.stl',  root, mats.housing);
addStl('cover',    'cover.stl',    root, mats.cover);
addStl('cradle',   'cradle.stl',   root, mats.cradle);
addStl('shaft',    'shaft.stl',    shaftPivot, mats.shaft);
addStl('pinion',   'pinion.stl',   shaftPivot, mats.pinion);
addStl('retainer', 'retainer.stl', shaftPivot, mats.retainer);

// Controls state
let mode = 'auto';
let manualRpm = 100;
let shaftAngle = 0;

window.setMode = (m) => {
  mode = m;
  ['rev','stop','fwd','auto'].forEach(id => {
    document.getElementById('btn-' + id).classList.toggle('active', id === m);
  });
  if (m === 'rev') { manualRpm = -100; document.getElementById('rpm-slider').value = -100; }
  if (m === 'stop') { manualRpm = 0; document.getElementById('rpm-slider').value = 0; }
  if (m === 'fwd') { manualRpm = 100; document.getElementById('rpm-slider').value = 100; }
};

window.setManualRpm = (val) => {
  manualRpm = Number(val);
  mode = manualRpm === 0 ? 'stop' : (manualRpm > 0 ? 'fwd' : 'rev');
  ['rev','stop','fwd','auto'].forEach(id => document.getElementById('btn-' + id).classList.remove('active'));
};

window.setPart = (name, state) => {
  const targets = name === 'shaft' ? ['shaft', 'retainer'] : [name];
  targets.forEach(t => {
    const m = meshes[t];
    if (!m) return;
    if (state === 'off') {
      m.visible = false;
    } else if (state === 'xray') {
      m.visible = true;
      m.material.transparent = true;
      m.material.opacity = 0.35;
      m.material.depthWrite = false;
    } else {
      m.visible = true;
      m.material.transparent = false;
      m.material.opacity = 1.0;
      m.material.depthWrite = true;
    }
  });
  ['solid','xray','off'].forEach(s => {
    const el = document.getElementById(`p-${name}-${s}`);
    if (el) el.classList.toggle('active', s === state);
  });
};

window.setExplode = (pct) => {
  document.getElementById('exp-val').textContent = pct + '%';
  const k = Number(pct) / 100;
  if (meshes.cradle)   meshes.cradle.position.z   = baseZ.cradle - 16 * k;
  if (meshes.cover)    meshes.cover.position.z    = baseZ.cover + 24 * k;
  if (meshes.pinion)   meshes.pinion.position.z   = baseZ.pinion + 28 * k;
  if (meshes.retainer) meshes.retainer.position.z = baseZ.retainer + 42 * k;
};

window.resetCam = () => {
  camera.position.set(85, 68, 95);
  controls.target.set(-10, 16, 0);
  controls.update();
};

window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});

const clock = new THREE.Clock();
let simTime = 0;

function animate() {
  requestAnimationFrame(animate);
  const dt = Math.min(clock.getDelta(), 0.05);
  simTime += dt;

  let rpm = manualRpm;
  if (mode === 'auto') {
    const ph = simTime % 8.0;
    rpm = ph < 3.0 ? 100 : (ph < 4.0 ? 0 : (ph < 7.0 ? -100 : 0));
  }

  const omega = rpm * 2 * Math.PI / 60;
  shaftAngle += omega * dt;
  shaftPivot.rotation.z = shaftAngle;

  document.getElementById('rpm-val').textContent = `${rpm} RPM`;
  document.getElementById('st-ang').textContent = `${((shaftAngle * 180 / Math.PI) % 360).toFixed(1)}°`;
  document.getElementById('st-lin').textContent = `${(omega * 10.5).toFixed(1)} mm/s`;

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
    for port in range(8766, 8780):
        try:
            srv = http.server.ThreadingHTTPServer(("127.0.0.1", port), Handler)
            break
        except OSError:
            continue
    else:
        raise RuntimeError("No free port in 8766..8779")

    url = f"http://127.0.0.1:{port}/"
    print(f"4632-370 3D Rig Studio running at {url}  (Ctrl+C to stop)")
    threading.Timer(0.4, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        srv.shutdown()


if __name__ == "__main__":
    main()
