"""Interactive Three.js 3D Studio & 90-deg Rig Viewer for the TD-8125MG 25KG Digital Servo.

Usage:
  python3 sim/view_td8125mg.py
"""

import http.server
import os
import socketserver
import threading
import webbrowser

HERE = os.path.dirname(os.path.abspath(__file__))
MESH_DIR = os.path.join(HERE, "build", "meshes_td8125mg")

HTML = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>TD-8125MG 25KG Servo — 3D Rig Studio</title>
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
  button.ok.active { background: #16a34a; border-color: #22c55e; }
  input[type=range] { width: 100%; accent-color: #3b82f6; cursor: pointer; }
  .stat-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 6px; margin-top: 8px; }
  .stat { background: rgba(255,255,255,0.05); border-radius: 7px; padding: 6px 8px; }
  .stat .k { font-size: 10px; color: #94a3b8; }
  .stat .v { font-size: 13px; font-weight: 600; font-variant-numeric: tabular-nums; margin-top: 1px; }
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
  <h1>TD-8125MG 25KG Servo</h1>
  <div class="sub">180° Digital · PWM 500~2500µs · 25T Spline</div>

  <div class="sec">
    <div class="sec-title">90° Rail-Switch Rig</div>
    <div class="btn-group" style="width:100%">
      <button id="btn-0" style="flex:1" onclick="setTarget(0)">0° Vert</button>
      <button id="btn-90" style="flex:1" onclick="setTarget(90)">90° Horiz</button>
      <button id="btn-180" style="flex:1" onclick="setTarget(180)">180°</button>
      <button id="btn-auto" class="active ok" style="flex:1" onclick="setAuto()">Auto 0↔90</button>
    </div>
    <div class="row" style="margin-top:8px">
      <span>Angle</span>
      <span id="ang-val" style="font-variant-numeric:tabular-nums">0.0°</span>
    </div>
    <input id="ang-slider" type="range" min="0" max="180" step="1" value="0" oninput="setTarget(Number(this.value))"/>

    <div class="stat-grid">
      <div class="stat"><div class="k">PWM Pulse</div><div class="v" id="st-pwm">500 µs</div></div>
      <div class="stat"><div class="k">Rail Mode</div><div class="v" id="st-mode">Vertical</div></div>
    </div>
  </div>

  <div class="sec">
    <div class="sec-title">Parts Visibility</div>
    <div class="row">
      <span>Servo Housing</span>
      <div class="btn-group">
        <button id="p-case-solid" class="active" onclick="setCase('solid')">Show</button>
        <button id="p-case-xray" onclick="setCase('xray')">X-Ray</button>
        <button id="p-case-off" onclick="setCase('off')">Hide</button>
      </div>
    </div>
    <div class="row">
      <span>25T Brass Spline</span>
      <div class="btn-group">
        <button id="p-spline-solid" class="active" onclick="setSpline(true)">Show</button>
        <button id="p-spline-off" onclick="setSpline(false)">Hide</button>
      </div>
    </div>
    <div class="row" style="margin-top:8px">
      <button style="width:100%" onclick="resetCam()">Reset Camera</button>
    </div>
  </div>
</div>

<script type="module">
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x141820);

const camera = new THREE.PerspectiveCamera(40, window.innerWidth / window.innerHeight, 0.5, 1000);
camera.position.set(65, 58, 72);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
document.getElementById('canvas-wrap').appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.target.set(0, 22, 0);
controls.update();

scene.add(new THREE.HemisphereLight(0xdbeafe, 0x1e293b, 1.2));
const key = new THREE.DirectionalLight(0xffffff, 1.6);
key.position.set(80, 120, 90);
key.castShadow = true;
scene.add(key);

const grid = new THREE.GridHelper(160, 16, 0x334155, 0x1e293b);
scene.add(grid);

const root = new THREE.Group();
root.rotation.x = -Math.PI / 2;
scene.add(root);

const splinePivot = new THREE.Group();
splinePivot.position.set(10.0, 0, 42.0);
root.add(splinePivot);

// Visual pointer bar on spline to make 0->90 deg rotation unmistakable
const ptr = new THREE.Mesh(
  new THREE.BoxGeometry(22, 3.5, 1.2),
  new THREE.MeshStandardMaterial({ color: 0x38bdf8, roughness: 0.4 })
);
ptr.position.set(8, 0, 5.1);
splinePivot.add(ptr);

const loader = new STLLoader();
const caseMeshes = [];
let splineMesh = null;

function addStl(file, parent, mat, isCase) {
  loader.load('/meshes/' + file, (geom) => {
    geom.computeVertexNormals();
    const m = new THREE.Mesh(geom, mat);
    m.castShadow = true;
    parent.add(m);
    if (isCase) caseMeshes.push(m);
    else splineMesh = m;
  });
}

addStl('case_bottom.stl', root, new THREE.MeshStandardMaterial({ color: 0x22252a, roughness: 0.55 }), true);
addStl('case_mid.stl',    root, new THREE.MeshStandardMaterial({ color: 0xc41e24, metalness: 0.65, roughness: 0.30 }), true);
addStl('case_top.stl',    root, new THREE.MeshStandardMaterial({ color: 0x22252a, roughness: 0.50 }), true);
addStl('spline.stl',      splinePivot, new THREE.MeshStandardMaterial({ color: 0xd4ad52, metalness: 0.85, roughness: 0.25 }), false);

let autoMode = true;
let targetDeg = 0;
let currentDeg = 0;

window.setTarget = (deg) => {
  autoMode = false;
  targetDeg = deg;
  document.getElementById('ang-slider').value = deg;
  ['btn-0','btn-90','btn-180','btn-auto'].forEach(id => document.getElementById(id).classList.remove('active'));
  if (deg === 0 || deg === 90 || deg === 180) document.getElementById('btn-' + deg).classList.add('active');
};

window.setAuto = () => {
  autoMode = true;
  ['btn-0','btn-90','btn-180','btn-auto'].forEach(id => document.getElementById(id).classList.toggle('active', id === 'btn-auto'));
};

window.setCase = (state) => {
  caseMeshes.forEach(m => {
    if (state === 'off') { m.visible = false; }
    else if (state === 'xray') { m.visible = true; m.material.transparent = true; m.material.opacity = 0.35; m.material.depthWrite = false; }
    else { m.visible = true; m.material.transparent = false; m.material.opacity = 1.0; m.material.depthWrite = true; }
  });
  ['solid','xray','off'].forEach(s => document.getElementById('p-case-' + s).classList.toggle('active', s === state));
};

window.setSpline = (show) => {
  if (splineMesh) splineMesh.visible = show;
  ptr.visible = show;
  document.getElementById('p-spline-solid').classList.toggle('active', show);
  document.getElementById('p-spline-off').classList.toggle('active', !show);
};

window.resetCam = () => {
  camera.position.set(65, 58, 72);
  controls.target.set(0, 22, 0);
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

  if (autoMode) {
    const ph = t % 4.0;
    targetDeg = ph < 2.0 ? 0 : 90;
  }
  currentDeg += (targetDeg - currentDeg) * Math.min(1, dt * 8.0);
  splinePivot.rotation.z = currentDeg * Math.PI / 180;

  document.getElementById('ang-val').textContent = currentDeg.toFixed(1) + '°';
  document.getElementById('st-pwm').textContent = Math.round(500 + (currentDeg / 180) * 2000) + ' µs';
  document.getElementById('st-mode').textContent = Math.abs(currentDeg - 90) < 5 ? 'Horizontal' : (currentDeg < 5 ? 'Vertical' : 'Turning');

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
    for port in range(8780, 8795):
        try:
            srv = http.server.ThreadingHTTPServer(("127.0.0.1", port), Handler)
            break
        except OSError:
            continue
    else:
        raise RuntimeError("No free port in 8780..8794")

    url = f"http://127.0.0.1:{port}/"
    print(f"TD-8125MG 3D Rig Studio running at {url}  (Ctrl+C to stop)")
    threading.Timer(0.4, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        srv.shutdown()


if __name__ == "__main__":
    main()
