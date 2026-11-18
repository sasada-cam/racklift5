"""Interactive Three.js 3D Studio for Option 1 Separate Body & Picker Assembly.

Displays the non-swivelling robot payload assembly (no legs), centered on the B01 column
attachment collar on the 40 x 25 x 2.0 mm hollow aluminium column stub:
  - B01 Split/Wrap Column Collar + P07-BODY anti-crush plug + 4x M4x35 EPDM-cushioned bolts
  - B02 Tray Boom + 5 kg TAL220 Bar Load Cell / HX711 + 12 mm Plywood Tray
  - Left & Right 2-Stage Telescopic Picker Arms + Servo 1/2 + 4x Flip Fingers
  - B03 Counterweight Battery & Controller PCB Enclosure
  - Reference Shelf 1 Bay + Daraz Basket

Usage:
  python3 sim/view_option1_body.py
"""

import http.server
import os
import socketserver
import threading
import webbrowser

HERE = os.path.dirname(os.path.abspath(__file__))
BODY_MESH_DIR = os.path.join(HERE, "build", "meshes_option1_body")

HTML = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>Option 1 — Body Attachment & Picker Studio</title>
<style>
  * { box-sizing: border-box; }
  body { margin: 0; overflow: hidden; font-family: system-ui, -apple-system, sans-serif; background: #10151f; color: #e6ebf2; }
  #canvas-wrap { position: fixed; inset: 0; }
  #panel {
    position: fixed; top: 14px; left: 14px; width: 358px; max-height: calc(100vh - 28px);
    overflow-y: auto; background: rgba(18, 24, 34, 0.92); backdrop-filter: blur(10px);
    border: 1px solid rgba(255,255,255,0.12); border-radius: 12px; padding: 14px;
    box-shadow: 0 12px 32px rgba(0,0,0,0.45); font-size: 12px;
  }
  h1 { font-size: 14.5px; margin: 0 0 2px; font-weight: 650; }
  .sub { font-size: 11px; color: #94a3b8; margin-bottom: 10px; }
  .sec { margin-top: 10px; padding-top: 9px; border-top: 1px solid rgba(255,255,255,0.10); }
  .sec-title { font-size: 10.5px; text-transform: uppercase; letter-spacing: 0.06em; color: #94a3b8; margin-bottom: 6px; font-weight: 600; }
  .row { display: flex; align-items: center; justify-content: space-between; margin: 5px 0; gap: 6px; }
  .btn-group { display: flex; gap: 4px; flex-wrap: wrap; }
  button {
    background: #232b3a; color: #dce4f0; border: 1px solid rgba(255,255,255,0.14);
    border-radius: 6px; padding: 4px 7px; font-size: 11px; cursor: pointer; transition: 0.12s;
  }
  button:hover { background: #2f3a4f; }
  button.active { background: #2563eb; border-color: #3b82f6; color: #fff; }
  button.ok.active { background: #16a34a; border-color: #22c55e; }
  input[type=range] { width: 100%; accent-color: #3b82f6; cursor: pointer; }
  .dot { display: inline-block; width: 8px; height: 8px; border-radius: 50%; margin-right: 5px; }
  .cut-table { width: 100%; border-collapse: collapse; font-size: 11px; color: #cbd5e1; margin-top: 4px; }
  .cut-table td { padding: 2px 0; border-bottom: 1px solid rgba(255,255,255,0.06); }
  .cut-table td:last-child { text-align: right; font-variant-numeric: tabular-nums; color: #93c5fd; font-weight: 600; }
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
  <h1>Option 1 · Separate Body & Attachment</h1>
  <div class="sub">B01 Column Collar · B02 Y-Gusset Boom · B03 Y-Cradle · B04 180×175 Printed Tray</div>

  <div class="sec">
    <div class="sec-title">1. Picker & Payload Motion</div>
    <div class="btn-group" style="width:100%">
      <button id="m-auto" class="active ok" style="flex:1" onclick="setMode('auto')">Auto Pick Cycle</button>
      <button id="m-stow" style="flex:1" onclick="setMode('stow')">Stowed</button>
      <button id="m-reach" style="flex:1" onclick="setMode('reach')">Reach Shelf</button>
    </div>
    <div class="row" style="margin-top:8px">
      <span>Arm Stroke Y (0 .. 205 mm)</span><span id="arm-val">0.0 mm</span>
    </div>
    <input id="arm-sl" type="range" min="0" max="205" step="1" value="0" oninput="setPose('arm', Number(this.value))"/>
    <div class="row">
      <span>Flip Fingers (0° Up .. 90° Grab)</span><span id="fng-val">0.0°</span>
    </div>
    <input id="fng-sl" type="range" min="0" max="90" step="1" value="0" oninput="setPose('fng', Number(this.value))"/>
    <div class="row">
      <span>Body Column Lift dZ (-50 .. 200 mm)</span><span id="bz-val">0.0 mm</span>
    </div>
    <input id="bz-sl" type="range" min="-50" max="200" step="2" value="0" oninput="setPose('bz', Number(this.value))"/>
    <div class="row">
      <span>Animation Speed (0× Pause .. 3×)</span><span id="spd-val">1.0×</span>
    </div>
    <input id="spd-sl" type="range" min="0" max="3" step="0.1" value="1" oninput="setSpeed(Number(this.value))"/>
  </div>

  <div class="sec">
    <div class="sec-title">2. Layers</div>
    <div class="row">
      <span><span class="dot" style="background:#38bdf8"></span>B01 Column Collar + Bolts</span>
      <div class="btn-group">
        <button id="v-col_m-solid" class="active" onclick="setVis('col_m','solid')">Solid</button>
        <button id="v-col_m-xray" onclick="setVis('col_m','xray')">Ghost</button>
        <button id="v-col_m-off" onclick="setVis('col_m','off')">Hide</button>
      </div>
    </div>
    <div class="row">
      <span><span class="dot" style="background:#3b82f6"></span>B04 Tray + B02 Base + B05 Wishbone</span>
      <div class="btn-group">
        <button id="v-tray-solid" class="active" onclick="setVis('tray','solid')">Solid</button>
        <button id="v-tray-xray" onclick="setVis('tray','xray')">Ghost</button>
        <button id="v-tray-off" onclick="setVis('tray','off')">Hide</button>
      </div>
    </div>
    <div class="row">
      <span><span class="dot" style="background:#f59e0b"></span>Telescopic Arms + Flip Fingers</span>
      <div class="btn-group">
        <button id="v-arms-solid" class="active" onclick="setVis('arms','solid')">Show</button>
        <button id="v-arms-off" onclick="setVis('arms','off')">Hide</button>
      </div>
    </div>
    <div class="row">
      <span><span class="dot" style="background:#22c55e"></span>B03 Battery + PCB Box</span>
      <div class="btn-group">
        <button id="v-bat-solid" class="active" onclick="setVis('bat','solid')">Solid</button>
        <button id="v-bat-xray" onclick="setVis('bat','xray')">Ghost</button>
        <button id="v-bat-off" onclick="setVis('bat','off')">Hide</button>
      </div>
    </div>
    <div class="row">
      <span><span class="dot" style="background:#cbd5e1"></span>Ref 40×25×2 Column Stub</span>
      <div class="btn-group">
        <button id="v-stub-solid" onclick="setVis('stub','solid')">Solid</button>
        <button id="v-stub-xray" class="active" onclick="setVis('stub','xray')">Ghost</button>
        <button id="v-stub-off" onclick="setVis('stub','off')">Hide</button>
      </div>
    </div>
    <div class="row">
      <span><span class="dot" style="background:#64748b"></span>Ref Daraz Basket + Shelf 1</span>
      <div class="btn-group">
        <button id="v-env-solid" onclick="setVis('env','solid')">Solid</button>
        <button id="v-env-xray" class="active" onclick="setVis('env','xray')">Ghost</button>
        <button id="v-env-off" onclick="setVis('env','off')">Hide</button>
      </div>
    </div>
  </div>

  <div class="sec">
    <div class="sec-title">3. Attachment & Prototype Spec</div>
    <table class="cut-table">
      <tr><td>B01 Clamp Zone (leg-free gap)</td><td>Z 138..226 mm</td></tr>
      <tr><td>B04 3D-Printed Ribbed Tray</td><td>180 × 175 mm PETG</td></tr>
      <tr><td>Prototype Basket / Rack Bay</td><td>140×130 / 184 mm</td></tr>
      <tr><td>B03 Compact Counterweight</td><td>78 × 46 × 66 mm</td></tr>
      <tr><td>Load Cell Mount (B02 Y-Pocket)</td><td>80 × 12.7 mm (5 kg)</td></tr>
      <tr><td>Telescopic Reach (2:1 stage 2)</td><td>up to 205 mm</td></tr>
    </table>
  </div>

  <div class="sec">
    <div class="btn-group" style="width:100%">
      <button style="flex:1" onclick="resetCam()">Reset View</button>
      <button style="flex:1" onclick="focusAttach()">Zoom Attachment (B01)</button>
    </div>
  </div>
</div>

<script type="module">
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';

const BXR = 180.0;
const XC_LEFT = BXR - 80.5;
const XC_RIGHT = BXR + 80.5;
const ZC_FS = 178.0 + 62.0; // TRAY_TOP_Z + FS_ZT = 240.0

const wrap = document.getElementById('canvas-wrap');
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x10151f);

const camera = new THREE.PerspectiveCamera(42, window.innerWidth / window.innerHeight, 1, 5000);
camera.position.set(-220, 420, 480);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(window.innerWidth, window.innerHeight);
wrap.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.target.set(80, 190, 140);
controls.update();

scene.add(new THREE.HemisphereLight(0xdbeafe, 0x1e293b, 1.35));
const key = new THREE.DirectionalLight(0xffffff, 1.5);
key.position.set(-300, 650, 450);
scene.add(key);

const grid = new THREE.GridHelper(1200, 24, 0x334155, 0x1e293b);
grid.position.set(80, 0, 100);
scene.add(grid);

const worldRoot = new THREE.Group();
worldRoot.rotation.x = -Math.PI / 2;
scene.add(worldRoot);

const loader = new STLLoader();
const m = (hex, extra={}) => new THREE.MeshStandardMaterial({ color: hex, roughness: 0.42, ...extra });

const mats = {
  column_stub:    m(0xcbd5e1, { metalness: 0.70, roughness: 0.30, transparent: true, opacity: 0.38, depthWrite: false }),
  anti_crush_plug:m(0xf29429),
  body_collar:    m(0x38bdf8),
  body_fasteners: m(0x94a3b8, { metalness: 0.85, roughness: 0.25 }),
  tray_boom:      m(0x3b82f6),
  live_wishbone:  m(0xef4444),
  load_cell:      m(0xe2e8f0, { metalness: 0.75, roughness: 0.30 }),
  tray_deck:      m(0x2e8ee6, { roughness: 0.45 }),
  tray_stops:     m(0x64748b),
  arms_outer:     m(0x94a3b8, { metalness: 0.65 }),
  arms_mid:       m(0x2563eb),
  arms_in:        m(0xf59e0b),
  fingers_left:   m(0xef4444),
  fingers_right:  m(0xef4444),
  picker_belts:   m(0x1e293b, { roughness: 0.8 }),
  picker_servos:  m(0xc41e24, { metalness: 0.45 }),
  battery_pod:    m(0x22c55e),
  battery_lid:    m(0x16a34a),
  battery_pack:   m(0x334155),
  ref_basket:     m(0x38bdf8, { transparent: true, opacity: 0.25, depthWrite: false }),
  ref_rack_bay:   m(0x64748b, { transparent: true, opacity: 0.28, depthWrite: false }),
};

function addMesh(name, parent, mat) {
  const holder = new THREE.Group();
  parent.add(holder);
  loader.load('/meshes_body/' + name + '.stl', (g) => {
    g.computeVertexNormals();
    holder.add(new THREE.Mesh(g, mat));
  });
  return holder;
}

const bodyLiftGroup = new THREE.Group();
worldRoot.add(bodyLiftGroup);

const stubGroup = new THREE.Group();
const envGroup = new THREE.Group();
worldRoot.add(stubGroup, envGroup);

addMesh('column_stub', stubGroup, mats.column_stub);
addMesh('anti_crush_plug', stubGroup, mats.anti_crush_plug);
addMesh('ref_rack_bay', envGroup, mats.ref_rack_bay);
const refBasketMesh = addMesh('ref_basket', bodyLiftGroup, mats.ref_basket);

const collarGroup = new THREE.Group();
const trayGroup = new THREE.Group();
const armsGroup = new THREE.Group();
const batGroup = new THREE.Group();
bodyLiftGroup.add(collarGroup, trayGroup, armsGroup, batGroup);

addMesh('body_collar', collarGroup, mats.body_collar);
addMesh('body_fasteners', collarGroup, mats.body_fasteners);
addMesh('tray_boom', trayGroup, mats.tray_boom);
addMesh('live_wishbone', trayGroup, mats.live_wishbone);
addMesh('load_cell', trayGroup, mats.load_cell);
addMesh('tray_deck', trayGroup, mats.tray_deck);
addMesh('tray_stops', trayGroup, mats.tray_stops);

addMesh('arms_outer', armsGroup, mats.arms_outer);
addMesh('picker_belts', armsGroup, mats.picker_belts);
addMesh('picker_servos', armsGroup, mats.picker_servos);
const armsMidMesh = addMesh('arms_mid', armsGroup, mats.arms_mid);
const armsInMesh = addMesh('arms_in', armsGroup, mats.arms_in);

const fingersLeftPivot = new THREE.Group();
fingersLeftPivot.position.set(XC_LEFT, 0, ZC_FS);
const fingersRightPivot = new THREE.Group();
fingersRightPivot.position.set(XC_RIGHT, 0, ZC_FS);
armsGroup.add(fingersLeftPivot, fingersRightPivot);
addMesh('fingers_left', fingersLeftPivot, mats.fingers_left);
addMesh('fingers_right', fingersRightPivot, mats.fingers_right);

addMesh('battery_pod', batGroup, mats.battery_pod);
addMesh('battery_lid', batGroup, mats.battery_lid);
addMesh('battery_pack', batGroup, mats.battery_pack);

let mode = 'auto';
let armMm = 0.0, fngDeg = 0.0, liftZ = 0.0, animSpeed = 1.0;

window.setMode = (mName) => {
  mode = mName;
  ['auto', 'stow', 'reach'].forEach(id => document.getElementById('m-' + id).classList.toggle('active', id === mName));
  if (mName === 'stow') { armMm = 0; fngDeg = 0; }
  if (mName === 'reach') { armMm = 205; fngDeg = 90; }
};
window.setPose = (k, v) => {
  mode = 'manual';
  ['auto', 'stow', 'reach'].forEach(id => document.getElementById('m-' + id).classList.remove('active'));
  if (k === 'arm') armMm = v;
  if (k === 'fng') fngDeg = v;
  if (k === 'bz') liftZ = v;
};
window.setSpeed = (v) => {
  animSpeed = v;
  document.getElementById('spd-val').textContent = v.toFixed(1) + '×';
};
window.setVis = (name, st) => {
  if (name === 'col_m') {
    collarGroup.visible = (st !== 'off');
    mats.body_collar.transparent = (st === 'xray');
    mats.body_collar.opacity = (st === 'xray') ? 0.35 : 1.0;
    mats.body_collar.depthWrite = (st !== 'xray');
  } else if (name === 'tray') {
    trayGroup.visible = (st !== 'off');
    mats.tray_deck.transparent = (st === 'xray');
    mats.tray_deck.opacity = (st === 'xray') ? 0.30 : 1.0;
    mats.tray_deck.depthWrite = (st !== 'xray');
  }
  else if (name === 'arms') armsGroup.visible = (st !== 'off');
  else if (name === 'bat') {
    batGroup.visible = (st !== 'off');
    mats.battery_pod.transparent = (st === 'xray');
    mats.battery_pod.opacity = (st === 'xray') ? 0.32 : 1.0;
    mats.battery_pod.depthWrite = (st !== 'xray');
    mats.battery_lid.transparent = (st === 'xray');
    mats.battery_lid.opacity = (st === 'xray') ? 0.25 : 1.0;
    mats.battery_lid.depthWrite = (st !== 'xray');
  } else if (name === 'stub') {
    stubGroup.visible = (st !== 'off');
    mats.column_stub.transparent = (st === 'xray');
    mats.column_stub.opacity = (st === 'xray') ? 0.38 : 1.0;
    mats.column_stub.depthWrite = (st !== 'xray');
  } else if (name === 'env') {
    envGroup.visible = (st !== 'off');
    refBasketMesh.visible = (st !== 'off');
    mats.ref_basket.transparent = (st === 'xray');
    mats.ref_basket.opacity = (st === 'xray') ? 0.25 : 0.85;
    mats.ref_rack_bay.transparent = (st === 'xray');
    mats.ref_rack_bay.opacity = (st === 'xray') ? 0.28 : 0.85;
  }
  ['solid', 'xray', 'off'].forEach(s => {
    const el = document.getElementById(`v-${name}-${s}`);
    if (el) el.classList.toggle('active', s === st);
  });
};
window.resetCam = () => {
  camera.position.set(-220, 420, 480);
  controls.target.set(80, 190, 140);
  controls.update();
};
window.focusAttach = () => {
  camera.position.set(-140, 220 + liftZ, 320);
  controls.target.set(0, 185 + liftZ, 158);
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
  t += dt * animSpeed;

  if (mode === 'auto') {
    // 8s pick-and-place demonstration: extend out -> drop fingers -> pull back -> lift fingers
    const ph = t % 8.0;
    if (ph < 2.5) { armMm = 205 * (ph / 2.5); fngDeg = 0; }
    else if (ph < 3.5) { armMm = 205; fngDeg = 90 * (ph - 2.5); }
    else if (ph < 6.0) { armMm = 205 * (1 - (ph - 3.5) / 2.5); fngDeg = 90; }
    else if (ph < 7.0) { armMm = 0; fngDeg = 90 * (1 - (ph - 6.0)); }
    else { armMm = 0; fngDeg = 0; }
  }

  bodyLiftGroup.position.set(0, 0, liftZ);
  armsMidMesh.position.y = 0.5 * armMm;
  armsInMesh.position.y = armMm;
  fingersLeftPivot.position.set(XC_LEFT, armMm, ZC_FS);
  fingersRightPivot.position.set(XC_RIGHT, armMm, ZC_FS);
  const fRad = fngDeg * Math.PI / 180;
  fingersLeftPivot.rotation.y = fRad;
  fingersRightPivot.rotation.y = -fRad;

  document.getElementById('arm-sl').value = armMm;
  document.getElementById('fng-sl').value = fngDeg;
  document.getElementById('bz-sl').value = liftZ;
  document.getElementById('arm-val').textContent = armMm.toFixed(1) + ' mm';
  document.getElementById('fng-val').textContent = fngDeg.toFixed(1) + '°';
  document.getElementById('bz-val').textContent = liftZ.toFixed(1) + ' mm';

  controls.update();
  renderer.render(scene, camera);
}
animate();
</script>
</body>
</html>
"""


class Handler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        path = self.path.split("?")[0]
        if path in ("/", "/index.html"):
            body = HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif path.startswith("/meshes_body/"):
            fname = os.path.basename(path)
            fpath = os.path.join(BODY_MESH_DIR, fname)
            if os.path.isfile(fpath):
                with open(fpath, "rb") as f:
                    data = f.read()
                self.send_response(200)
                self.send_header("Content-Type", "application/octet-stream")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            else:
                self.send_error(404)
        else:
            self.send_error(404)

    def log_message(self, format, *args):
        pass


def main():
    port = 8848
    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.ThreadingTCPServer(("127.0.0.1", port), Handler) as httpd:
        url = f"http://127.0.0.1:{port}/"
        print(f"Serving Option 1 Body Studio at {url}  (Ctrl+C to stop)")
        threading.Timer(0.6, lambda: webbrowser.open(url)).start()
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
