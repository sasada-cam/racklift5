"""Interactive Three.js Studio for the Dedicated 2-Stage Telescopic Forklift Compartment.

Visualizes the isolated underneath load-carrying base (B02 + TAL220 + B05 + B04),
the bolted R46-B01 sloped skid-wing deck & rear motor housing, and the R47–R52 2-stage
1:2 differential reach forklift with live Pick & Place kinematics and exploded views.

Usage:
  .pixi/envs/default/bin/python sim/view_objects_compartment.py
"""

import http.server
import os
import socketserver
import threading
import webbrowser

HERE = os.path.dirname(os.path.abspath(__file__))
MESH_DIR = os.path.join(HERE, "build", "meshes_objects_compartment")

HTML = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>Objects Compartment — 2-Stage Forklift Assembly</title>
<style>
  * { box-sizing: border-box; }
  body { margin: 0; overflow: hidden; font-family: system-ui, -apple-system, sans-serif; background: #10151f; color: #e6ebf2; }
  #canvas-wrap { position: fixed; inset: 0; }
  #panel {
    position: fixed; top: 14px; left: 14px; width: 368px; max-height: calc(100vh - 28px);
    overflow-y: auto; background: rgba(18, 24, 34, 0.93); backdrop-filter: blur(10px);
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
    border-radius: 6px; padding: 4px 8px; font-size: 11px; cursor: pointer; transition: 0.12s;
  }
  button:hover { background: #2f3a4f; }
  button.active { background: #2563eb; border-color: #3b82f6; color: #fff; }
  button.ok.active { background: #16a34a; border-color: #22c55e; }
  input[type=range] { width: 100%; accent-color: #3b82f6; cursor: pointer; }
  .dot { display: inline-block; width: 8px; height: 8px; border-radius: 50%; margin-right: 5px; }
  .phase-pill {
    display: inline-block; padding: 4px 8px; border-radius: 6px; font-size: 11px; font-weight: 600;
    background: rgba(34, 197, 94, 0.14); color: #86efac; border: 1px solid rgba(34, 197, 94, 0.28);
    width: 100%; margin-bottom: 6px; text-align: center;
  }
  .spec-table { width: 100%; border-collapse: collapse; font-size: 11px; color: #cbd5e1; margin-top: 2px; }
  .spec-table td { padding: 3px 0; border-bottom: 1px solid rgba(255,255,255,0.06); }
  .spec-table td:last-child { text-align: right; color: #93c5fd; font-weight: 600; }
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
  <h1>Objects Compartment · 2-Stage Forklift</h1>
  <div class="sub">R46-B01 Skid Deck + R47–R52 Differential Reach Mechanism</div>

  <div class="sec">
    <div class="sec-title">1. Pick &amp; Place Kinematics</div>
    <div id="phase-pill" class="phase-pill">Cycle: Ready</div>
    <div class="btn-group" style="width:100%;margin-bottom:6px">
      <button id="a-play" class="active ok" style="flex:1" onclick="setAnim(true)">▶ Auto Pick &amp; Place</button>
      <button id="a-pause" style="flex:1" onclick="setAnim(false)">⏸ Manual</button>
      <button onclick="resetPose()">Home</button>
    </div>
    <div class="row"><span>Tine Reach (R50 1× · R51-R52 2×)</span><span id="ext-val">0 mm</span></div>
    <input id="ext-slider" type="range" min="0" max="240" step="1" value="0" oninput="onManual('ext', this.value)"/>
    <div class="row"><span>Robot Leg Z-Lift</span><span id="lift-val">0.0 mm</span></div>
    <input id="lift-slider" type="range" min="0" max="12" step="0.2" value="0" oninput="onManual('lift', this.value)"/>
    <div class="row"><span>Load Y Position (Tray ↔ Shelf)</span><span id="load-val">240 mm</span></div>
    <input id="load-slider" type="range" min="0" max="240" step="1" value="240" oninput="onManual('load', this.value)"/>
    <div class="row" style="margin-top:6px"><span>Exploded Assembly Stack</span><span id="exp-val">0%</span></div>
    <input id="exp-slider" type="range" min="0" max="100" step="1" value="0" oninput="setExplode(this.value)"/>
  </div>

  <div class="sec">
    <div class="sec-title">2. Layer Visibility &amp; Cameras</div>
    <div class="row"><span><span class="dot" style="background:#f97316"></span>R51-R52 Stage-2 Fork Tines + Racks</span><button class="active" onclick="toggleLayer('fork_tines', this)">Shown</button></div>
    <div class="row"><span><span class="dot" style="background:#EAB308"></span>R54-R55 Horizontal 1:2 Pinions + F683ZZ</span><button class="active" onclick="togglePinions(this)">Shown</button></div>
    <div class="row"><span><span class="dot" style="background:#43A047"></span>R50 Stage-1 Bridged H-Carriage</span><button class="active" onclick="toggleLayer('fork_mid_stage', this)">Shown</button></div>
    <div class="row"><span><span class="dot" style="background:#d97706"></span>CNC4759-A Copper T8 Nut</span><button class="active" onclick="toggleLayer('fork_t8_nut', this)">Shown</button></div>
    <div class="row"><span><span class="dot" style="background:#80CBC4"></span>R49 Front 688ZZ Bearing Arch</span><button class="active" onclick="toggleLayer('fork_front_arch', this)">Shown</button></div>
    <div class="row"><span><span class="dot" style="background:#00897B"></span>R47-R48 Base Rails &amp; CNC4759-A</span><button class="active" onclick="toggleForkFixed(this)">Shown</button></div>
    <div class="row"><span><span class="dot" style="background:#26A69A"></span>R46 Deck + Rear KP08/17HS4401</span><button class="active" onclick="toggleLayer('compartment_deck', this)">Shown</button></div>
    <div class="row"><span><span class="dot" style="background:#2E7D32"></span>R53 Removable Rear Under-Tray Tie</span><button class="active" onclick="toggleLayer('fork_under_tie', this)">Shown</button></div>
    <div class="row"><span><span class="dot" style="background:#38bdf8"></span>B04 + Load Cell + Boom Base</span><button class="active" onclick="toggleBaseUnder(this)">Shown</button></div>
    <div class="row"><span><span class="dot" style="background:#eab308"></span>Skidded Load (140×130×80)</span><button onclick="cycleMode('ref_load', this)" data-mode="ghost">Ghost</button></div>
    <div class="row"><span><span class="dot" style="background:#475569"></span>Shelf 1 Bay (184 mm gap)</span><button onclick="cycleMode('ref_rack_bay', this)" data-mode="ghost">Ghost</button></div>
    <div class="btn-group" style="margin-top:8px;width:100%">
      <button style="flex:1" onclick="setCam('iso')">3D Iso</button>
      <button style="flex:1" onclick="setCam('top')">Top</button>
      <button style="flex:1" onclick="setCam('side')">Side</button>
      <button style="flex:1" onclick="setCam('front')">Front</button>
    </div>
  </div>

  <div class="sec">
    <div class="sec-title">3. CNC4759-A Kit &amp; Hardware Specs</div>
    <table class="spec-table">
      <tr><td>Lead Screw Kit</td><td>Duino.lk CNC4759-A (T8 Lead 8)</td></tr>
      <tr><td>Motor (5 mm D-shaft)</td><td>17HS4401 (42.3×42.3×40, MT0382)</td></tr>
      <tr><td>Rear / Front Bearing</td><td>1× KP08 (Rear) + 1× 688ZZ (Front)</td></tr>
      <tr><td>Stage 2 Drive</td><td>1:2 Rack &amp; Pinion (16T MOD 1, 2× F683ZZ)</td></tr>
      <tr><td>Stage 1 / Stage 2 Reach</td><td>+120 mm / +240 mm (44 mm lap)</td></tr>
      <tr><td>CAD Collision Check</td><td>0 Overlaps (Home &amp; +240 mm)</td></tr>
    </table>
  </div>
</div>

<script type="module">
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0d121c);

const camera = new THREE.PerspectiveCamera(42, window.innerWidth / window.innerHeight, 1, 4000);
camera.position.set(420, 340, 380);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(window.innerWidth, window.innerHeight);
document.getElementById('canvas-wrap').appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(180, 195, 130);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.update();

scene.add(new THREE.HemisphereLight(0xdbeafe, 0x1e293b, 1.1));
const d1 = new THREE.DirectionalLight(0xffffff, 1.1); d1.position.set(350, 520, 420); scene.add(d1);
const d2 = new THREE.DirectionalLight(0x93c5fd, 0.45); d2.position.set(-280, 140, -240); scene.add(d2);

const worldRoot = new THREE.Group();
worldRoot.rotation.x = -Math.PI / 2; // CAD Z-up -> Three.js Y-up
scene.add(worldRoot);

const robotLiftGroup = new THREE.Group();
worldRoot.add(robotLiftGroup);

const grid = new THREE.GridHelper(900, 30, 0x263248, 0x18202e);
grid.position.set(180, 0, 130);
scene.add(grid);

function mkMat(hex, opts = {}) {
  return new THREE.MeshStandardMaterial({
    color: hex, roughness: opts.rough ?? 0.45, metalness: opts.metal ?? 0.15,
    transparent: opts.trans ?? false, opacity: opts.op ?? 1.0,
    depthWrite: opts.dw ?? !(opts.trans ?? false),
  });
}

const mats = {
  base_boom:        mkMat(0x2563eb, { metal: 0.25 }),
  load_cell:        mkMat(0xd1d5db, { metal: 0.80, rough: 0.25 }),
  live_wishbone:    mkMat(0x1d4ed8, { metal: 0.22 }),
  tray_deck:        mkMat(0x0ea5e9, { rough: 0.38 }),
  compartment_deck: mkMat(0x26a69a, { rough: 0.36 }),
  fork_under_tie:   mkMat(0x2e7d32, { rough: 0.36 }),
  base_fasteners:   mkMat(0x334155, { metal: 0.75, rough: 0.28 }),
  fork_base_rails:  mkMat(0x00897b, { metal: 0.25, rough: 0.34 }),
  fork_front_arch:  mkMat(0x80cbc4, { rough: 0.34 }),
  fork_mid_stage:   mkMat(0x43a047, { metal: 0.25, rough: 0.34 }),
  fork_pinion_left:  mkMat(0xeab308, { metal: 0.25, rough: 0.34 }),
  fork_pinion_right: mkMat(0xca8a04, { metal: 0.25, rough: 0.34 }),
  fork_t8_nut:      mkMat(0xd97706, { metal: 0.82, rough: 0.22 }),
  fork_mid_fasteners: mkMat(0x334155, { metal: 0.78, rough: 0.26 }),
  fork_tines:       mkMat(0xf97316, { metal: 0.30, rough: 0.32 }),
  fork_drive:       mkMat(0xe2e8f0, { metal: 0.68, rough: 0.30 }),
  ref_load:         mkMat(0xeab308, { trans: true, op: 0.32, dw: false }),
  ref_rack_bay:     mkMat(0x64748b, { trans: true, op: 0.20, dw: false }),
};

const meshes = {};
const loader = new STLLoader();
function loadMesh(name, parent, recenter = null) {
  const holder = new THREE.Group();
  parent.add(holder);
  meshes[name] = holder;
  loader.load('/meshes/' + name + '.stl', (geom) => {
    if (recenter) geom.translate(recenter[0], recenter[1], recenter[2]);
    holder.add(new THREE.Mesh(geom, mats[name]));
  });
}

[
  'base_boom','load_cell','live_wishbone','tray_deck','compartment_deck','fork_under_tie','base_fasteners',
  'fork_base_rails','fork_front_arch','fork_mid_stage',
  'fork_t8_nut','fork_mid_fasteners','fork_tines','fork_drive'
].forEach((k) => loadMesh(k, robotLiftGroup));
loadMesh('fork_pinion_left', robotLiftGroup, [-148.5, 202.0, 0]);
loadMesh('fork_pinion_right', robotLiftGroup, [-211.5, 202.0, 0]);
['ref_load','ref_rack_bay'].forEach((k) => loadMesh(k, worldRoot));

let autoAnim = true;
let animT = 0;
let extMm = 0, liftMm = 0, loadMm = 240, explodeK = 0;
let phaseLabel = 'Home';

window.setAnim = (flag) => {
  autoAnim = flag;
  document.getElementById('a-play').classList.toggle('active', flag);
  document.getElementById('a-play').classList.toggle('ok', flag);
  document.getElementById('a-pause').classList.toggle('active', !flag);
};

window.resetPose = () => {
  animT = 0; extMm = 0; liftMm = 0; loadMm = 240; phaseLabel = 'Home — Load Waiting on Shelf 1';
  syncSliders();
  applyTransforms();
};

window.onManual = (which, val) => {
  setAnim(false);
  if (which === 'ext') extMm = Number(val);
  if (which === 'lift') liftMm = Number(val);
  if (which === 'load') loadMm = Number(val);
  phaseLabel = 'Manual Inspection';
  syncSliders();
  applyTransforms();
};

window.setExplode = (pct) => {
  explodeK = Number(pct) / 100;
  document.getElementById('exp-slider').value = pct;
  document.getElementById('exp-val').textContent = pct + '%';
  applyTransforms();
};

function syncSliders() {
  document.getElementById('ext-slider').value = extMm;
  document.getElementById('ext-val').textContent = extMm.toFixed(0) + ' mm';
  document.getElementById('lift-slider').value = liftMm;
  document.getElementById('lift-val').textContent = liftMm.toFixed(1) + ' mm';
  document.getElementById('load-slider').value = loadMm;
  document.getElementById('load-val').textContent = loadMm.toFixed(0) + ' mm';
  document.getElementById('phase-pill').textContent = phaseLabel;
}

function applyTransforms() {
  if (meshes.base_boom)        meshes.base_boom.position.set(0, 0, -40 * explodeK);
  if (meshes.load_cell)        meshes.load_cell.position.set(0, 0, 0);
  if (meshes.live_wishbone)    meshes.live_wishbone.position.set(0, 0, 26 * explodeK);
  if (meshes.base_fasteners)   meshes.base_fasteners.position.set(0, 0, 14 * explodeK);
  if (meshes.tray_deck)        meshes.tray_deck.position.set(0, 0, 52 * explodeK);
  if (meshes.compartment_deck) meshes.compartment_deck.position.set(0, 0, 86 * explodeK);
  if (meshes.fork_under_tie)   meshes.fork_under_tie.position.set(0, 0, 12 * explodeK);

  if (meshes.fork_base_rails)    meshes.fork_base_rails.position.set(0, 0, 114 * explodeK);
  if (meshes.fork_front_arch)    meshes.fork_front_arch.position.set(0, 0, 114 * explodeK);
  if (meshes.fork_drive)         meshes.fork_drive.position.set(0, 0, 118 * explodeK);
  if (meshes.fork_mid_stage)     meshes.fork_mid_stage.position.set(0, 0.5 * extMm, 142 * explodeK);
  if (meshes.fork_pinion_left) {
    meshes.fork_pinion_left.position.set(148.5, -202.0 + 0.5 * extMm, 152 * explodeK);
    meshes.fork_pinion_left.rotation.z = -(0.5 * extMm) / 8.0;
  }
  if (meshes.fork_pinion_right) {
    meshes.fork_pinion_right.position.set(211.5, -202.0 + 0.5 * extMm, 152 * explodeK);
    meshes.fork_pinion_right.rotation.z = (0.5 * extMm) / 8.0;
  }
  if (meshes.fork_t8_nut)        meshes.fork_t8_nut.position.set(0, 0.5 * extMm, 142 * explodeK);
  if (meshes.fork_mid_fasteners) meshes.fork_mid_fasteners.position.set(0, 0.5 * extMm, 142 * explodeK);
  if (meshes.fork_tines)         meshes.fork_tines.position.set(0, extMm, 170 * explodeK);

  robotLiftGroup.position.set(0, 0, liftMm);

  if (meshes.ref_load) {
    // At rest on C01 (loadMm=0): skids rest on C01 outer sloped wings (Z_shift = 0).
    // When on Shelf 1 (loadMm=240): skids sit at Shelf 1 top 178.0 mm (-5.6 mm below tray height)
    // unless tines are underneath & lifting!
    let loadZ = 0;
    if (loadMm > 200 && liftMm < 2.2) {
      // Resting cleanly on Shelf 1 (Z_skid = 178.0 mm -> delta -5.6 vs tray 183.6)
      loadZ = -5.6;
    } else if (Math.abs(extMm - loadMm) < 35 && liftMm >= 2.2) {
      // Lifted on Fork Tines (+2.2 mm tine-to-box engagement gap)
      loadZ = liftMm - 2.2;
    } else if (loadMm > 10 && loadMm <= 200) {
      loadZ = liftMm - 2.2;
    }
    meshes.ref_load.position.set(0, loadMm, 198 * explodeK + loadZ);
  }
}

window.toggleLayer = (name, btn) => {
  const h = meshes[name];
  if (!h) return;
  h.visible = !h.visible;
  btn.textContent = h.visible ? 'Shown' : 'Hidden';
  btn.classList.toggle('active', h.visible);
};

window.togglePinions = (btn) => {
  const next = !(meshes.fork_pinion_left && meshes.fork_pinion_left.visible);
  ['fork_pinion_left','fork_pinion_right','fork_mid_fasteners'].forEach((k) => { if (meshes[k]) meshes[k].visible = next; });
  btn.textContent = next ? 'Shown' : 'Hidden';
  btn.classList.toggle('active', next);
};

window.toggleForkFixed = (btn) => {
  const next = !(meshes.fork_base_rails && meshes.fork_base_rails.visible);
  ['fork_base_rails','fork_drive'].forEach((k) => { if (meshes[k]) meshes[k].visible = next; });
  btn.textContent = next ? 'Shown' : 'Hidden';
  btn.classList.toggle('active', next);
};

window.toggleBaseUnder = (btn) => {
  const next = !(meshes.base_boom && meshes.base_boom.visible);
  ['base_boom','load_cell','live_wishbone','tray_deck','base_fasteners'].forEach((k) => {
    if (meshes[k]) meshes[k].visible = next;
  });
  btn.textContent = next ? 'Shown' : 'Hidden';
  btn.classList.toggle('active', next);
};

window.cycleMode = (name, btn) => {
  const h = meshes[name], mat = mats[name];
  if (!h || !mat) return;
  const cur = btn.dataset.mode;
  const next = cur === 'ghost' ? 'solid' : (cur === 'solid' ? 'hidden' : 'ghost');
  btn.dataset.mode = next;
  h.visible = next !== 'hidden';
  mat.transparent = next === 'ghost';
  mat.opacity = next === 'ghost' ? (name === 'ref_load' ? 0.32 : 0.20) : 1.0;
  mat.depthWrite = next !== 'ghost';
  btn.textContent = next[0].toUpperCase() + next.slice(1);
  btn.classList.toggle('active', next === 'solid');
};

window.setCam = (preset) => {
  controls.target.set(180, 195, 130);
  if (preset === 'iso')   camera.position.set(420, 340, 380);
  if (preset === 'top')   camera.position.set(180, 580, 130);
  if (preset === 'side')  camera.position.set(490, 210, 130);
  if (preset === 'front') camera.position.set(180, 225, 460);
  controls.update();
};

const sm = (t, a, b) => {
  const u = Math.max(0, Math.min(1, (t - a) / (b - a)));
  return u * u * (3 - 2 * u);
};

let lastT = performance.now();
function tick(now) {
  const dt = Math.min(0.05, (now - lastT) / 1000);
  lastT = now;
  if (autoAnim) {
    animT = (animT + dt) % 12.0;
    const t = animT;
    if (t < 2.2) {
      phaseLabel = '1/5 · Extend Tines Under Shelf Load (+240 mm)';
      extMm = 240 * sm(t, 0.2, 2.2);
      liftMm = 0;
      loadMm = 240;
    } else if (t < 3.4) {
      phaseLabel = '2/5 · Robot Legs Lift +10 mm (Clear of Shelf 1)';
      extMm = 240;
      liftMm = 10 * sm(t, 2.2, 3.2);
      loadMm = 240;
    } else if (t < 5.6) {
      phaseLabel = '3/5 · Retract Load Onto Sloped C01 Deck';
      const u = sm(t, 3.4, 5.4);
      extMm = 240 * (1 - u);
      liftMm = 10;
      loadMm = 240 * (1 - u);
    } else if (t < 6.8) {
      phaseLabel = '4/5 · Settle Skids on C01 (Tines Unload Free)';
      extMm = 0;
      liftMm = 10 * (1 - sm(t, 5.6, 6.6));
      loadMm = 0;
    } else if (t < 7.8) {
      phaseLabel = '5/5 · Seated on C01 + Rear Wall (Ready for Travel)';
      extMm = 0;
      liftMm = 0;
      loadMm = 0;
    } else if (t < 8.8) {
      phaseLabel = 'Place Back · Lift Load (+10 mm)';
      extMm = 0;
      liftMm = 10 * sm(t, 7.8, 8.6);
      loadMm = 0;
    } else if (t < 10.6) {
      phaseLabel = 'Place Back · Extend to Shelf 1 (+240 mm)';
      const u = sm(t, 8.8, 10.4);
      extMm = 240 * u;
      liftMm = 10;
      loadMm = 240 * u;
    } else if (t < 11.4) {
      phaseLabel = 'Place Back · Lower Skids onto Shelf 1';
      extMm = 240;
      liftMm = 10 * (1 - sm(t, 10.6, 11.2));
      loadMm = 240;
    } else {
      phaseLabel = 'Place Back · Retract Empty Tines';
      extMm = 240 * (1 - sm(t, 11.4, 12.0));
      liftMm = 0;
      loadMm = 240;
    }
    syncSliders();
    applyTransforms();
  }
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(tick);
}

resetPose();
requestAnimationFrame(tick);

window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});
</script>
</body>
</html>
"""


class Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
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
            fname = os.path.basename(self.path.split("?")[0])
            fpath = os.path.join(MESH_DIR, fname)
            if not os.path.isfile(fpath):
                self.send_error(404)
                return
            with open(fpath, "rb") as f:
                data = f.read()
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            return
        self.send_error(404)


def main():
    socketserver.ThreadingTCPServer.allow_reuse_address = True
    for port in range(8880, 8896):
        try:
            srv = http.server.ThreadingHTTPServer(("127.0.0.1", port), Handler)
            break
        except OSError:
            continue
    else:
        raise RuntimeError("No free port in 8880..8895")
    url = f"http://127.0.0.1:{port}/"
    print(f"Objects Compartment Forklift Studio -> {url}")
    threading.Timer(0.6, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
