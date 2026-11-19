"""Interactive Three.js 3D Studio for Unified Option 1 Full Robot on Complete 2-Bay 2020 Rails.

Combines:
  - Dual Option 1 swivel legs (H1 @ Z=330.33, H2 mirrored @ Z=90.0) + P08/P09 Hall sensors + magnets
  - 40 x 25 x 2.0 mm hollow aluminium column + 3-zone internal anti-crush plugs (H2, BODY, H1)
  - Complete 2-bay v3 2020 railing set, 4x 17.5P crossing turntables, and 5-shelf slotted-angle racks
  - R40 closed-box body collar + R41 truss boom + 5 kg TAL220 load cell + R45 wishbone + R44 tray
  - Left & right 2-stage telescopic picker arms + flip fingers + Daraz basket
  - R42 counterweight enclosure + triangular knee gusset + flush R43 lid
  - Exploded Assembly View (0% .. 100%) working live alongside climb/travel/swivel/pick animation

Usage:
  pixi run python sim/view_option1_full.py
"""

import http.server
import os
import socketserver
import threading
import webbrowser

HERE = os.path.dirname(os.path.abspath(__file__))
POD_MESH_DIR = os.path.join(HERE, "build", "meshes_t_slot_option1")
FULL_MESH_DIR = os.path.join(HERE, "build", "meshes_option1_full")

HTML = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>RackLift5 — Complete Full Robot System</title>
<style>
  * { box-sizing: border-box; }
  body { margin: 0; overflow: hidden; font-family: system-ui, -apple-system, sans-serif; background: #10151f; color: #e6ebf2; }
  #canvas-wrap { position: fixed; inset: 0; }
  #panel {
    position: fixed; top: 14px; left: 14px; width: 372px; max-height: calc(100vh - 28px);
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
  <h1>RackLift5 · Complete Full Robot System</h1>
  <div class="sub">Mirrored Swivel Legs + Protective Leg Hoods + Telescopic Picker + Mounted 2-Bay Racks</div>

  <div class="sec">
    <div class="sec-title">1. Robot & Picker Motion</div>
    <div class="btn-group" style="width:100%">
      <button id="m-auto" class="active ok" style="flex:1" onclick="setMode('auto')">Auto Full Cycle</button>
      <button id="m-vert" style="flex:1" onclick="setMode('vert')">0° Home</button>
      <button id="m-horiz" style="flex:1" onclick="setMode('horiz')">90° Travel</button>
      <button id="m-reach" style="flex:1" onclick="setMode('reach')">Reach Shelf</button>
    </div>
    <div class="row" style="margin-top:8px">
      <span>Bay Travel X (0 .. 358.14 mm)</span><span id="x-val">0.0 mm</span>
    </div>
    <input id="x-sl" type="range" min="0" max="358.14" step="1" value="0" oninput="setPose('x', Number(this.value))"/>
    <div class="row">
      <span>Climb Height dZ (0 .. 525 mm)</span><span id="z-val">0.0 mm</span>
    </div>
    <input id="z-sl" type="range" min="0" max="525" step="2" value="0" oninput="setPose('z', Number(this.value))"/>
    <div class="row">
      <span>Legs Swivel (0° .. 90°)</span><span id="sw-val">L 0° / U 0°</span>
    </div>
    <input id="sw-sl" type="range" min="0" max="90" step="1" value="0" oninput="setPose('sw', Number(this.value))"/>
    <div class="row">
      <span>Picker Arm Stroke Y (0 .. 212 mm)</span><span id="arm-val">0.0 mm</span>
    </div>
    <input id="arm-sl" type="range" min="0" max="212" step="1" value="0" oninput="setPose('arm', Number(this.value))"/>
    <div class="row">
      <span>Flip Fingers (0° .. 90°)</span><span id="fng-val">0.0°</span>
    </div>
    <input id="fng-sl" type="range" min="0" max="90" step="1" value="0" oninput="setPose('fng', Number(this.value))"/>
    <div class="row">
      <span>Animation Speed (0× Pause .. 3×)</span><span id="spd-val">1.0×</span>
    </div>
    <input id="spd-sl" type="range" min="0" max="3" step="0.1" value="1" oninput="setSpeed(Number(this.value))"/>
  </div>

  <div class="sec">
    <div class="sec-title">2. Exploded Assembly View</div>
    <div class="row">
      <span>Exploded View</span><span id="exp-val">0%</span>
    </div>
    <input id="exp-sl" type="range" min="0" max="100" step="1" value="0" oninput="setExplode(Number(this.value))"/>
    <div class="btn-group" style="width:100%; margin-top:4px">
      <button id="e-0" class="active" style="flex:1" onclick="setExplode(0)">Assembled (0%)</button>
      <button id="e-50" style="flex:1" onclick="setExplode(50)">Half (50%)</button>
      <button id="e-100" style="flex:1" onclick="setExplode(100)">Exploded (100%)</button>
    </div>
  </div>

  <div class="sec">
    <div class="sec-title">3. Layers</div>
    <div class="row">
      <span><span class="dot" style="background:#2e8ee6"></span>Dual Swivel Legs (H1, H2)</span>
      <div class="btn-group">
        <button id="v-legs-solid" class="active" onclick="setVis('legs','solid')">Solid</button>
        <button id="v-legs-xray" onclick="setVis('legs','xray')">Ghost</button>
        <button id="v-legs-off" onclick="setVis('legs','off')">Hide</button>
      </div>
    </div>
    <div class="row">
      <span><span class="dot" style="background:#a855f7"></span>Stationary Leg Hoods (H1/H2)</span>
      <div class="btn-group">
        <button id="v-cov-solid" onclick="setVis('cov','solid')">Solid</button>
        <button id="v-cov-xray" class="active" onclick="setVis('cov','xray')">Ghost</button>
        <button id="v-cov-off" onclick="setVis('cov','off')">Hide</button>
      </div>
    </div>
    <div class="row">
      <span><span class="dot" style="background:#cbd5e1"></span>40×25×2 Column + 3× Plugs</span>
      <div class="btn-group">
        <button id="v-col-solid" onclick="setVis('col','solid')">Solid</button>
        <button id="v-col-xray" class="active" onclick="setVis('col','xray')">Ghost</button>
        <button id="v-col-off" onclick="setVis('col','off')">Hide</button>
      </div>
    </div>
    <div class="row">
      <span><span class="dot" style="background:#1f7a4d"></span>R40 Closed Collar + Bolts</span>
      <div class="btn-group">
        <button id="v-col_m-solid" class="active" onclick="setVis('col_m','solid')">Solid</button>
        <button id="v-col_m-xray" onclick="setVis('col_m','xray')">Ghost</button>
        <button id="v-col_m-off" onclick="setVis('col_m','off')">Hide</button>
      </div>
    </div>
    <div class="row">
      <span><span class="dot" style="background:#4caf50"></span>R44 Tray + R41 Boom + R45</span>
      <div class="btn-group">
        <button id="v-tray-solid" class="active" onclick="setVis('tray','solid')">Solid</button>
        <button id="v-tray-xray" onclick="setVis('tray','xray')">Ghost</button>
        <button id="v-tray-off" onclick="setVis('tray','off')">Hide</button>
      </div>
    </div>
    <div class="row">
      <span><span class="dot" style="background:#f59e0b"></span>Telescopic Arms + Fingers</span>
      <div class="btn-group">
        <button id="v-arms-solid" class="active" onclick="setVis('arms','solid')">Show</button>
        <button id="v-arms-off" onclick="setVis('arms','off')">Hide</button>
      </div>
    </div>
    <div class="row">
      <span><span class="dot" style="background:#66bb6a"></span>R42 Battery + R43 Lid</span>
      <div class="btn-group">
        <button id="v-bat-solid" class="active" onclick="setVis('bat','solid')">Solid</button>
        <button id="v-bat-xray" onclick="setVis('bat','xray')">Ghost</button>
        <button id="v-bat-off" onclick="setVis('bat','off')">Hide</button>
      </div>
    </div>
    <div class="row">
      <span><span class="dot" style="background:#94a3b8"></span>2020 Rails + Racks + Crossings</span>
      <div class="btn-group">
        <button id="v-rails-solid" class="active" onclick="setVis('rails','solid')">Show</button>
        <button id="v-rails-off" onclick="setVis('rails','off')">Hide</button>
      </div>
    </div>
    <div class="row">
      <span><span class="dot" style="background:#fb7185"></span>Rack Mount Brackets + Bolts</span>
      <div class="btn-group">
        <button id="v-rmnt-solid" class="active" onclick="setVis('rmnt','solid')">Solid</button>
        <button id="v-rmnt-xray" onclick="setVis('rmnt','xray')">Ghost</button>
        <button id="v-rmnt-off" onclick="setVis('rmnt','off')">Hide</button>
      </div>
    </div>
    <div class="row">
      <span><span class="dot" style="background:#64748b"></span>Ref 2-Bay Racks + Basket</span>
      <div class="btn-group">
        <button id="v-racks-solid" onclick="setVis('racks','solid')">Solid</button>
        <button id="v-racks-xray" class="active" onclick="setVis('racks','xray')">Ghost</button>
        <button id="v-racks-off" onclick="setVis('racks','off')">Hide</button>
      </div>
    </div>
    <div class="row">
      <span><span class="dot" style="background:#06b6d4"></span>Hall Sensors + Magnets</span>
      <div class="btn-group">
        <button id="v-sens-solid" class="active" onclick="setVis('sens','solid')">Show</button>
        <button id="v-sens-off" onclick="setVis('sens','off')">Hide</button>
      </div>
    </div>
  </div>

  <div class="sec">
    <div class="btn-group" style="width:100%">
      <button style="flex:1" onclick="resetCam()">Reset View</button>
      <button style="flex:1" onclick="focusBody()">Zoom Body</button>
      <button style="flex:1" onclick="focusLegs()">Zoom Legs</button>
    </div>
  </div>
</div>

<script type="module">
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';

const BAY_P = 76 * Math.PI * 1.5; // 358.14 mm compact 76-pitch bay spacing
const JZ_LO = 90.0;
const JZ_UP = JZ_LO + 51 * Math.PI * 1.5; // 330.33 mm
const BXR = 180.0;
const XC_LEFT = BXR - 80.5;
const XC_RIGHT = BXR + 80.5;
const ZC_FS = 178.0 + 62.0;

const wrap = document.getElementById('canvas-wrap');
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x10151f);

const camera = new THREE.PerspectiveCamera(42, window.innerWidth / window.innerHeight, 1, 6000);
camera.position.set(520, 440, 680);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(window.innerWidth, window.innerHeight);
wrap.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.target.set(180, 250, -40);
controls.update();

scene.add(new THREE.HemisphereLight(0xdbeafe, 0x1e293b, 1.35));
const key = new THREE.DirectionalLight(0xffffff, 1.5);
key.position.set(350, 750, 520);
scene.add(key);

const grid = new THREE.GridHelper(1800, 36, 0x334155, 0x1e293b);
grid.position.set(220, 0, -50);
scene.add(grid);

const worldRoot = new THREE.Group();
worldRoot.rotation.x = -Math.PI / 2;
scene.add(worldRoot);

const loader = new STLLoader();
const mat = (hex, extra={}) => new THREE.MeshStandardMaterial({ color: hex, roughness: 0.42, ...extra });

const mats = {
  // Pod & crossings
  body_mount:   mat(0x33b873),
  bearing_bot:  mat(0xd8dee6, { metalness: 0.85, roughness: 0.25 }),
  bearing_top:  mat(0xd8dee6, { metalness: 0.85, roughness: 0.25 }),
  bearing_pinion: mat(0xd8dee6, { metalness: 0.85, roughness: 0.25 }),
  m8_hardware:  mat(0x64748b, { metalness: 0.80, roughness: 0.30 }),
  servo_body:   mat(0xc41e24, { metalness: 0.50, roughness: 0.35 }),
  servo_spline: mat(0xd4ad52, { metalness: 0.85, roughness: 0.25 }),
  alum_horn:    mat(0xe04040, { metalness: 0.75, roughness: 0.30 }),
  gear_20t:     mat(0xf59e0b),
  carriage_pod: mat(0x2e8ee6),
  motor_housing:mat(0xb4bcc8, { metalness: 0.65, roughness: 0.35 }),
  motor_shaft:  mat(0xd8dee6, { metalness: 0.85, roughness: 0.20 }),
  pinion:       mat(0xeb7a24),
  retainer:     mat(0x333942),
  gantry_plate: mat(0x22262d, { metalness: 0.50 }),
  v_wheels:     mat(0x3c414a),
  pinion_cover: mat(0x8fd0ff),
  motor_cap:    mat(0x1b4a99),
  screw_pinion: mat(0x9aa5b4, { metalness: 0.90, roughness: 0.25 }),
  screws_pcover:mat(0x9aa5b4, { metalness: 0.90, roughness: 0.25 }),
  screws_cap:   mat(0x9aa5b4, { metalness: 0.90, roughness: 0.25 }),
  fasteners_turn: mat(0x9aa5b4, { metalness: 0.90, roughness: 0.25 }),
  fasteners_fixed:mat(0x9aa5b4, { metalness: 0.90, roughness: 0.25 }),
  fasteners_gear: mat(0x9aa5b4, { metalness: 0.90, roughness: 0.25 }),
  xp_rail:      mat(0xe2e8f0, { metalness: 0.70, roughness: 0.30 }),
  xp_rack:      mat(0xf59e0b),
  xp_rotor:     mat(0xa8b2c0, { metalness: 0.70, roughness: 0.35 }),
  xp_pivot_608zz:mat(0xd8dee6, { metalness: 0.85, roughness: 0.25 }),
  xp_pivot_m8_stack:mat(0x64748b, { metalness: 0.80, roughness: 0.30 }),
  xp_rotor_screws:mat(0x64748b, { metalness: 0.80, roughness: 0.30 }),
  xp_tnuts:     mat(0x9aa5b4, { metalness: 0.90, roughness: 0.25 }),
  xp_stop_pin:  mat(0x64748b, { metalness: 0.80, roughness: 0.30 }),
  // Dual rails + racks + sensors
  column_bar:   mat(0xcbd5e1, { metalness: 0.70, roughness: 0.30, transparent: true, opacity: 0.45, depthWrite: false }),
  anti_crush:   mat(0xf59e0b),
  rails_vert:   mat(0x94a3b8, { metalness: 0.70, roughness: 0.30 }),
  rails_horiz:  mat(0x94a3b8, { metalness: 0.70, roughness: 0.30 }),
  racks_vert:   mat(0xd97706),
  racks_horiz:  mat(0xd97706),
  crossings_fixed: mat(0x7c8a9c, { metalness: 0.60 }),
  rack_bays:    mat(0x64748b, { transparent: true, opacity: 0.22, depthWrite: false }),
  rack_mounts:  mat(0xfb7185, { metalness: 0.55, roughness: 0.35 }),
  rack_mount_hw:mat(0xcbd5e1, { metalness: 0.90, roughness: 0.22 }),
  sensors_pod_fixed: mat(0x06b6d4, { roughness: 0.30 }),
  sensors_pod_turn:  mat(0x06b6d4, { roughness: 0.30 }),
  sensors_hardware:  mat(0xcbd5e1, { metalness: 0.90, roughness: 0.22 }),
  leg_hood_upper:    mat(0xa855f7, { roughness: 0.35, transparent: true, opacity: 0.38, depthWrite: false }),
  leg_hood_lower:    mat(0x9333ea, { roughness: 0.35, transparent: true, opacity: 0.38, depthWrite: false }),
  crossing_magnets:  mat(0xe11d48, { metalness: 0.70, roughness: 0.25 }),
  // Body + Picker
  body_collar:  mat(0x1f7a4d),
  body_fasteners:mat(0x94a3b8, { metalness: 0.85, roughness: 0.25 }),
  tray_boom:    mat(0x4caf50),
  live_wishbone:mat(0x2e6b47),
  load_cell:    mat(0xe2e8f0, { metalness: 0.75, roughness: 0.30 }),
  tray_deck:    mat(0x7fd8a6, { roughness: 0.45 }),
  tray_stops:   mat(0x64748b),
  arms_outer:   mat(0x94a3b8, { metalness: 0.65 }),
  arms_mid:     mat(0x7c8a9c),
  arms_in:      mat(0xa8b2c0),
  fingers_left: mat(0x00897b),
  fingers_right:mat(0x00897b),
  picker_belts: mat(0x1e293b, { roughness: 0.8 }),
  picker_servos:mat(0xc41e24, { metalness: 0.45 }),
  battery_pod:  mat(0x66bb6a),
  battery_lid:  mat(0x388e3c),
  battery_pack: mat(0x334155),
  ref_basket:   mat(0x38bdf8, { transparent: true, opacity: 0.28, depthWrite: false }),
};

const geomCache = {};
function loadStl(url, onLoad) {
  if (geomCache[url]) { onLoad(geomCache[url]); return; }
  loader.load(url, (g) => {
    g.computeVertexNormals();
    geomCache[url] = g;
    onLoad(g);
  });
}
function addMesh(url, parent, material) {
  const holder = new THREE.Group();
  parent.add(holder);
  loadStl(url, (g) => holder.add(new THREE.Mesh(g, material)));
  return holder;
}

// Fixed environment groups
const railsGroup = new THREE.Group();
const crossFixedGroup = new THREE.Group();
const racksRefGroup = new THREE.Group();
const rackMountsGroup = new THREE.Group();
const sensorsEnvGroup = new THREE.Group();
worldRoot.add(railsGroup, crossFixedGroup, racksRefGroup, rackMountsGroup, sensorsEnvGroup);

addMesh('/meshes_full/rails_vert.stl', railsGroup, mats.rails_vert);
addMesh('/meshes_full/racks_vert.stl', railsGroup, mats.racks_vert);
addMesh('/meshes_full/rails_horiz.stl', railsGroup, mats.rails_horiz);
addMesh('/meshes_full/racks_horiz.stl', railsGroup, mats.racks_horiz);
addMesh('/meshes_full/crossings_fixed.stl', crossFixedGroup, mats.crossings_fixed);
addMesh('/meshes_full/rack_bays.stl', racksRefGroup, mats.rack_bays);
addMesh('/meshes_full/rack_mounts.stl', rackMountsGroup, mats.rack_mounts);
addMesh('/meshes_full/rack_mount_hw.stl', rackMountsGroup, mats.rack_mount_hw);
addMesh('/meshes_full/crossing_magnets.stl', sensorsEnvGroup, mats.crossing_magnets);

function applyPodToHub(grp) {
  grp.matrixAutoUpdate = false;
  grp.matrix.set(
    0, -1,  0,  0,
    0,  0, -1, -62.475,
    1,  0,  0,  0,
    0,  0,  0,  1
  );
}

// 4x Crossing turntables
const crossTurnGroup = new THREE.Group();
worldRoot.add(crossTurnGroup);
const crossingTurns = {};
for (const [bIdx, bx] of [[0, 0], [1, BAY_P]]) {
  for (const [tag, zc] of [['lo', JZ_LO], ['up', JZ_UP]]) {
    const outer = new THREE.Group();
    outer.position.set(bx, 0, zc);
    crossTurnGroup.add(outer);
    const podMap = new THREE.Group();
    applyPodToHub(podMap);
    outer.add(podMap);
    const rot = new THREE.Group();
    podMap.add(rot);
    for (const k of ['xp_rail', 'xp_rack', 'xp_rotor', 'xp_pivot_608zz', 'xp_pivot_m8_stack', 'xp_rotor_screws', 'xp_stop_pin', 'xp_tnuts']) {
      addMesh('/meshes_pod/' + k + '.stl', rot, mats[k]);
    }
    crossingTurns[`b${bIdx}_${tag}`] = rot;
  }
}

// Unified Robot moving group
const robotGroup = new THREE.Group();
worldRoot.add(robotGroup);
const colGroup = addMesh('/meshes_full/column_bar.stl', robotGroup, mats.column_bar);
const plugGroup = addMesh('/meshes_full/anti_crush_inserts.stl', robotGroup, mats.anti_crush);

// Dual Legs (H1 upper, H2 lower) + stationary column-mounted Lampshade Hoods (legs swivel inside)
const legMeshes = [];
const sensorPodHolders = [];
const hoodUpperHolder = addMesh('/meshes_full/leg_hood_upper.stl', robotGroup, mats.leg_hood_upper);
const hoodLowerHolder = addMesh('/meshes_full/leg_hood_lower.stl', robotGroup, mats.leg_hood_lower);
function createHubLeg(zc, mirror) {
  const hubPos = new THREE.Group();
  hubPos.position.set(0, 0, zc);
  if (mirror) hubPos.rotation.y = Math.PI;
  robotGroup.add(hubPos);

  const podMap = new THREE.Group();
  applyPodToHub(podMap);
  hubPos.add(podMap);

  const fixedG = new THREE.Group();
  const servoPivot = new THREE.Group();
  servoPivot.position.set(-37.5, 0, 0);
  const carriagePivot = new THREE.Group();
  const climbPivot = new THREE.Group();
  climbPivot.position.set(52.5, 0, 5.325);
  carriagePivot.add(climbPivot);
  podMap.add(fixedG, servoPivot, carriagePivot);

  for (const k of ['body_mount', 'bearing_bot', 'bearing_top', 'm8_hardware', 'servo_body', 'fasteners_fixed']) {
    addMesh('/meshes_pod/' + k + '.stl', fixedG, mats[k]);
  }
  for (const k of ['gear_20t', 'alum_horn', 'servo_spline', 'fasteners_gear']) {
    addMesh('/meshes_pod/' + k + '.stl', servoPivot, mats[k]);
  }
  for (const k of ['carriage_pod', 'motor_cap', 'pinion_cover', 'bearing_pinion', 'motor_housing', 'gantry_plate', 'v_wheels', 'fasteners_turn', 'screws_pcover', 'screws_cap']) {
    addMesh('/meshes_pod/' + k + '.stl', carriagePivot, mats[k]);
  }
  for (const k of ['motor_shaft', 'pinion', 'retainer', 'screw_pinion']) {
    addMesh('/meshes_pod/' + k + '.stl', climbPivot, mats[k]);
  }
  sensorPodHolders.push(addMesh('/meshes_full/sensors_pod_fixed.stl', fixedG, mats.sensors_pod_fixed));
  sensorPodHolders.push(addMesh('/meshes_full/sensors_pod_turn.stl', carriagePivot, mats.sensors_pod_turn));
  sensorPodHolders.push(addMesh('/meshes_full/sensors_hardware.stl', carriagePivot, mats.sensors_hardware));
  legMeshes.push(hubPos);
  return { hubPos, carriagePivot, servoPivot, climbPivot, zc };
}

const lowerHub = createHubLeg(JZ_LO, true);
const upperHub = createHubLeg(JZ_UP, false);

// Body + Picker groups attached to robotGroup
const collarGroup = new THREE.Group();
const trayGroup = new THREE.Group();
const armsGroup = new THREE.Group();
const batGroup = new THREE.Group();
robotGroup.add(collarGroup, trayGroup, armsGroup, batGroup);

const bodyMeshes = {};
const regBody = (name, parent, material) => {
  const h = addMesh('/meshes_full/' + name + '.stl', parent, material);
  bodyMeshes[name] = h;
  return h;
};

regBody('body_collar', collarGroup, mats.body_collar);
regBody('body_fasteners', collarGroup, mats.body_fasteners);
regBody('tray_boom', trayGroup, mats.tray_boom);
regBody('live_wishbone', trayGroup, mats.live_wishbone);
regBody('load_cell', trayGroup, mats.load_cell);
regBody('tray_deck', trayGroup, mats.tray_deck);
regBody('tray_stops', trayGroup, mats.tray_stops);
const refBasketMesh = regBody('ref_basket', robotGroup, mats.ref_basket);

regBody('arms_outer', armsGroup, mats.arms_outer);
regBody('picker_belts', armsGroup, mats.picker_belts);
regBody('picker_servos', armsGroup, mats.picker_servos);
const armsMidMesh = regBody('arms_mid', armsGroup, mats.arms_mid);
const armsInMesh = regBody('arms_in', armsGroup, mats.arms_in);

const fingersLeftPivot = new THREE.Group();
fingersLeftPivot.position.set(XC_LEFT, 0, ZC_FS);
const fingersRightPivot = new THREE.Group();
fingersRightPivot.position.set(XC_RIGHT, 0, ZC_FS);
armsGroup.add(fingersLeftPivot, fingersRightPivot);
regBody('fingers_left', fingersLeftPivot, mats.fingers_left);
regBody('fingers_right', fingersRightPivot, mats.fingers_right);

regBody('battery_pod', batGroup, mats.battery_pod);
regBody('battery_lid', batGroup, mats.battery_lid);
regBody('battery_pack', batGroup, mats.battery_pack);

// ---- Motion Planner: Legs climb/swivel/travel + Picker extend/grab ----
let mode = 'auto';
let poseX = 0.0, poseZ = 0.0, swL = 0.0, swU = 0.0, armMm = 0.0, fngDeg = 0.0;
let tx = 0.0, tz = 0.0, tsw = 0.0, tArm = 0.0, tFng = 0.0;
let animSpeed = 1.0, explodeK = 0.0;
const BAYS = [0.0, BAY_P];
const crossAng = { b0_lo: 0, b0_up: 0, b1_lo: 90, b1_up: 90 };
const SPD = { x: 165, z: 165, sw: 80, arm: 150, fng: 120 };
const nearestBay = (x) => BAYS.reduce((a, b) => Math.abs(b - x) < Math.abs(a - x) ? b : a);
const approach = (v, goal, maxStep) => {
  const d = goal - v;
  const step = Math.sign(d) * Math.min(Math.abs(d), maxStep, Math.max(Math.abs(d) * 0.25, maxStep * 0.15));
  return Math.abs(d) < 1e-3 ? goal : v + step;
};

function stepPlanner(dt) {
  const eps = 0.05;
  const sx = SPD.x * dt, sz = SPD.z * dt, ss = SPD.sw * dt, sa = SPD.arm * dt, sf = SPD.fng * dt;
  // the arm is fully in before the robot climbs or travels (every mode: a slider drag cannot drive it into a rack post)
  const atBayT = BAYS.some(b => Math.abs(tx - b) < 0.5);
  const legsMove = Math.abs(poseX - tx) > eps || Math.abs(poseZ - (atBayT ? tz : 0)) > eps;
  if (legsMove && armMm > eps) { armMm = approach(armMm, 0, sa); return; }
  // at the shelf: fingers first, then the arm (pick: extend with fingers up -> fingers down behind the basket -> pull;
  // put-back: fingers down behind the basket on the tray -> push it out -> fingers up -> retract) - never both at once
  if (!legsMove) {
    const armGoal = BAYS.some(b => Math.abs(poseX - b) < 0.5) ? tArm : 0;   // the arm only goes out at a bay (in front of a shelf)
    if (Math.abs(fngDeg - tFng) > eps) fngDeg = approach(fngDeg, tFng, sf);
    else armMm = approach(armMm, armGoal, sa);
  }
  if (Math.abs(poseX - tx) > eps) {
    if (poseZ > eps) poseZ = approach(poseZ, 0, sz);
    else if (swL < 90 - eps) swL = approach(swL, 90, ss);
    else if (swU < 90 - eps) swU = approach(swU, 90, ss);
    else poseX = approach(poseX, tx, sx);
    return;
  }
  const atBay = BAYS.some(b => Math.abs(poseX - b) < 0.5);
  const wantZ = atBay ? tz : 0;
  const wantSw = wantZ > eps ? 0 : (atBay ? tsw : 90);
  if (wantZ > eps || poseZ > eps) {
    if (swL > eps) swL = approach(swL, 0, ss);
    else if (swU > eps) swU = approach(swU, 0, ss);
    else poseZ = approach(poseZ, wantZ, sz);
    return;
  }
  if (Math.abs(swL - wantSw) > eps) swL = approach(swL, wantSw, ss);
  else if (Math.abs(swU - wantSw) > eps) swU = approach(swU, wantSw, ss);
}
const reached = () => (
  Math.abs(poseX - tx) < 0.1 && Math.abs(poseZ - (BAYS.some(b => Math.abs(tx - b) < 0.5) ? tz : 0)) < 0.1 &&
  Math.abs(swL - (tz > 0.1 ? 0 : tsw)) < 0.1 && Math.abs(swU - (tz > 0.1 ? 0 : tsw)) < 0.1 &&
  Math.abs(armMm - tArm) < 0.1 && Math.abs(fngDeg - tFng) < 0.1
);

// Auto demo: climb Bay 0 Shelf 2 -> reach & grab -> retract -> down -> travel to Bay 1 -> reach Shelf 1 -> home
const AUTO = [
  [0, 175, 0, 0, 0],
  [0, 175, 0, 212, 0],
  [0, 175, 0, 212, 90],
  [0, 175, 0, 0, 90],
  [0, 0, 0, 0, 0],
  [BAY_P, 0, 0, 0, 0],
  [BAY_P, 0, 0, 0, 90],
  [BAY_P, 0, 0, 212, 90],
  [BAY_P, 0, 0, 212, 0],
  [BAY_P, 0, 0, 0, 0],
  [0, 0, 0, 0, 0],
];
let wp = 0, dwell = 0;

const setActive = (m) => ['auto', 'vert', 'horiz', 'reach'].forEach(id => {
  const el = document.getElementById('m-' + id);
  if (el) el.classList.toggle('active', id === m);
});

window.setMode = (m) => {
  mode = m === 'auto' ? 'auto' : 'manual';
  setActive(m);
  if (m === 'auto') { wp = 0; dwell = 0; }
  if (m === 'vert') { tx = nearestBay(poseX); tz = 0; tsw = 0; tArm = 0; tFng = 0; }
  if (m === 'horiz') { tz = 0; tsw = 90; tArm = 0; tFng = 0; }
  if (m === 'reach') { tx = nearestBay(poseX); tz = 0; tsw = 0; tArm = 212; tFng = 90; }
};

window.setPose = (axis, val) => {
  mode = 'manual';
  setActive('');
  if (axis === 'x') { tx = val; tz = 0; tsw = 90; }
  if (axis === 'z') { tx = nearestBay(poseX); tz = val; tsw = 0; }
  if (axis === 'sw') { tx = nearestBay(poseX); tz = 0; tsw = val; }
  if (axis === 'arm') { tArm = val; }
  if (axis === 'fng') { tFng = val; }
};

window.setSpeed = (v) => {
  animSpeed = v;
  document.getElementById('spd-val').textContent = v.toFixed(1) + '×';
};

window.setExplode = (pct) => {
  const val = Math.max(0, Math.min(100, Number(pct)));
  explodeK = val / 100.0;
  const k = explodeK;
  document.getElementById('exp-sl').value = val;
  document.getElementById('exp-val').textContent = Math.round(val) + '%';
  ['0', '50', '100'].forEach(p => {
    const el = document.getElementById('e-' + p);
    if (el) el.classList.toggle('active', Math.round(val) === Number(p));
  });

  // Leg hubs separate vertically slightly; stationary lampshade hoods slide forward (-Y) off TOP H1 / BOT H2 column; plugs lift up
  lowerHub.hubPos.position.set(0, 0, JZ_LO - 20 * k);
  upperHub.hubPos.position.set(0, 0, JZ_UP + 20 * k);
  hoodUpperHolder.position.set(0, -68 * k, 20 * k);
  hoodLowerHolder.position.set(0, -68 * k, -20 * k);
  plugGroup.position.set(0, 0, 35 * k);

  // Body collar + bolts pull forward (-Y) off column
  bodyMeshes.body_collar.position.set(0, -45 * k, 0);
  bodyMeshes.body_fasteners.position.set(0, -90 * k, 0);
  // +X payload stack
  bodyMeshes.tray_boom.position.set(60 * k, 0, -22 * k);
  bodyMeshes.load_cell.position.set(60 * k, 0, 18 * k);
  bodyMeshes.live_wishbone.position.set(60 * k, 0, 52 * k);
  bodyMeshes.tray_deck.position.set(60 * k, 0, 88 * k);
  bodyMeshes.tray_stops.position.set(60 * k, 0, 112 * k);
  refBasketMesh.position.set(60 * k, 0, 130 * k);
  armsGroup.position.set(60 * k, 0, 120 * k);
  // -X counterweight stack
  bodyMeshes.battery_pod.position.set(-60 * k, 0, 0);
  bodyMeshes.battery_pack.position.set(-60 * k, 0, 52 * k);
  bodyMeshes.battery_lid.position.set(-60 * k, 0, 102 * k);
};

window.setVis = (name, state) => {
  if (name === 'legs') {
    legMeshes.forEach(g => g.visible = (state !== 'off'));
    for (const k of ['body_mount','bearing_bot','bearing_top','bearing_pinion','m8_hardware','servo_body','servo_spline','alum_horn','gear_20t','carriage_pod','motor_housing','motor_shaft','pinion','retainer','gantry_plate','v_wheels','pinion_cover','motor_cap']) {
      mats[k].transparent = (state === 'xray');
      mats[k].opacity = (state === 'xray') ? 0.35 : 1.0;
      mats[k].depthWrite = (state !== 'xray');
    }
  } else if (name === 'col') {
    colGroup.visible = (state !== 'off');
    plugGroup.visible = (state !== 'off');
    mats.column_bar.transparent = (state === 'xray');
    mats.column_bar.opacity = (state === 'xray') ? 0.32 : 1.0;
    mats.column_bar.depthWrite = (state !== 'xray');
  } else if (name === 'col_m') {
    collarGroup.visible = (state !== 'off');
    mats.body_collar.transparent = (state === 'xray');
    mats.body_collar.opacity = (state === 'xray') ? 0.35 : 1.0;
    mats.body_collar.depthWrite = (state !== 'xray');
  } else if (name === 'tray') {
    trayGroup.visible = (state !== 'off');
    mats.tray_deck.transparent = (state === 'xray');
    mats.tray_deck.opacity = (state === 'xray') ? 0.30 : 1.0;
    mats.tray_deck.depthWrite = (state !== 'xray');
  } else if (name === 'arms') {
    armsGroup.visible = (state !== 'off');
  } else if (name === 'bat') {
    batGroup.visible = (state !== 'off');
    mats.battery_pod.transparent = (state === 'xray');
    mats.battery_pod.opacity = (state === 'xray') ? 0.32 : 1.0;
    mats.battery_pod.depthWrite = (state !== 'xray');
    mats.battery_lid.transparent = (state === 'xray');
    mats.battery_lid.opacity = (state === 'xray') ? 0.28 : 1.0;
    mats.battery_lid.depthWrite = (state !== 'xray');
  } else if (name === 'rails') {
    railsGroup.visible = (state !== 'off');
    crossFixedGroup.visible = (state !== 'off');
    crossTurnGroup.visible = (state !== 'off');
  } else if (name === 'racks') {
    racksRefGroup.visible = (state !== 'off');
    refBasketMesh.visible = (state !== 'off');
    mats.rack_bays.transparent = (state === 'xray');
    mats.rack_bays.opacity = (state === 'xray') ? 0.22 : 0.85;
    mats.rack_bays.depthWrite = (state !== 'xray');
    mats.ref_basket.transparent = (state === 'xray');
    mats.ref_basket.opacity = (state === 'xray') ? 0.28 : 0.85;
  } else if (name === 'sens') {
    sensorsEnvGroup.visible = (state !== 'off');
    sensorPodHolders.forEach(g => g.visible = (state !== 'off'));
  } else if (name === 'cov') {
    hoodUpperHolder.visible = (state !== 'off');
    hoodLowerHolder.visible = (state !== 'off');
    for (const k of ['leg_hood_upper', 'leg_hood_lower']) {
      mats[k].transparent = (state === 'xray');
      mats[k].opacity = (state === 'xray') ? 0.38 : 1.0;
      mats[k].depthWrite = (state !== 'xray');
    }
  } else if (name === 'rmnt') {
    rackMountsGroup.visible = (state !== 'off');
    for (const k of ['rack_mounts', 'rack_mount_hw']) {
      mats[k].transparent = (state === 'xray');
      mats[k].opacity = (state === 'xray') ? 0.30 : 1.0;
      mats[k].depthWrite = (state !== 'xray');
    }
  }
  ['solid', 'xray', 'off'].forEach(s => {
    const el = document.getElementById(`v-${name}-${s}`);
    if (el) el.classList.toggle('active', s === state);
  });
};

window.resetCam = () => {
  camera.position.set(520, 440, 680);
  controls.target.set(180, 250, -40);
  controls.update();
};
window.focusBody = () => {
  camera.position.set(poseX - 180, 240 + poseZ, 340);
  controls.target.set(poseX + 40, 185 + poseZ, 150);
  controls.update();
};
window.focusLegs = () => {
  camera.position.set(poseX - 240, 220 + poseZ, 340);
  controls.target.set(poseX, 210 + poseZ, 60);
  controls.update();
};

window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});

const clock = new THREE.Clock();
function animate() {
  requestAnimationFrame(animate);
  const dt = Math.min(clock.getDelta(), 0.05);

  if (mode === 'auto') {
    [tx, tz, tsw, tArm, tFng] = AUTO[wp];
    if (reached()) { dwell += dt * animSpeed; if (dwell > 0.6) { wp = (wp + 1) % AUTO.length; dwell = 0; } }
  }
  stepPlanner(dt * animSpeed);

  robotGroup.position.set(poseX, 0, poseZ);
  lowerHub.carriagePivot.rotation.z = swL * Math.PI / 180;
  upperHub.carriagePivot.rotation.z = swU * Math.PI / 180;
  lowerHub.servoPivot.rotation.z = -2.0 * swL * Math.PI / 180;
  upperHub.servoPivot.rotation.z = -2.0 * swU * Math.PI / 180;

  BAYS.forEach((bx, i) => {
    if (Math.abs(poseX - bx) < 0.5 && poseZ < 0.05) { crossAng[`b${i}_lo`] = swL; crossAng[`b${i}_up`] = swU; }
  });
  for (const [k, g] of Object.entries(crossingTurns)) g.rotation.z = crossAng[k] * Math.PI / 180;

  const climbAng = (poseZ + poseX) / 10.5;
  upperHub.climbPivot.rotation.y = -climbAng;
  lowerHub.climbPivot.rotation.y = climbAng;

  // Picker arm telescoping + finger rotation + explode offsets
  armsMidMesh.position.set(0, 0.5 * armMm, 18 * explodeK);
  armsInMesh.position.set(0, armMm, 36 * explodeK);
  fingersLeftPivot.position.set(XC_LEFT - 18 * explodeK, armMm, ZC_FS + 44 * explodeK);
  fingersRightPivot.position.set(XC_RIGHT + 18 * explodeK, armMm, ZC_FS + 44 * explodeK);
  const fRad = fngDeg * Math.PI / 180;
  fingersLeftPivot.rotation.y = fRad;
  fingersRightPivot.rotation.y = -fRad;

  if (mode === 'auto') {
    document.getElementById('x-sl').value = poseX;
    document.getElementById('z-sl').value = poseZ;
    document.getElementById('sw-sl').value = (swL + swU) / 2;
    document.getElementById('arm-sl').value = armMm;
    document.getElementById('fng-sl').value = fngDeg;
  }
  document.getElementById('x-val').textContent = poseX.toFixed(1) + ' mm';
  document.getElementById('z-val').textContent = poseZ.toFixed(1) + ' mm';
  document.getElementById('sw-val').textContent = `L ${swL.toFixed(0)}° / U ${swU.toFixed(0)}°`;
  document.getElementById('arm-val').textContent = armMm.toFixed(1) + ' mm';
  document.getElementById('fng-val').textContent = fngDeg.toFixed(1) + '°';

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

        if self.path.startswith("/meshes_pod/") or self.path.startswith("/meshes_full/"):
            base = POD_MESH_DIR if self.path.startswith("/meshes_pod/") else FULL_MESH_DIR
            fname = os.path.basename(self.path.split("?", 1)[0])
            fpath = os.path.join(base, fname)
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
    for port in range(8849, 8865):
        try:
            srv = http.server.ThreadingHTTPServer(("127.0.0.1", port), Handler)
            break
        except OSError:
            continue
    else:
        raise RuntimeError("No free port in 8849..8864")

    url = f"http://127.0.0.1:{port}/"
    print(f"Option 1 Complete Full Robot + Rails Studio running at {url}  (Ctrl+C to stop)")
    threading.Timer(0.4, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        srv.shutdown()


if __name__ == "__main__":
    main()
