"""Interactive Three.js 3D Studio for Dual-Leg Option 1 Robot on Complete 2-Bay 2020 Railing Set.

Shows BOTH swivel legs (Upper Hub H1 at Z = 330.33 mm, Lower Hub H2 at Z = 90.0 mm,
spaced D_HUB = 51 * P = 240.33 mm) bolted onto the 28 x 24 x 340 mm aluminium column bar,
mounted in place on the complete 2-bay railing layout with 4x 17.5-pitch crossing turntables,
exact v3 cut rail lengths, continuous-pitch in-slot racks, and reference 5-shelf rack bays.

Usage:
  python3 sim/view_option1_dual_legs.py
"""

import http.server
import os
import socketserver
import threading
import webbrowser

HERE = os.path.dirname(os.path.abspath(__file__))
POD_MESH_DIR = os.path.join(HERE, "build", "meshes_t_slot_option1")
ENV_MESH_DIR = os.path.join(HERE, "build", "meshes_option1_dual_legs")

HTML = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>Dual-Leg Climber on 2-Bay 2020 Rails</title>
<style>
  * { box-sizing: border-box; }
  body { margin: 0; overflow: hidden; font-family: system-ui, -apple-system, sans-serif; background: #10151f; color: #e6ebf2; }
  #canvas-wrap { position: fixed; inset: 0; }
  #panel {
    position: fixed; top: 14px; left: 14px; width: 356px; max-height: calc(100vh - 28px);
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
  <h1>Dual-Leg Climber on 2-Bay Rails</h1>
  <div class="sub">Mirrored Upper &amp; Lower Swivel Legs (240.33 mm Spacing) · 4 Turntable Crossings</div>

  <div class="sec">
    <div class="sec-title">1. Pose & Motion</div>
    <div class="btn-group" style="width:100%">
      <button id="m-auto" class="active ok" style="flex:1" onclick="setMode('auto')">Auto Bay-to-Bay</button>
      <button id="m-vert" style="flex:1" onclick="setMode('vert')">0° Vertical</button>
      <button id="m-horiz" style="flex:1" onclick="setMode('horiz')">90° Horizontal</button>
    </div>
    <div class="row" style="margin-top:8px">
      <span>Bay Travel X (0 .. 438.25 mm)</span><span id="x-val">0.0 mm</span>
    </div>
    <input id="x-sl" type="range" min="0" max="438.25" step="1" value="0" oninput="setPose('x', Number(this.value))"/>
    <div class="row">
      <span>Climb Height dZ (0 .. 600 mm)</span><span id="z-val">0.0 mm</span>
    </div>
    <input id="z-sl" type="range" min="0" max="600" step="2" value="0" oninput="setPose('z', Number(this.value))"/>
    <div class="row">
      <span>Both Legs Swivel (0° .. 90°)</span><span id="sw-val">0.0°</span>
    </div>
    <input id="sw-sl" type="range" min="0" max="90" step="1" value="0" oninput="setPose('sw', Number(this.value))"/>
    <div class="row">
      <span>Animation Speed (0× Pause .. 3×)</span><span id="spd-val">1.0×</span>
    </div>
    <input id="spd-sl" type="range" min="0" max="3" step="0.1" value="1" oninput="setSpeed(Number(this.value))"/>
  </div>

  <div class="sec">
    <div class="sec-title">2. Layers</div>
    <div class="row">
      <span><span class="dot" style="background:#2e8ee6"></span>Upper + Lower Legs (H1, H2)</span>
      <div class="btn-group">
        <button id="v-legs-solid" class="active" onclick="setVis('legs','solid')">Solid</button>
        <button id="v-legs-xray" onclick="setVis('legs','xray')">Ghost</button>
        <button id="v-legs-off" onclick="setVis('legs','off')">Hide</button>
      </div>
    </div>
    <div class="row">
      <span><span class="dot" style="background:#cbd5e1"></span>40×25×2 Hollow Column + P07 Plugs</span>
      <div class="btn-group">
        <button id="v-col-solid" onclick="setVis('col','solid')">Solid</button>
        <button id="v-col-xray" class="active" onclick="setVis('col','xray')">Ghost</button>
        <button id="v-col-off" onclick="setVis('col','off')">Hide</button>
      </div>
    </div>
    <div class="row">
      <span><span class="dot" style="background:#94a3b8"></span>2020 Rails + Racks (V + H)</span>
      <div class="btn-group">
        <button id="v-rails-solid" class="active" onclick="setVis('rails','solid')">Show</button>
        <button id="v-rails-off" onclick="setVis('rails','off')">Hide</button>
      </div>
    </div>
    <div class="row">
      <span><span class="dot" style="background:#e2e8f0"></span>4× Crossing Turntables</span>
      <div class="btn-group">
        <button id="v-cross-solid" class="active" onclick="setVis('cross','solid')">Show</button>
        <button id="v-cross-off" onclick="setVis('cross','off')">Hide</button>
      </div>
    </div>
    <div class="row">
      <span><span class="dot" style="background:#64748b"></span>Ref 2-Bay Rack Frame (5 shelves)</span>
      <div class="btn-group">
        <button id="v-racks-solid" onclick="setVis('racks','solid')">Solid</button>
        <button id="v-racks-xray" class="active" onclick="setVis('racks','xray')">Ghost</button>
        <button id="v-racks-off" onclick="setVis('racks','off')">Hide</button>
      </div>
    </div>
    <div class="row">
      <span><span class="dot" style="background:#06b6d4"></span>Hall Sensors (Cyan) + Magnets (Red)</span>
      <div class="btn-group">
        <button id="v-sens-solid" class="active" onclick="setVis('sens','solid')">Show</button>
        <button id="v-sens-off" onclick="setVis('sens','off')">Hide</button>
      </div>
    </div>
  </div>

  <div class="sec">
    <div class="sec-title">3. Rail Cut Lengths (v3 Exact)</div>
    <table class="cut-table">
      <tr><td>4× Crossing pieces (17.5 P)</td><td>168.65 mm</td></tr>
      <tr><td>2× Vertical middle bridge</td><td>69.69 mm</td></tr>
      <tr><td>2× Vertical upper climb</td><td>664.35 mm</td></tr>
      <tr><td>2× Horizontal bay span</td><td>267.61 mm</td></tr>
      <tr><td>2× Horizontal left tail</td><td>94.68 mm</td></tr>
      <tr><td>2× Horizontal right overhang</td><td>226.43 mm</td></tr>
      <tr><td>Hub spacing D_HUB (51 P)</td><td>240.33 mm</td></tr>
      <tr><td>Bay pitch BAY_P (93 P)</td><td>438.25 mm</td></tr>
      <tr><td>40×25×2 Hollow alu column</td><td>340.00 mm (224 g)</td></tr>
    </table>
  </div>

  <div class="sec">
    <div class="btn-group" style="width:100%">
      <button style="flex:1" onclick="resetCam()">Reset View</button>
      <button style="flex:1" onclick="focusLegs()">Zoom on 2 Legs</button>
    </div>
  </div>
</div>

<script type="module">
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';

const BAY_P = 93 * Math.PI * 1.5; // 438.25 mm
const JZ_LO = 90.0;
const JZ_UP = JZ_LO + 51 * Math.PI * 1.5; // 330.33 mm

const wrap = document.getElementById('canvas-wrap');
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x10151f);

const camera = new THREE.PerspectiveCamera(42, window.innerWidth / window.innerHeight, 1, 6000);
camera.position.set(520, 420, 650);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(window.innerWidth, window.innerHeight);
wrap.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.target.set(160, 260, 0);
controls.update();

scene.add(new THREE.HemisphereLight(0xdbeafe, 0x1e293b, 1.35));
const key = new THREE.DirectionalLight(0xffffff, 1.5);
key.position.set(400, 700, 500);
scene.add(key);

const grid = new THREE.GridHelper(1800, 36, 0x334155, 0x1e293b);
grid.position.set(220, 0, -50);
scene.add(grid);

const worldRoot = new THREE.Group();
worldRoot.rotation.x = -Math.PI / 2;
scene.add(worldRoot);

const loader = new STLLoader();
const podMat = (hex, extra={}) => new THREE.MeshStandardMaterial({ color: hex, roughness: 0.42, ...extra });

const mats = {
  body_mount:   podMat(0x33b873),
  bearing_bot:  podMat(0xd8dee6, { metalness: 0.85, roughness: 0.25 }),
  bearing_top:  podMat(0xd8dee6, { metalness: 0.85, roughness: 0.25 }),
  bearing_pinion: podMat(0xd8dee6, { metalness: 0.85, roughness: 0.25 }),
  m8_hardware:  podMat(0x64748b, { metalness: 0.80, roughness: 0.30 }),
  servo_body:   podMat(0xc41e24, { metalness: 0.50, roughness: 0.35 }),
  servo_spline: podMat(0xd4ad52, { metalness: 0.85, roughness: 0.25 }),
  alum_horn:    podMat(0xe04040, { metalness: 0.75, roughness: 0.30 }),
  gear_20t:     podMat(0xf59e0b),
  carriage_pod: podMat(0x2e8ee6),
  motor_housing:podMat(0xb4bcc8, { metalness: 0.65, roughness: 0.35 }),
  motor_shaft:  podMat(0xd8dee6, { metalness: 0.85, roughness: 0.20 }),
  pinion:       podMat(0xeb7a24),
  retainer:     podMat(0x333942),
  gantry_plate: podMat(0x22262d, { metalness: 0.50 }),
  v_wheels:     podMat(0x3c414a),
  pinion_cover: podMat(0x8fd0ff),
  motor_cap:    podMat(0x1b4a99),
  screw_pinion:    podMat(0x9aa5b4, { metalness: 0.90, roughness: 0.25 }),
  screws_pcover:   podMat(0x9aa5b4, { metalness: 0.90, roughness: 0.25 }),
  screws_cap:      podMat(0x9aa5b4, { metalness: 0.90, roughness: 0.25 }),
  fasteners_turn:  podMat(0x9aa5b4, { metalness: 0.90, roughness: 0.25 }),
  fasteners_fixed: podMat(0x9aa5b4, { metalness: 0.90, roughness: 0.25 }),
  fasteners_gear:  podMat(0x9aa5b4, { metalness: 0.90, roughness: 0.25 }),
  // Crossing + environment
  xp_rail:          podMat(0xe2e8f0, { metalness: 0.70, roughness: 0.30 }),
  xp_rack:          podMat(0xf59e0b),
  xp_rotor:         podMat(0xa8b2c0, { metalness: 0.70, roughness: 0.35 }),
  xp_pivot_608zz:   podMat(0xd8dee6, { metalness: 0.85, roughness: 0.25 }),
  xp_pivot_m8_stack:podMat(0x64748b, { metalness: 0.80, roughness: 0.30 }),
  xp_rotor_screws:  podMat(0x64748b, { metalness: 0.80, roughness: 0.30 }),
  xp_tnuts:         podMat(0x9aa5b4, { metalness: 0.90, roughness: 0.25 }),
  xp_stop_pin:      podMat(0x64748b, { metalness: 0.80, roughness: 0.30 }),
  column_bar:       podMat(0xcbd5e1, { metalness: 0.70, roughness: 0.30, transparent: true, opacity: 0.45, depthWrite: false }),
  anti_crush:       podMat(0xf59e0b),
  rails_vert:       podMat(0x94a3b8, { metalness: 0.70, roughness: 0.30 }),
  rails_horiz:      podMat(0x94a3b8, { metalness: 0.70, roughness: 0.30 }),
  racks_vert:       podMat(0xd97706),
  racks_horiz:      podMat(0xd97706),
  crossings_fixed:  podMat(0x7c8a9c, { metalness: 0.60 }),
  rack_bays:        podMat(0x64748b, { transparent: true, opacity: 0.22, depthWrite: false }),
  sensors_pod_fixed:podMat(0x06b6d4, { roughness: 0.30 }),
  sensors_pod_turn: podMat(0x06b6d4, { roughness: 0.30 }),
  sensors_hardware: podMat(0xcbd5e1, { metalness: 0.90, roughness: 0.22 }),
  crossing_magnets: podMat(0xe11d48, { metalness: 0.70, roughness: 0.25 }),
};

const envGroup = new THREE.Group();
worldRoot.add(envGroup);
const railsGroup = new THREE.Group();
const crossFixedGroup = new THREE.Group();
const racksRefGroup = new THREE.Group();
const sensorsEnvGroup = new THREE.Group();
envGroup.add(railsGroup, crossFixedGroup, racksRefGroup, sensorsEnvGroup);

const geomCache = {};
function loadStl(url, onLoad) {
  if (geomCache[url]) { onLoad(geomCache[url]); return; }
  loader.load(url, (g) => {
    g.computeVertexNormals();
    geomCache[url] = g;
    onLoad(g);
  });
}
function addMesh(url, parent, mat) {
  const holder = new THREE.Group();
  parent.add(holder);
  loadStl(url, (g) => holder.add(new THREE.Mesh(g, mat)));
  return holder;
}

addMesh('/meshes_env/rails_vert.stl', railsGroup, mats.rails_vert);
addMesh('/meshes_env/racks_vert.stl', railsGroup, mats.racks_vert);
addMesh('/meshes_env/rails_horiz.stl', railsGroup, mats.rails_horiz);
addMesh('/meshes_env/racks_horiz.stl', railsGroup, mats.racks_horiz);
addMesh('/meshes_env/crossings_fixed.stl', crossFixedGroup, mats.crossings_fixed);
addMesh('/meshes_env/rack_bays.stl', racksRefGroup, mats.rack_bays);
addMesh('/meshes_env/crossing_magnets.stl', sensorsEnvGroup, mats.crossing_magnets);

// POD_TO_HUB matrix: x_h = -y_p, y_h = -62.475 - z_p, z_h = x_p
function applyPodToHub(grp) {
  grp.matrixAutoUpdate = false;
  grp.matrix.set(
    0, -1,  0,  0,
    0,  0, -1, -62.475,
    1,  0,  0,  0,
    0,  0,  0,  1
  );
}

// 4x Crossing turntables (at (0, JZ_LO), (0, JZ_UP), (BAY_P, JZ_LO), (BAY_P, JZ_UP))
const crossTurnGroup = new THREE.Group();
worldRoot.add(crossTurnGroup);
const crossingTurns = {}; // key: "b0_lo", "b0_up", "b1_lo", "b1_up"
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

// Robot moving assembly: 30x25x2 hollow column bar + anti-crush inserts + Upper Leg (JZ_UP) + Lower Leg (JZ_LO)
const robotGroup = new THREE.Group();
worldRoot.add(robotGroup);
const colGroup = addMesh('/meshes_env/column_bar.stl', robotGroup, mats.column_bar);
const plugGroup = addMesh('/meshes_env/anti_crush_inserts.stl', robotGroup, mats.anti_crush);

const legMeshes = [];
const sensorPodHolders = [];
function createHubLeg(zc, mirror) {
  const hubPos = new THREE.Group();
  hubPos.position.set(0, 0, zc);
  // lower pod mirrored (as v3: HUB_BASE lower = 180 deg about the hub's y axis): its pinion sits on the other side of the hub,
  // so the two pinions never cross a 1 mm crossing cut at the same time (worst case the other is 32 mm into solid rack)
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
  sensorPodHolders.push(addMesh('/meshes_env/sensors_pod_fixed.stl', fixedG, mats.sensors_pod_fixed));
  sensorPodHolders.push(addMesh('/meshes_env/sensors_pod_turn.stl', carriagePivot, mats.sensors_pod_turn));
  sensorPodHolders.push(addMesh('/meshes_env/sensors_hardware.stl', carriagePivot, mats.sensors_hardware));
  legMeshes.push(hubPos);
  return { carriagePivot, servoPivot, climbPivot };
}

const lowerHub = createHubLeg(JZ_LO, true);
const upperHub = createHubLeg(JZ_UP, false);

// ---- motion planner: the robot's real rules, so nothing can jump ----------------------------------------------------
//  * climb (z) only at a bay with BOTH hubs at 0 deg;  travel (x) only at crossing height with BOTH hubs at 90 deg
//  * the hubs swivel ONE AT A TIME (lower first), each together with the crossing piece it sits on; every crossing piece
//    keeps its angle when the robot leaves it
//  * every move is speed-limited and eases in (no snaps), in auto mode and when dragging a slider
let mode = 'auto';
let poseX = 0.0, poseZ = 0.0, swL = 0.0, swU = 0.0;          // current pose
let tx = 0.0, tz = 0.0, tsw = 0.0;                            // target
const BAYS = [0.0, BAY_P];
const crossAng = { b0_lo: 0, b0_up: 0, b1_lo: 90, b1_up: 90 };
const SPD = { x: 160, z: 160, sw: 70 };                       // mm/s, mm/s, deg/s
const nearestBay = (x) => BAYS.reduce((a, b) => Math.abs(b - x) < Math.abs(a - x) ? b : a);
const approach = (v, goal, maxStep) => {                      // speed-limited, slows down over the last part (ease-in)
  const d = goal - v;
  const step = Math.sign(d) * Math.min(Math.abs(d), maxStep, Math.max(Math.abs(d) * 0.25, maxStep * 0.15));
  return Math.abs(d) < 1e-3 ? goal : v + step;
};
function stepPlanner(dt) {
  const eps = 0.05, sx = SPD.x * dt, sz = SPD.z * dt, ss = SPD.sw * dt;
  if (Math.abs(poseX - tx) > eps) {                           // travel: down to crossing height, both hubs to 90, then x
    if (poseZ > eps) poseZ = approach(poseZ, 0, sz);
    else if (swL < 90 - eps) swL = approach(swL, 90, ss);
    else if (swU < 90 - eps) swU = approach(swU, 90, ss);
    else poseX = approach(poseX, tx, sx);
    return;
  }
  const atBay = BAYS.some(b => Math.abs(poseX - b) < 0.5);
  const wantZ = atBay ? tz : 0;
  const wantSw = wantZ > eps ? 0 : (atBay ? tsw : 90);
  if (wantZ > eps || poseZ > eps) {                           // climbing: both hubs vertical first (lower first)
    if (swL > eps) swL = approach(swL, 0, ss);
    else if (swU > eps) swU = approach(swU, 0, ss);
    else poseZ = approach(poseZ, wantZ, sz);
    return;
  }
  if (Math.abs(swL - wantSw) > eps) swL = approach(swL, wantSw, ss);   // at crossing height: lower hub, then upper
  else if (Math.abs(swU - wantSw) > eps) swU = approach(swU, wantSw, ss);
}
const reached = () => Math.abs(poseX - tx) < 0.05 && Math.abs(poseZ - (BAYS.some(b => Math.abs(tx - b) < 0.5) ? tz : 0)) < 0.05
  && Math.abs(swL - (tz > 0.05 ? 0 : tsw)) < 0.05 && Math.abs(swU - (tz > 0.05 ? 0 : tsw)) < 0.05;
// auto cycle: climb bay 0 -> back to the crossings -> to bay 1 (swivels on the way) -> climb bay 1 -> back -> home
const AUTO = [[0, 340, 0], [0, 0, 0], [BAY_P, 0, 0], [BAY_P, 220, 0], [BAY_P, 0, 0], [0, 0, 0]];
let wp = 0, dwell = 0;

const setActive = (m) => ['auto', 'vert', 'horiz'].forEach(id => document.getElementById('m-' + id).classList.toggle('active', id === m));
window.setMode = (m) => {
  mode = m === 'auto' ? 'auto' : 'manual';
  setActive(m);
  if (m === 'auto') { wp = 0; dwell = 0; }
  if (m === 'vert') { tx = nearestBay(poseX); tz = 0; tsw = 0; }
  if (m === 'horiz') { tz = 0; tsw = 90; }
};
window.setPose = (axis, val) => {
  mode = 'manual';
  setActive('');
  if (axis === 'x') { tx = val; tz = 0; tsw = 90; }
  if (axis === 'z') { tx = nearestBay(poseX); tz = val; tsw = 0; }
  if (axis === 'sw') { tx = nearestBay(poseX); tz = 0; tsw = val; }
};
window.setVis = (name, state) => {
  if (name === 'legs') {
    legMeshes.forEach(g => g.visible = (state !== 'off'));
    for (const [k, m] of Object.entries(mats)) {
      if (['column_bar','rails_vert','rails_horiz','racks_vert','racks_horiz','crossings_fixed','rack_bays','sensors_pod_fixed','sensors_pod_turn','sensors_hardware','crossing_magnets'].includes(k) || k.startsWith('xp_')) continue;
      m.transparent = (state === 'xray');
      m.opacity = (state === 'xray') ? 0.35 : 1.0;
      m.depthWrite = (state !== 'xray');
    }
  } else if (name === 'col') {
    colGroup.visible = (state !== 'off');
    plugGroup.visible = (state !== 'off');
    mats.column_bar.transparent = (state === 'xray');
    mats.column_bar.opacity = (state === 'xray') ? 0.32 : 1.0;
    mats.column_bar.depthWrite = (state !== 'xray');
  }
  else if (name === 'rails') railsGroup.visible = (state !== 'off');
  else if (name === 'cross') { crossFixedGroup.visible = (state !== 'off'); crossTurnGroup.visible = (state !== 'off'); }
  else if (name === 'racks') {
    racksRefGroup.visible = (state !== 'off');
    mats.rack_bays.opacity = (state === 'xray') ? 0.22 : 0.85;
    mats.rack_bays.transparent = (state === 'xray');
    mats.rack_bays.depthWrite = (state !== 'xray');
  }
  else if (name === 'sens') {
    sensorsEnvGroup.visible = (state !== 'off');
    sensorPodHolders.forEach(g => g.visible = (state !== 'off'));
  }
  ['solid', 'xray', 'off'].forEach(s => {
    const el = document.getElementById(`v-${name}-${s}`);
    if (el) el.classList.toggle('active', s === state);
  });
};
window.resetCam = () => {
  camera.position.set(520, 420, 650);
  controls.target.set(160, 260, 0);
  controls.update();
};
window.focusLegs = () => {
  camera.position.set(poseX - 240, 220 + poseZ, 340);
  controls.target.set(poseX, 210 + poseZ, -60);
  controls.update();
};
window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});

const clock = new THREE.Clock();
let t = 0;
let animSpeed = 1.0;

window.setSpeed = (v) => {
  animSpeed = v;
  document.getElementById('spd-val').textContent = v.toFixed(1) + '×';
};

function animate() {
  requestAnimationFrame(animate);
  const dt = Math.min(clock.getDelta(), 0.05);
  t += dt * animSpeed;

  if (mode === 'auto') {
    [tx, tz, tsw] = AUTO[wp];
    if (reached()) { dwell += dt * animSpeed; if (dwell > 0.6) { wp = (wp + 1) % AUTO.length; dwell = 0; } }
  }
  stepPlanner(dt * animSpeed);

  robotGroup.position.set(poseX, 0, poseZ);
  // each hub swivels with its own angle (lower hub mirrored: same pod-frame rotation, see createHubLeg)
  lowerHub.carriagePivot.rotation.z = swL * Math.PI / 180;
  upperHub.carriagePivot.rotation.z = swU * Math.PI / 180;
  lowerHub.servoPivot.rotation.z = -2.0 * swL * Math.PI / 180;
  upperHub.servoPivot.rotation.z = -2.0 * swU * Math.PI / 180;

  // a crossing piece turns only with the hub sitting on it, and keeps its angle when the robot leaves
  BAYS.forEach((bx, i) => {
    if (Math.abs(poseX - bx) < 0.5 && poseZ < 0.05) { crossAng[`b${i}_lo`] = swL; crossAng[`b${i}_up`] = swU; }
  });
  for (const [k, g] of Object.entries(crossingTurns)) g.rotation.z = crossAng[k] * Math.PI / 180;

  // pinions roll with the distance travelled (pitch radius 10.5 mm); the mirrored lower pinion turns the other way
  const climbAng = (poseZ + poseX) / 10.5;
  upperHub.climbPivot.rotation.y = -climbAng;
  lowerHub.climbPivot.rotation.y = climbAng;

  if (mode === 'auto') {                                      // (in manual mode the sliders show the target, not fought)
    document.getElementById('x-sl').value = poseX;
    document.getElementById('z-sl').value = poseZ;
    document.getElementById('sw-sl').value = (swL + swU) / 2;
  }
  document.getElementById('x-val').textContent = poseX.toFixed(1) + ' mm';
  document.getElementById('z-val').textContent = poseZ.toFixed(1) + ' mm';
  document.getElementById('sw-val').textContent = `L ${swL.toFixed(0)}° / U ${swU.toFixed(0)}°`;

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

        if self.path.startswith("/meshes_pod/") or self.path.startswith("/meshes_env/"):
            base = POD_MESH_DIR if self.path.startswith("/meshes_pod/") else ENV_MESH_DIR
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
    for port in range(8845, 8860):
        try:
            srv = http.server.ThreadingHTTPServer(("127.0.0.1", port), Handler)
            break
        except OSError:
            continue
    else:
        raise RuntimeError("No free port in 8845..8859")

    url = f"http://127.0.0.1:{port}/"
    print(f"Option 1 Dual-Leg + Complete Railing Studio running at {url}  (Ctrl+C to stop)")
    threading.Timer(0.4, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        srv.shutdown()


if __name__ == "__main__":
    main()
