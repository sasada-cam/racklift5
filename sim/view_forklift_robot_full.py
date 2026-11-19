"""Interactive Three.js 3D Studio for Full Integrated Option 1 Robot + Embedded Forklift.

Combines:
  - Dual Option 1 swivel legs (H1 @ Z=330.33, H2 mirrored @ Z=90.0) + P08/P09 Hall sensors + magnets
  - 40 x 25 x 2.0 mm hollow aluminium column + 3-zone internal anti-crush plugs (H2, BODY, H1)
  - Complete 2-bay v3 2020 railing set, 4x 17.5P crossing turntables, and 5-shelf slotted-angle racks
  - R40 closed-box body collar + R42/R43 battery pod + R41 boom + TAL220 + R45 wishbone + R44 tray
  - Embedded CNC4759-A 2-Stage Forklift Compartment (C01 + F01-F04 + NEMA 17 + Skidded Load)
  - Live Auto Cycle: Bay 0 Pick (+240 mm reach, +10 mm leg Z-lift) -> 90 deg Swivel -> Bay 1 Place!

Usage:
  .pixi/envs/default/bin/python sim/view_forklift_robot_full.py
"""

import http.server
import os
import socketserver
import threading
import webbrowser

HERE = os.path.dirname(os.path.abspath(__file__))
POD_MESH_DIR = os.path.join(HERE, "build", "meshes_t_slot_option1")
FULL_MESH_DIR = os.path.join(HERE, "build", "meshes_forklift_robot_full")

HTML = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>RackLift5 — Full Robot + Embedded CNC4759-A Forklift</title>
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
  <h1>RackLift5 · Full Robot + Forklift</h1>
  <div class="sub">Dual Swivel Legs + Embedded CNC4759-A 2-Stage Forklift + 2-Bay Racks</div>

  <div class="sec">
    <div class="sec-title">1. Robot &amp; Forklift Cycle</div>
    <div id="phase-lbl" style="font-size:11px;color:#38bdf8;font-weight:600;margin-bottom:6px;min-height:15px">Auto Loop · Initializing…</div>
    <div class="btn-group" style="width:100%">
      <button id="m-auto" class="active ok" style="flex:1.3" onclick="setMode('auto')">Auto Full Loop</button>
      <button id="m-vert" style="flex:1" onclick="setMode('vert')">0° Home</button>
      <button id="m-horiz" style="flex:1" onclick="setMode('horiz')">90° Travel</button>
      <button id="m-reach" style="flex:1" onclick="setMode('reach')">Fork +240</button>
    </div>
    <div class="btn-group" style="width:100%; margin-top:5px">
      <button style="flex:1" onclick="goShelf(0)">Shelf 1 (0)</button>
      <button style="flex:1" onclick="goShelf(173)">Shelf 2 (+173)</button>
      <button style="flex:1" onclick="goShelf(346)">Shelf 3 (+346)</button>
      <button style="flex:1" onclick="goShelf(519)">Shelf 4 (+519)</button>
    </div>
    <div class="row" style="margin-top:8px">
      <span>Bay Travel X (0 .. 358.14 mm)</span><span id="x-val">0.0 mm</span>
    </div>
    <input id="x-sl" type="range" min="0" max="358.14" step="1" value="0" oninput="setPose('x', Number(this.value))"/>
    <div class="row">
      <span>Climb / Leg Z-Lift dZ (0 .. 535 mm)</span><span id="z-val">0.0 mm</span>
    </div>
    <input id="z-sl" type="range" min="0" max="535" step="1" value="0" oninput="setPose('z', Number(this.value))"/>
    <div class="row">
      <span>Legs Swivel (0° .. 90°)</span><span id="sw-val">L 0° / U 0°</span>
    </div>
    <input id="sw-sl" type="range" min="0" max="90" step="1" value="0" oninput="setPose('sw', Number(this.value))"/>
    <div class="row">
      <span>Forklift Tines Y (0 .. 240 mm)</span><span id="arm-val">0.0 mm</span>
    </div>
    <input id="arm-sl" type="range" min="0" max="240" step="1" value="0" oninput="setPose('arm', Number(this.value))"/>
    <div class="row">
      <span>Animation Speed (0× Pause .. 3×)</span><span id="spd-val">1.0×</span>
    </div>
    <input id="spd-sl" type="range" min="0" max="3" step="0.1" value="1" oninput="setSpeed(Number(this.value))"/>
  </div>

  <div class="sec">
    <div class="sec-title">2. Exploded Assembly View</div>
    <div class="row"><span>Exploded View</span><span id="exp-val">0%</span></div>
    <input id="exp-sl" type="range" min="0" max="100" step="1" value="0" oninput="setExplode(Number(this.value))"/>
    <div class="btn-group" style="width:100%; margin-top:4px">
      <button id="e-0" class="active" style="flex:1" onclick="setExplode(0)">Assembled (0%)</button>
      <button id="e-50" style="flex:1" onclick="setExplode(50)">Half (50%)</button>
      <button id="e-100" style="flex:1" onclick="setExplode(100)">Exploded (100%)</button>
    </div>
  </div>

  <div class="sec">
    <div class="sec-title">3. Layers</div>
    <div class="row"><span><span class="dot" style="background:#f97316"></span>R46–R57 Forklift + MGN Rails + 1:2 Rack-Pinion</span>
      <div class="btn-group"><button id="v-fork-solid" class="active" onclick="setVis('fork','solid')">Show</button><button id="v-fork-off" onclick="setVis('fork','off')">Hide</button></div>
    </div>
    <div class="row"><span><span class="dot" style="background:#0e7490"></span>R56 Screwed Top Motor &amp; Tail Cover</span>
      <div class="btn-group"><button id="v-cover-solid" class="active" onclick="setVis('cover','solid')">Solid</button><button id="v-cover-xray" onclick="setVis('cover','xray')">Ghost</button><button id="v-cover-off" onclick="setVis('cover','off')">Hide</button></div>
    </div>
    <div class="row"><span><span class="dot" style="background:#0284c7"></span>R58 Center-Bay Overhead Shroud</span>
      <div class="btn-group"><button id="v-baycov-solid" class="active" onclick="setVis('baycov','solid')">Solid</button><button id="v-baycov-xray" onclick="setVis('baycov','xray')">Ghost</button><button id="v-baycov-off" onclick="setVis('baycov','off')">Hide</button></div>
    </div>
    <div class="row"><span><span class="dot" style="background:#6366f1"></span>R57 Screwed MGN/T-Slot Hold-Down Rails</span>
      <div class="btn-group"><button id="v-hold-solid" class="active" onclick="setVis('hold','solid')">Solid</button><button id="v-hold-xray" onclick="setVis('hold','xray')">Ghost</button><button id="v-hold-off" onclick="setVis('hold','off')">Hide</button></div>
    </div>
    <div class="row"><span><span class="dot" style="background:#16a34a"></span>2x KW4-3Z-3 Limit Switches</span>
      <div class="btn-group"><button id="v-sw-solid" class="active" onclick="setVis('sw','solid')">Show</button><button id="v-sw-off" onclick="setVis('sw','off')">Hide</button></div>
    </div>
    <div class="row"><span><span class="dot" style="background:#4caf50"></span>B04 Tray + Boom + Wishbone + Load Cell</span>
      <div class="btn-group"><button id="v-tray-solid" class="active" onclick="setVis('tray','solid')">Solid</button><button id="v-tray-xray" onclick="setVis('tray','xray')">Ghost</button><button id="v-tray-off" onclick="setVis('tray','off')">Hide</button></div>
    </div>
    <div class="row"><span><span class="dot" style="background:#2e8ee6"></span>Dual Swivel Legs (H1, H2)</span>
      <div class="btn-group"><button id="v-legs-solid" class="active" onclick="setVis('legs','solid')">Solid</button><button id="v-legs-xray" onclick="setVis('legs','xray')">Ghost</button><button id="v-legs-off" onclick="setVis('legs','off')">Hide</button></div>
    </div>
    <div class="row"><span><span class="dot" style="background:#a855f7"></span>Leg Hoods + Column + B01/B03</span>
      <div class="btn-group"><button id="v-body-solid" class="active" onclick="setVis('body','solid')">Show</button><button id="v-body-off" onclick="setVis('body','off')">Hide</button></div>
    </div>
    <div class="row"><span><span class="dot" style="background:#94a3b8"></span>2020 Rails + Crossings</span>
      <div class="btn-group"><button id="v-rails-solid" class="active" onclick="setVis('rails','solid')">Show</button><button id="v-rails-off" onclick="setVis('rails','off')">Hide</button></div>
    </div>
    <div class="row"><span><span class="dot" style="background:#64748b"></span>Ref 2-Bay Racks + Skidded Load</span>
      <div class="btn-group"><button id="v-racks-solid" onclick="setVis('racks','solid')">Solid</button><button id="v-racks-xray" class="active" onclick="setVis('racks','xray')">Ghost</button><button id="v-racks-off" onclick="setVis('racks','off')">Hide</button></div>
    </div>
  </div>

  <div class="sec">
    <div class="btn-group" style="width:100%">
      <button style="flex:1" onclick="resetCam()">Reset View</button>
      <button style="flex:1" onclick="focusForklift()">Zoom Forklift</button>
      <button style="flex:1" onclick="focusLegs()">Zoom Legs</button>
    </div>
  </div>
</div>

<script type="module">
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';

const BAY_P = 76 * Math.PI * 1.5; // 358.14 mm
const JZ_LO = 90.0;
const JZ_UP = JZ_LO + 51 * Math.PI * 1.5; // 330.33 mm

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
  body_mount: mat(0x33b873), bearing_bot: mat(0xd8dee6, { metalness: 0.85, roughness: 0.25 }),
  bearing_top: mat(0xd8dee6, { metalness: 0.85, roughness: 0.25 }), bearing_pinion: mat(0xd8dee6, { metalness: 0.85, roughness: 0.25 }),
  m8_hardware: mat(0x64748b, { metalness: 0.80, roughness: 0.30 }), servo_body: mat(0xc41e24, { metalness: 0.50, roughness: 0.35 }),
  servo_spline: mat(0xd4ad52, { metalness: 0.85, roughness: 0.25 }), alum_horn: mat(0xe04040, { metalness: 0.75, roughness: 0.30 }),
  gear_20t: mat(0xf59e0b), carriage_pod: mat(0x2e8ee6), motor_housing: mat(0xb4bcc8, { metalness: 0.65, roughness: 0.35 }),
  motor_shaft: mat(0xd8dee6, { metalness: 0.85, roughness: 0.20 }), pinion: mat(0xeb7a24), retainer: mat(0x333942),
  gantry_plate: mat(0x22262d, { metalness: 0.50 }), v_wheels: mat(0x3c414a), pinion_cover: mat(0x8fd0ff), motor_cap: mat(0x1b4a99),
  screw_pinion: mat(0x9aa5b4, { metalness: 0.90 }), screws_pcover: mat(0x9aa5b4, { metalness: 0.90 }), screws_cap: mat(0x9aa5b4, { metalness: 0.90 }),
  fasteners_turn: mat(0x9aa5b4, { metalness: 0.90 }), fasteners_fixed: mat(0x9aa5b4, { metalness: 0.90 }), fasteners_gear: mat(0x9aa5b4, { metalness: 0.90 }),
  xp_rail: mat(0xe2e8f0, { metalness: 0.70 }), xp_rack: mat(0xf59e0b), xp_rotor: mat(0xa8b2c0, { metalness: 0.70 }),
  xp_pivot_608zz: mat(0xd8dee6, { metalness: 0.85 }), xp_pivot_m8_stack: mat(0x64748b, { metalness: 0.80 }),
  xp_rotor_screws: mat(0x64748b, { metalness: 0.80 }), xp_tnuts: mat(0x9aa5b4, { metalness: 0.90 }), xp_stop_pin: mat(0x64748b, { metalness: 0.80 }),
  column_bar: mat(0xcbd5e1, { metalness: 0.70, transparent: true, opacity: 0.45, depthWrite: false }), anti_crush: mat(0xf59e0b),
  rails_vert: mat(0x94a3b8, { metalness: 0.70 }), rails_horiz: mat(0x94a3b8, { metalness: 0.70 }),
  racks_vert: mat(0xd97706), racks_horiz: mat(0xd97706), crossings_fixed: mat(0x7c8a9c, { metalness: 0.60 }),
  rack_bays: mat(0x64748b, { transparent: true, opacity: 0.22, depthWrite: false }),
  rack_mounts: mat(0xfb7185, { metalness: 0.55 }), rack_mount_hw: mat(0xcbd5e1, { metalness: 0.90 }),
  sensors_pod_fixed: mat(0x06b6d4), sensors_pod_turn: mat(0x06b6d4), sensors_hardware: mat(0xcbd5e1, { metalness: 0.90 }),
  leg_hood_upper: mat(0xa855f7, { transparent: true, opacity: 0.38, depthWrite: false }),
  leg_hood_lower: mat(0x9333ea, { transparent: true, opacity: 0.38, depthWrite: false }),
  crossing_magnets: mat(0xe11d48, { metalness: 0.70 }),
  body_collar: mat(0x1f7a4d), body_fasteners: mat(0x334155, { metalness: 0.80 }),
  tray_boom: mat(0x2563eb, { metalness: 0.25 }), live_wishbone: mat(0x1d4ed8), load_cell: mat(0xe2e8f0, { metalness: 0.80 }),
  tray_deck: mat(0x0ea5e9), battery_pod: mat(0x66bb6a), battery_lid: mat(0x388e3c), battery_pack: mat(0x334155),
  compartment_deck: mat(0x26a69a), fork_under_tie: mat(0x2e7d32), fork_top_cover: mat(0x0e7490),
  fork_cover_fasteners: mat(0x334155, { metalness: 0.78 }),
  fork_bay_cover: mat(0x0284c7), fork_bay_cover_fasteners: mat(0x334155, { metalness: 0.78 }), fork_limit_switches: mat(0x16a34a),
  fork_hold_rails: mat(0x6366f1), fork_hold_fasteners: mat(0x334155, { metalness: 0.80 }),
  fork_base_rails: mat(0x00897b, { metalness: 0.25 }),
  fork_front_arch: mat(0x80cbc4), fork_mid_stage: mat(0x43a047, { metalness: 0.25 }),
  fork_pinion_left: mat(0xeab308, { metalness: 0.25 }), fork_pinion_right: mat(0xca8a04, { metalness: 0.25 }),
  fork_t8_nut: mat(0xd97706, { metalness: 0.82, roughness: 0.22 }),
  fork_mid_fasteners: mat(0x334155, { metalness: 0.78 }), fork_tines: mat(0xf97316, { metalness: 0.30 }),
  fork_drive: mat(0xe2e8f0, { metalness: 0.68 }), ref_load: mat(0xeab308, { transparent: true, opacity: 0.36, depthWrite: false }),
};

const geomCache = {};
function addMesh(url, parent, material) {
  const h = new THREE.Group();
  parent.add(h);
  if (geomCache[url]) { h.add(new THREE.Mesh(geomCache[url], material)); return h; }
  loader.load(url, (g) => { geomCache[url] = g; h.add(new THREE.Mesh(g, material)); });
  return h;
}

const railsGroup = new THREE.Group(), crossFixedGroup = new THREE.Group(), racksRefGroup = new THREE.Group();
const rackMountsGroup = new THREE.Group(), sensorsEnvGroup = new THREE.Group();
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
  grp.matrix.set(0, -1, 0, 0, 0, 0, -1, -62.475, 1, 0, 0, 0, 0, 0, 0, 1);
}

const crossTurnGroup = new THREE.Group();
worldRoot.add(crossTurnGroup);
const crossingTurns = {};
for (const [bIdx, bx] of [[0, 0], [1, BAY_P]]) {
  for (const [tag, zc] of [['lo', JZ_LO], ['up', JZ_UP]]) {
    const outer = new THREE.Group(); outer.position.set(bx, 0, zc); crossTurnGroup.add(outer);
    const podMap = new THREE.Group(); applyPodToHub(podMap); outer.add(podMap);
    const rot = new THREE.Group(); podMap.add(rot);
    for (const k of ['xp_rail', 'xp_rack', 'xp_rotor', 'xp_pivot_608zz', 'xp_pivot_m8_stack', 'xp_rotor_screws', 'xp_stop_pin', 'xp_tnuts']) {
      addMesh('/meshes_pod/' + k + '.stl', rot, mats[k]);
    }
    crossingTurns[`b${bIdx}_${tag}`] = rot;
  }
}

const robotGroup = new THREE.Group();
worldRoot.add(robotGroup);
const colGroup = addMesh('/meshes_full/column_bar.stl', robotGroup, mats.column_bar);
const plugGroup = addMesh('/meshes_full/anti_crush_inserts.stl', robotGroup, mats.anti_crush);

const legMeshes = [], sensorPodHolders = [];
const hoodUpperHolder = addMesh('/meshes_full/leg_hood_upper.stl', robotGroup, mats.leg_hood_upper);
const hoodLowerHolder = addMesh('/meshes_full/leg_hood_lower.stl', robotGroup, mats.leg_hood_lower);

function createHubLeg(zc, mirror) {
  const hubPos = new THREE.Group(); hubPos.position.set(0, 0, zc); if (mirror) hubPos.rotation.y = Math.PI;
  robotGroup.add(hubPos);
  const podMap = new THREE.Group(); applyPodToHub(podMap); hubPos.add(podMap);
  const fixedG = new THREE.Group(), servoPivot = new THREE.Group(); servoPivot.position.set(-37.5, 0, 0);
  const carriagePivot = new THREE.Group(), climbPivot = new THREE.Group(); climbPivot.position.set(52.5, 0, 5.325);
  carriagePivot.add(climbPivot); podMap.add(fixedG, servoPivot, carriagePivot);
  for (const k of ['body_mount', 'bearing_bot', 'bearing_top', 'm8_hardware', 'servo_body', 'fasteners_fixed']) addMesh('/meshes_pod/' + k + '.stl', fixedG, mats[k]);
  for (const k of ['gear_20t', 'alum_horn', 'servo_spline', 'fasteners_gear']) addMesh('/meshes_pod/' + k + '.stl', servoPivot, mats[k]);
  for (const k of ['carriage_pod', 'motor_cap', 'pinion_cover', 'bearing_pinion', 'motor_housing', 'gantry_plate', 'v_wheels', 'fasteners_turn', 'screws_pcover', 'screws_cap']) addMesh('/meshes_pod/' + k + '.stl', carriagePivot, mats[k]);
  for (const k of ['motor_shaft', 'pinion', 'retainer', 'screw_pinion']) addMesh('/meshes_pod/' + k + '.stl', climbPivot, mats[k]);
  sensorPodHolders.push(addMesh('/meshes_full/sensors_pod_fixed.stl', fixedG, mats.sensors_pod_fixed));
  sensorPodHolders.push(addMesh('/meshes_full/sensors_pod_turn.stl', carriagePivot, mats.sensors_pod_turn));
  sensorPodHolders.push(addMesh('/meshes_full/sensors_hardware.stl', carriagePivot, mats.sensors_hardware));
  legMeshes.push(hubPos);
  return { hubPos, carriagePivot, servoPivot, climbPivot, zc };
}

const lowerHub = createHubLeg(JZ_LO, true);
const upperHub = createHubLeg(JZ_UP, false);

const collarGroup = new THREE.Group(), trayGroup = new THREE.Group(), forkGroup = new THREE.Group(), batGroup = new THREE.Group();
robotGroup.add(collarGroup, trayGroup, forkGroup, batGroup);

const bodyMeshes = {};
const regBody = (name, parent, material) => { const h = addMesh('/meshes_full/' + name + '.stl', parent, material); bodyMeshes[name] = h; return h; };

regBody('body_collar', collarGroup, mats.body_collar);
regBody('body_fasteners', collarGroup, mats.body_fasteners);
regBody('tray_boom', trayGroup, mats.tray_boom);
regBody('live_wishbone', trayGroup, mats.live_wishbone);
regBody('load_cell', trayGroup, mats.load_cell);
regBody('tray_deck', trayGroup, mats.tray_deck);
regBody('compartment_deck', forkGroup, mats.compartment_deck);
const fTieMesh = regBody('fork_under_tie', forkGroup, mats.fork_under_tie);
const fCoverMesh = regBody('fork_top_cover', forkGroup, mats.fork_top_cover);
const fCoverHwMesh = regBody('fork_cover_fasteners', forkGroup, mats.fork_cover_fasteners);
const fBayCovMesh = regBody('fork_bay_cover', forkGroup, mats.fork_bay_cover);
const fBayCovHwMesh = regBody('fork_bay_cover_fasteners', forkGroup, mats.fork_bay_cover_fasteners);
const fSwMesh = regBody('fork_limit_switches', forkGroup, mats.fork_limit_switches);
const fHoldMesh = regBody('fork_hold_rails', forkGroup, mats.fork_hold_rails);
const fHoldHwMesh = regBody('fork_hold_fasteners', forkGroup, mats.fork_hold_fasteners);
regBody('fork_base_rails', forkGroup, mats.fork_base_rails);
regBody('fork_front_arch', forkGroup, mats.fork_front_arch);
regBody('fork_drive', forkGroup, mats.fork_drive);
const fMidMesh = regBody('fork_mid_stage', forkGroup, mats.fork_mid_stage);
const fPinLMesh = new THREE.Group();
const fPinRMesh = new THREE.Group();
forkGroup.add(fPinLMesh, fPinRMesh);
loader.load('/meshes_full/fork_pinion_left.stl', (g) => { g.translate(-148.5, 202.0, 0); fPinLMesh.add(new THREE.Mesh(g, mats.fork_pinion_left)); });
loader.load('/meshes_full/fork_pinion_right.stl', (g) => { g.translate(-211.5, 202.0, 0); fPinRMesh.add(new THREE.Mesh(g, mats.fork_pinion_right)); });
const fNutMesh = regBody('fork_t8_nut', forkGroup, mats.fork_t8_nut);
const fFastMesh = regBody('fork_mid_fasteners', forkGroup, mats.fork_mid_fasteners);
const fTinesMesh = regBody('fork_tines', forkGroup, mats.fork_tines);
const refLoadMesh = regBody('ref_load', worldRoot, mats.ref_load);
regBody('battery_pod', batGroup, mats.battery_pod);
regBody('battery_lid', batGroup, mats.battery_lid);
regBody('battery_pack', batGroup, mats.battery_pack);

let mode = 'auto', poseX = 0, poseZ = 0, swL = 0, swU = 0, forkMm = 0;
let tx = 0, tz = 0, tsw = 0, tFork = 0;
let animSpeed = 1.0, explodeK = 0.0, carryingLoad = false, loadBayX = 0.0, loadShelfZ = 0.0;
const BAYS = [0.0, BAY_P];
const crossAng = { b0_lo: 0, b0_up: 0, b1_lo: 90, b1_up: 90 };
const SPD = { x: 185, z: 175, sw: 95, fork: 210 };
const nearestBay = (x) => BAYS.reduce((a, b) => Math.abs(b - x) < Math.abs(a - x) ? b : a);
const approach = (v, goal, maxStep) => {
  const d = goal - v;
  const step = Math.sign(d) * Math.min(Math.abs(d), maxStep, Math.max(Math.abs(d) * 0.30, maxStep * 0.18));
  return Math.abs(d) < 1e-3 ? goal : v + step;
};

function stepPlanner(dt) {
  const eps = 0.05;
  const sx = SPD.x * dt, sz = SPD.z * dt, ss = SPD.sw * dt, sf = SPD.fork * dt;
  const atBayT = BAYS.some(b => Math.abs(tx - b) < 0.5);
  const xMove = Math.abs(poseX - tx) > eps;

  // Always retract forks before swiveling, climbing long vertical distances away from shelf, or horizontal rail travel
  if (xMove && forkMm > eps) { forkMm = approach(forkMm, 0, sf); return; }
  if (Math.abs(poseX - tx) < eps && Math.abs(poseZ - tz) > 20 && forkMm > eps) {
    forkMm = approach(forkMm, 0, sf);
    return;
  }

  // Vertical climb / leg Z-lift / fork motion (only at bay, legs aligned vertical 0 deg)
  if (!xMove && (Math.abs(poseZ - tz) > eps || Math.abs(forkMm - tFork) > eps)) {
    if (swL > eps) swL = approach(swL, 0, ss);
    else if (swU > eps) swU = approach(swU, 0, ss);
    else {
      poseZ = approach(poseZ, atBayT ? tz : 0, sz);
      forkMm = approach(forkMm, atBayT ? tFork : 0, sf);
    }
    return;
  }

  // Horizontal bay-to-bay rail travel (legs must first lower to crossing dZ=0, then swivel 90 deg)
  if (xMove) {
    if (poseZ > eps) poseZ = approach(poseZ, 0, sz);
    else if (swL < 90 - eps) swL = approach(swL, 90, ss);
    else if (swU < 90 - eps) swU = approach(swU, 90, ss);
    else poseX = approach(poseX, tx, sx);
    return;
  }

  const wantSw = poseZ > eps ? 0 : (atBayT ? tsw : 90);
  if (Math.abs(swL - wantSw) > eps) swL = approach(swL, wantSw, ss);
  else if (Math.abs(swU - wantSw) > eps) swU = approach(swU, wantSw, ss);
}
const reached = () => (
  Math.abs(poseX - tx) < 0.1 && Math.abs(poseZ - tz) < 0.1 &&
  Math.abs(swL - tsw) < 0.1 && Math.abs(swU - tsw) < 0.1 && Math.abs(forkMm - tFork) < 0.1
);

//   [x, z, sw, fork, carry, activeShelfZ, restBayX, restShelfZ, label, dwellSec]
// Picks from Bay 0 Bottom (Shelf 1, dZ=0) -> horizontal rail travel Bay 0->Bay 1 -> climbs UP vertical racks to Shelf 3 (dZ=346) -> places ->
// picks from Bay 1 Shelf 3 -> climbs DOWN vertical racks -> horizontal travel Bay 1->Bay 0 -> places back on Bay 0 Bottom!
const SHELF3_Z = 346.0;
const AUTO = [
  [0,     0,            0,   0,   false, 0,        0,     0,        'Bay 0 · Shelf 1 Ready',                0.45],
  [0,     0,            0,   240, false, 0,        0,     0,        '1. Extend Forks under Load (+240 mm)', 0.30],
  [0,     10,           0,   240, true,  0,        0,     0,        '2. Leg Climbers Lift +10 mm',          0.25],
  [0,     10,           0,   0,   true,  0,        0,     0,        '3. Retract Load onto R46 Tray',        0.25],
  [0,     0,            0,   0,   true,  0,        0,     0,        '4. Lower to Crossings (dZ = 0)',       0.20],
  [BAY_P, 0,            0,   0,   true,  0,        0,     0,        '5. Swivel 90° & Travel Bay 0 → Bay 1',  0.30],
  [BAY_P, SHELF3_Z + 10,0,   0,   true,  SHELF3_Z, 0,     0,        '6. Climb Vertical Racks → Shelf 3',    0.35],
  [BAY_P, SHELF3_Z + 10,0,   240, true,  SHELF3_Z, 0,     0,        '7. Extend Forks over Shelf 3',         0.25],
  [BAY_P, SHELF3_Z,     0,   240, true,  SHELF3_Z, BAY_P, SHELF3_Z, '8. Lower Load onto Bay 1 Shelf 3',     0.25],
  [BAY_P, SHELF3_Z,     0,   0,   false, SHELF3_Z, BAY_P, SHELF3_Z, '9. Retract Empty from Shelf 3',        0.75],
  [BAY_P, SHELF3_Z,     0,   240, false, SHELF3_Z, BAY_P, SHELF3_Z, '10. Re-Extend Forks under Shelf 3',    0.25],
  [BAY_P, SHELF3_Z + 10,0,   240, true,  SHELF3_Z, BAY_P, SHELF3_Z, '11. Lift +10 mm off Shelf 3',          0.25],
  [BAY_P, SHELF3_Z + 10,0,   0,   true,  SHELF3_Z, BAY_P, SHELF3_Z, '12. Retract Load onto Tray',           0.25],
  [BAY_P, 0,            0,   0,   true,  0,        BAY_P, SHELF3_Z, '13. Climb Down Vertical Racks → dZ=0', 0.30],
  [0,     0,            0,   0,   true,  0,        BAY_P, SHELF3_Z, '14. Swivel 90° & Return Bay 1 → 0',    0.30],
  [0,     10,           0,   0,   true,  0,        0,     0,        '15. Lift +10 mm for Shelf 1 Clearance', 0.15],
  [0,     10,           0,   240, true,  0,        0,     0,        '16. Extend Forks over Bay 0 Shelf 1',  0.25],
  [0,     0,            0,   240, true,  0,        0,     0,        '17. Lower Load onto Bay 0 Shelf 1',    0.25],
  [0,     0,            0,   0,   false, 0,        0,     0,        '18. Retract Empty — Cycle Complete',   0.60],
];
let wp = 0, dwell = 0, activeShelfZ = 0.0;

const setActive = (m) => ['auto', 'vert', 'horiz', 'reach'].forEach(id => {
  const el = document.getElementById('m-' + id);
  if (el) el.classList.toggle('active', id === m);
});

window.setMode = (m) => {
  mode = m === 'auto' ? 'auto' : 'manual';
  setActive(m);
  if (m === 'auto') {
    wp = 0; dwell = 0;
    poseX = 0; poseZ = 0; swL = 0; swU = 0; forkMm = 0;
    carryingLoad = false; loadBayX = 0; loadShelfZ = 0;
    crossAng.b0_lo = 0; crossAng.b0_up = 0; crossAng.b1_lo = 90; crossAng.b1_up = 90;
  }
  if (m === 'vert') { tx = nearestBay(poseX); tz = 0; tsw = 0; tFork = 0; document.getElementById('phase-lbl').textContent = 'Manual · 0° Vertical Home'; }
  if (m === 'horiz') { tz = 0; tsw = 90; tFork = 0; document.getElementById('phase-lbl').textContent = 'Manual · 90° Horizontal Travel Ready'; }
  if (m === 'reach') { tx = nearestBay(poseX); tz = poseZ > 20 ? poseZ : 10; tsw = 0; tFork = 240; document.getElementById('phase-lbl').textContent = 'Manual · +240 mm Fork Reach'; }
};

window.goShelf = (shelfZ) => {
  mode = 'manual'; setActive('');
  tx = nearestBay(poseX); tz = shelfZ; tsw = 0;
  document.getElementById('phase-lbl').textContent = `Manual · Climbing to Shelf dZ = +${shelfZ} mm`;
};

window.setPose = (axis, val) => {
  mode = 'manual'; setActive('');
  document.getElementById('phase-lbl').textContent = 'Manual Control';
  if (axis === 'x') { tx = val; tz = 0; tsw = 90; }
  if (axis === 'z') { tx = nearestBay(poseX); tz = val; tsw = 0; }
  if (axis === 'sw') { tx = nearestBay(poseX); tz = 0; tsw = val; }
  if (axis === 'arm') { tFork = val; }
};

window.setSpeed = (v) => { animSpeed = v; document.getElementById('spd-val').textContent = v.toFixed(1) + '×'; };

window.setExplode = (pct) => {
  const val = Math.max(0, Math.min(100, Number(pct)));
  explodeK = val / 100.0;
  const k = explodeK;
  document.getElementById('exp-sl').value = val;
  document.getElementById('exp-val').textContent = Math.round(val) + '%';
  ['0', '50', '100'].forEach(p => { const el = document.getElementById('e-' + p); if (el) el.classList.toggle('active', Math.round(val) === Number(p)); });

  lowerHub.hubPos.position.set(0, 0, JZ_LO - 20 * k);
  upperHub.hubPos.position.set(0, 0, JZ_UP + 20 * k);
  hoodUpperHolder.position.set(0, -68 * k, 20 * k);
  hoodLowerHolder.position.set(0, -68 * k, -20 * k);
  plugGroup.position.set(0, 0, 35 * k);
  bodyMeshes.body_collar.position.set(0, -45 * k, 0);
  bodyMeshes.body_fasteners.position.set(0, -90 * k, 0);
  bodyMeshes.tray_boom.position.set(60 * k, 0, -22 * k);
  bodyMeshes.load_cell.position.set(60 * k, 0, 18 * k);
  bodyMeshes.live_wishbone.position.set(60 * k, 0, 52 * k);
  bodyMeshes.tray_deck.position.set(60 * k, 0, 88 * k);
  forkGroup.position.set(60 * k, 0, 125 * k);
  bodyMeshes.battery_pod.position.set(-60 * k, 0, 0);
  bodyMeshes.battery_pack.position.set(-60 * k, 0, 52 * k);
  bodyMeshes.battery_lid.position.set(-60 * k, 0, 102 * k);
};

window.setVis = (name, state) => {
  if (name === 'fork') forkGroup.visible = (state !== 'off');
  else if (name === 'sw') { fSwMesh.visible = (state !== 'off'); }
  else if (name === 'hold') {
    fHoldMesh.visible = (state !== 'off'); fHoldHwMesh.visible = (state !== 'off');
    mats.fork_hold_rails.transparent = (state === 'xray');
    mats.fork_hold_rails.opacity = (state === 'xray') ? 0.28 : 1.0;
    mats.fork_hold_rails.depthWrite = (state !== 'xray');
  }
  else if (name === 'cover') {
    fCoverMesh.visible = (state !== 'off');
    fCoverHwMesh.visible = (state !== 'off');
    mats.fork_top_cover.transparent = (state === 'xray');
    mats.fork_top_cover.opacity = (state === 'xray') ? 0.28 : 1.0;
    mats.fork_top_cover.depthWrite = (state !== 'xray');
  }
  else if (name === 'baycov') {
    fBayCovMesh.visible = (state !== 'off');
    fBayCovHwMesh.visible = (state !== 'off');
    mats.fork_bay_cover.transparent = (state === 'xray');
    mats.fork_bay_cover.opacity = (state === 'xray') ? 0.28 : 1.0;
    mats.fork_bay_cover.depthWrite = (state !== 'xray');
  }
  else if (name === 'tray') {
    trayGroup.visible = (state !== 'off');
    mats.tray_deck.transparent = (state === 'xray');
    mats.tray_deck.opacity = (state === 'xray') ? 0.30 : 1.0;
    mats.tray_deck.depthWrite = (state !== 'xray');
  } else if (name === 'legs') {
    legMeshes.forEach(g => g.visible = (state !== 'off'));
    for (const k of ['body_mount','bearing_bot','bearing_top','bearing_pinion','m8_hardware','servo_body','servo_spline','alum_horn','gear_20t','carriage_pod','motor_housing','motor_shaft','pinion','retainer','gantry_plate','v_wheels','pinion_cover','motor_cap']) {
      mats[k].transparent = (state === 'xray'); mats[k].opacity = (state === 'xray') ? 0.35 : 1.0; mats[k].depthWrite = (state !== 'xray');
    }
  } else if (name === 'body') {
    const show = (state !== 'off');
    colGroup.visible = show; plugGroup.visible = show; collarGroup.visible = show; batGroup.visible = show;
    hoodUpperHolder.visible = show; hoodLowerHolder.visible = show;
  } else if (name === 'rails') {
    const show = (state !== 'off'); railsGroup.visible = show; crossFixedGroup.visible = show; crossTurnGroup.visible = show;
    rackMountsGroup.visible = show; sensorsEnvGroup.visible = show; sensorPodHolders.forEach(g => g.visible = show);
  } else if (name === 'racks') {
    racksRefGroup.visible = (state !== 'off'); refLoadMesh.visible = (state !== 'off');
    mats.rack_bays.transparent = (state === 'xray'); mats.rack_bays.opacity = (state === 'xray') ? 0.22 : 0.85; mats.rack_bays.depthWrite = (state !== 'xray');
    mats.ref_load.transparent = (state === 'xray'); mats.ref_load.opacity = (state === 'xray') ? 0.36 : 0.92; mats.ref_load.depthWrite = (state !== 'xray');
  }
  ['solid', 'xray', 'off'].forEach(s => { const el = document.getElementById(`v-${name}-${s}`); if (el) el.classList.toggle('active', s === state); });
};

window.resetCam = () => { camera.position.set(520, 440, 680); controls.target.set(180, 250, -40); controls.update(); };
window.focusForklift = () => { camera.position.set(poseX + 340, 280 + poseZ, 300); controls.target.set(poseX + 180, 185 + poseZ, 150); controls.update(); };
window.focusLegs = () => { camera.position.set(poseX - 180, 210 + poseZ, 320); controls.target.set(poseX, 180 + poseZ, 80); controls.update(); };

let last = performance.now();
function animate() {
  requestAnimationFrame(animate);
  const dt = Math.min(0.05, (performance.now() - last) / 1000) * animSpeed;
  last = performance.now();

  if (mode === 'auto' && animSpeed > 0) {
    const [gx, gz, gsw, gfork, carry, shelfClampZ, dropX, dropZ, lbl, holdSec] = AUTO[wp];
    tx = gx; tz = gz; tsw = gsw; tFork = gfork;
    carryingLoad = carry; activeShelfZ = shelfClampZ;
    document.getElementById('phase-lbl').textContent = lbl;
    stepPlanner(dt);
    if (reached()) {
      dwell += dt;
      if (dwell > holdSec) {
        dwell = 0;
        loadBayX = dropX;
        loadShelfZ = dropZ;
        wp = (wp + 1) % AUTO.length;
      }
    }
  } else {
    stepPlanner(dt > 0 ? dt : 0.016);
  }

  robotGroup.position.set(poseX, 0, poseZ);
  lowerHub.carriagePivot.rotation.z = swL * Math.PI / 180;
  upperHub.carriagePivot.rotation.z = swU * Math.PI / 180;
  lowerHub.servoPivot.rotation.z = -2.0 * swL * Math.PI / 180;
  upperHub.servoPivot.rotation.z = -2.0 * swU * Math.PI / 180;

  BAYS.forEach((bx, i) => { if (Math.abs(poseX - bx) < 0.5 && poseZ < 0.05) { crossAng[`b${i}_lo`] = swL; crossAng[`b${i}_up`] = swU; } });
  for (const [k, g] of Object.entries(crossingTurns)) g.rotation.z = crossAng[k] * Math.PI / 180;

  const climbAng = (poseZ + poseX) / 10.5;
  upperHub.climbPivot.rotation.y = -climbAng;
  lowerHub.climbPivot.rotation.y = climbAng;

  const k = explodeK;
  fTieMesh.position.set(0, 0, -48 * k);
  fCoverMesh.position.set(0, 0, 86 * k);
  fCoverHwMesh.position.set(0, 0, 114 * k);
  fHoldMesh.position.set(0, 0, 44 * k);
  fHoldHwMesh.position.set(0, 0, 72 * k);
  fMidMesh.position.set(0, 0.5 * forkMm, 26 * k);
  fPinLMesh.position.set(148.5, -202.0 + 0.5 * forkMm, 36 * k);
  fPinLMesh.rotation.z = -(0.5 * forkMm) / 8.0;
  fPinRMesh.position.set(211.5, -202.0 + 0.5 * forkMm, 36 * k);
  fPinRMesh.rotation.z = (0.5 * forkMm) / 8.0;
  fNutMesh.position.set(0, 0.5 * forkMm, 26 * k);
  fFastMesh.position.set(0, 0.5 * forkMm, 26 * k);
  fTinesMesh.position.set(0, forkMm, 54 * k);

  if (carryingLoad) {
    const clampZ = (mode === 'auto') ? activeShelfZ : loadShelfZ;
    const shelfLz = Math.max(clampZ - 3.6, poseZ - 5.3);
    const tTray = Math.max(0, Math.min(1, (220.0 - forkMm) / 220.0));
    const lz = shelfLz * (1.0 - tTray) + poseZ * tTray;
    refLoadMesh.position.set(poseX + 60 * k, forkMm, lz + 185 * k);
  } else {
    refLoadMesh.position.set(loadBayX + 60 * k, 240, (loadShelfZ - 3.6) + 185 * k);
  }

  if (mode === 'auto') {
    document.getElementById('x-sl').value = poseX;
    document.getElementById('z-sl').value = poseZ;
    document.getElementById('sw-sl').value = (swL + swU) / 2;
    document.getElementById('arm-sl').value = forkMm;
  }
  document.getElementById('x-val').textContent = poseX.toFixed(1) + ' mm';
  document.getElementById('z-val').textContent = poseZ.toFixed(1) + ' mm';
  document.getElementById('sw-val').textContent = `L ${swL.toFixed(0)}° / U ${swU.toFixed(0)}°`;
  document.getElementById('arm-val').textContent = forkMm.toFixed(1) + ' mm';

  controls.update();
  renderer.render(scene, camera);
}
animate();

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
    for port in range(8896, 8912):
        try:
            srv = http.server.ThreadingHTTPServer(("127.0.0.1", port), Handler)
            break
        except OSError:
            continue
    else:
        raise RuntimeError("No free port in 8896..8911")

    url = f"http://127.0.0.1:{port}/"
    print(f"Full Robot + Embedded Forklift Studio -> {url}")
    threading.Timer(0.4, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        srv.shutdown()


if __name__ == "__main__":
    main()
