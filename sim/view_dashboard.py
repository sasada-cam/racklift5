"""All-in-One RackLift5 Engineering Timeline & Reusable Parts 3D Dashboard.

Unifies all 10 CAD/Three.js iterations (reusable hardware kits, swivel leg prototypes,
Option 1 hub & crossing turntable, dual-leg rails, body/picker subassembly, and the
latest full robot with rack mounts and protective leg hoods) inside a single server
with chronological timeline navigation, dependency links, and parallel STL preloading.

Usage:
  .pixi/envs/default/bin/python sim/view_dashboard.py
  .pixi/envs/default/bin/python sim/view_dashboard.py --port 8860 --no-open

The MuJoCo stages (02 v2, 03 original v3) are drawn from their XML with the real body tree + joints (forward kinematics
in the browser, checked against MuJoCo: 0.0000 mm) and play back a motion RECORDED WITH THE MUJOCO SIM ITSELF (the
physics engine is unchanged); every joint also has a slider.  The recordings (git-ignored, sim/build/traj/) are made by
sim/record_qpos.py with each version's own code:
  git worktree add --detach /tmp/v3orig e40004d          # the original v3
  .pixi/envs/sim/bin/python sim/model_v3.py               # (run inside /tmp/v3orig) -> its XML + meshes
  .pixi/envs/sim/bin/python sim/record_qpos.py --kind v3 --root /tmp/v3orig --out sim/build/traj/v3_legacy.json
  .pixi/envs/sim/bin/python sim/record_qpos.py --kind v2 --root . --out sim/build/traj/v2.json
  git worktree remove --force /tmp/v3orig
"""

import argparse
import http.server
import importlib.util
import json
import math
import os
import re
import socketserver
import threading
import urllib.parse
import webbrowser
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
BUILD_DIR = os.path.join(HERE, "build")
MODELS_DIR = os.path.join(ROOT, "models", "components")


OPENBUILD_VIEWER_HTML = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>OpenBuilds MT0134 + V-Wheel + 2020 Kit</title>
<style>
  *{box-sizing:border-box;margin:0;padding:0}
  body{background:#0b1120;color:#e2e8f0;font:13px/1.45 -apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif;overflow:hidden}
  #wrap{position:fixed;inset:0}
  #hud{position:fixed;top:14px;left:14px;width:270px;background:rgba(15,23,42,.88);backdrop-filter:blur(10px);
       border:1px solid #334155;border-radius:10px;padding:12px 14px;z-index:10}
  h2{font-size:13px;font-weight:700;color:#f8fafc;margin-bottom:4px}
  p{font-size:11px;color:#94a3b8;margin-bottom:10px}
  .row{display:flex;align-items:center;justify-content:space-between;padding:5px 0;border-top:1px solid #1e293b}
  .row label{font-size:12px;color:#cbd5e1}
  button,input[type=range]{accent-color:#38bdf8}
  button{background:#1e293b;color:#e2e8f0;border:1px solid #334155;border-radius:6px;padding:4px 8px;font-size:11px;cursor:pointer}
  button.on{background:#0284c7;border-color:#38bdf8;color:#fff}
  .slider-row{padding-top:8px;border-top:1px solid #1e293b}
  .slider-row span{display:flex;justify-content:space-between;font-size:11px;color:#94a3b8;margin-bottom:4px}
  input[type=range]{width:100%}
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
<div id="wrap"></div>
<div id="hud">
  <h2>OpenBuilds MT0134 + V-Wheel + 2020 Kit</h2>
  <p>Reusable hardware library exported from <code>cad/build_openbuilds_gantry.py</code>.</p>
  <div class="row"><label>MT0134 Gantry Plate</label><button class="on" data-k="plate">Shown</button></div>
  <div class="row"><label>2020 T-Slot Rail (200mm)</label><button class="on" data-k="rail">Shown</button></div>
  <div class="row"><label>V-Wheel Stack (4x offset)</label><button class="on" data-k="stack">Shown</button></div>
  <div class="row"><label>Single Delrin V-Wheel</label><button class="on" data-k="wheel">Shown</button></div>
  <div class="slider-row">
    <span>Exploded Spread <b id="expVal">0%</b></span>
    <input id="expSlider" type="range" min="0" max="100" value="0"/>
  </div>
</div>
<script type="module">
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b1120);
const camera = new THREE.PerspectiveCamera(42, window.innerWidth / window.innerHeight, 0.5, 2000);
camera.position.set(140, 120, 170);
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(window.innerWidth, window.innerHeight);
document.getElementById('wrap').appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
scene.add(new THREE.HemisphereLight(0xdbeafe, 0x1e293b, 1.4));
const dir = new THREE.DirectionalLight(0xffffff, 1.5);
dir.position.set(180, 240, 160);
scene.add(dir);
const grid = new THREE.GridHelper(320, 16, 0x334155, 0x1e293b);
grid.position.y = -45;
scene.add(grid);

const root = new THREE.Group();
root.rotation.x = -Math.PI / 2;
scene.add(root);

const loader = new STLLoader();
const parts = {};
const basePos = {
  plate: new THREE.Vector3(0, 0, 16),
  rail:  new THREE.Vector3(0, 0, -24),
  stack: new THREE.Vector3(-48, 0, 0),
  wheel: new THREE.Vector3(58, 0, 0),
};
const expOff = {
  plate: new THREE.Vector3(0, 0, 35),
  rail:  new THREE.Vector3(0, 0, -35),
  stack: new THREE.Vector3(-35, 0, 0),
  wheel: new THREE.Vector3(35, 0, 0),
};
const defs = [
  ['plate', '/meshes/MT0134_gantry_plate.stl', 0x334155, 0.7, 0.3],
  ['rail',  '/meshes/T_slot_2020_rail_200mm.stl', 0x94a3b8, 0.85, 0.25],
  ['stack', '/meshes/V_wheel_stack.stl', 0x1e293b, 0.25, 0.55],
  ['wheel', '/meshes/Delrin_V_wheel.stl', 0x0f172a, 0.15, 0.65],
];
defs.forEach(([key, url, color, metal, rough]) => {
  loader.load(url, (geom) => {
    geom.center();
    const mat = new THREE.MeshStandardMaterial({ color, metalness: metal, roughness: rough });
    const mesh = new THREE.Mesh(geom, mat);
    mesh.position.copy(basePos[key]);
    root.add(mesh);
    parts[key] = mesh;
  });
});

document.querySelectorAll('button[data-k]').forEach((btn) => {
  btn.addEventListener('click', () => {
    const k = btn.dataset.k;
    if (!parts[k]) return;
    parts[k].visible = !parts[k].visible;
    btn.classList.toggle('on', parts[k].visible);
    btn.textContent = parts[k].visible ? 'Shown' : 'Hidden';
  });
});

const expSlider = document.getElementById('expSlider');
const expVal = document.getElementById('expVal');
expSlider.addEventListener('input', () => {
  const k = Number(expSlider.value) / 100;
  expVal.textContent = expSlider.value + '%';
  Object.keys(parts).forEach((key) => {
    parts[key].position.copy(basePos[key]).addScaledVector(expOff[key], k);
  });
});

window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});
(function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
})();
</script>
</body>
</html>
"""


def _qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def _qrot(q, v):
    w, x, y, z = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def _euler_xyz(e):
    """MuJoCo euler (radian, default 'xyz' sequence) -> quat (w, x, y, z)"""
    q = (1.0, 0.0, 0.0, 0.0)
    for ang, ax in zip(e, ((1, 0, 0), (0, 1, 0), (0, 0, 1))):
        h = ang / 2.0
        q = _qmul(q, (math.cos(h), ax[0] * math.sin(h), ax[1] * math.sin(h), ax[2] * math.sin(h)))
    return q


def _mjcf_tree(xml_path: str):
    """MuJoCo XML -> body tree (positions in mm), joints, geoms in body-local frames, mesh files"""
    root = ET.parse(xml_path).getroot()
    mesh_files = {m.attrib["name"]: os.path.basename(m.attrib["file"]) for m in root.findall(".//asset/mesh")
                  if "name" in m.attrib and "file" in m.attrib}
    f3 = lambda s_, d: [float(v) for v in (s_ or d).split()]
    bodies, geoms = [], []

    def frame(el):
        p = [v * 1000.0 for v in f3(el.attrib.get("pos"), "0 0 0")]
        if "quat" in el.attrib:
            q = f3(el.attrib["quat"], "1 0 0 0")
        elif "euler" in el.attrib:
            q = list(_euler_xyz(f3(el.attrib["euler"], "0 0 0")))
        else:
            q = [1.0, 0.0, 0.0, 0.0]
        return [round(v, 3) for v in p], [round(v, 6) for v in q]

    def add_geom(el, bi):
        t = el.attrib.get("type", "sphere")
        if t == "plane":
            return
        g = {"b": bi, "c": f3(el.attrib.get("rgba"), "0.65 0.72 0.80 1"), "g": el.attrib.get("group", "0")}
        if "fromto" in el.attrib and t in ("cylinder", "capsule"):
            ft = [v * 1000.0 for v in f3(el.attrib["fromto"], "0 0 0 0 0 1")]
            g.update(t="cylft", r=round(float(el.attrib.get("size", "0.004").split()[0]) * 1000.0, 2), a=ft[:3], z=ft[3:])
        else:
            p, q = frame(el)
            g.update(p=p, q=q)
            if t == "mesh" and el.attrib.get("mesh") in mesh_files:
                g.update(t="mesh", f=mesh_files[el.attrib["mesh"]])
            elif t == "box":
                g.update(t="box", s=[round(v * 2000.0, 2) for v in f3(el.attrib.get("size"), "0.01 0.01 0.01")])
            elif t in ("cylinder", "capsule"):
                v = [x * 1000.0 for x in f3(el.attrib.get("size"), "0.01 0.01")]
                g.update(t="cyl", r=round(v[0], 2), h=round(v[1] * 2.0, 2))
            elif t == "sphere":
                g.update(t="sph", r=round(f3(el.attrib.get("size"), "0.01")[0] * 1000.0, 2))
            else:
                return
        geoms.append(g)

    def walk(node, parent):
        for child in node:
            if child.tag == "body":
                p, q = frame(child)
                bi = len(bodies)
                bodies.append({"n": child.attrib.get("name", f"b{bi}"), "par": parent, "p": p, "q": q, "j": []})
                for el in child:
                    if el.tag == "freejoint":
                        bodies[bi]["j"].append({"name": el.attrib.get("name", ""), "t": "free"})
                    elif el.tag == "joint":
                        bodies[bi]["j"].append({"name": el.attrib.get("name", ""), "t": el.attrib.get("type", "hinge"),
                                                "ax": f3(el.attrib.get("axis"), "0 0 1"),
                                                "p": [v * 1000.0 for v in f3(el.attrib.get("pos"), "0 0 0")]})
                    elif el.tag == "geom":
                        add_geom(el, bi)
                walk(child, bi)
            elif child.tag == "geom" and parent == -1:
                add_geom(child, -1)

    walk(root.find("worldbody"), -1)
    return bodies, geoms, mesh_files


def build_mujoco_three_html(xml_path: str, title: str, subtitle: str, traj_file: str = "") -> str:
    """MuJoCo XML + STL meshes -> interactive Three.js viewer with the real body tree and joints (forward kinematics in the
    browser), playback of a qpos recording made with the MuJoCo sim itself (sim/record_qpos.py) and a slider per joint."""
    bodies, geoms, _ = _mjcf_tree(xml_path)
    groups = sorted(set(g["g"] for g in geoms))
    html = _MJ_VIEWER_TEMPLATE
    for k, v in (("__TITLE__", title), ("__SUB__", subtitle), ("__BODIES__", json.dumps(bodies, separators=(",", ":"))),
                 ("__GEOMS__", json.dumps(geoms, separators=(",", ":"))), ("__GROUPS__", json.dumps(groups)),
                 ("__TRAJ__", json.dumps(f"/traj/{traj_file}" if traj_file else ""))):
        html = html.replace(k, v)
    return html


_MJ_VIEWER_TEMPLATE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>__TITLE__</title>
<style>
  *{box-sizing:border-box;margin:0;padding:0}
  body{background:#0b1120;color:#e2e8f0;font:12.5px/1.4 system-ui,-apple-system,sans-serif;overflow:hidden}
  #wrap{position:fixed;inset:0}
  #hud{position:fixed;top:14px;left:14px;width:330px;max-height:calc(100vh - 28px);overflow:auto;background:rgba(15,23,42,.92);
       backdrop-filter:blur(10px);border:1px solid #334155;border-radius:10px;padding:12px 14px;z-index:10}
  h1{font-size:14px;font-weight:700;color:#f8fafc;margin-bottom:2px}
  h2{font-size:11.5px;font-weight:700;color:#cbd5e1;margin:10px 0 5px}
  .sub{font-size:11px;color:#94a3b8;margin-bottom:9px}
  .bar{height:5px;background:#1e293b;border-radius:99px;overflow:hidden;margin-bottom:8px}
  .fill{height:100%;width:0%;background:linear-gradient(90deg,#38bdf8,#6366f1);transition:width .1s}
  .meta{font-size:11px;color:#94a3b8;margin-bottom:6px}
  .row{display:flex;gap:5px;flex-wrap:wrap;align-items:center}
  button{background:#1e293b;color:#e2e8f0;border:1px solid #334155;border-radius:6px;padding:4px 8px;font-size:11px;cursor:pointer}
  button.on{background:#0284c7;border-color:#38bdf8;color:#fff}
  select{background:#1e293b;color:#e2e8f0;border:1px solid #334155;border-radius:6px;padding:4px;font-size:11px;width:100%}
  input[type=range]{width:100%}
  .jrow{display:grid;grid-template-columns:120px 1fr 52px;gap:6px;align-items:center;font-size:10.5px;margin:2px 0}
  .jrow span{overflow:hidden;text-overflow:ellipsis;white-space:nowrap;color:#cbd5e1}
  .jrow b{font-weight:500;color:#94a3b8;text-align:right}
  details summary{cursor:pointer;color:#cbd5e1;font-weight:700;font-size:11.5px;margin-top:10px}
</style>
<script type="importmap">
{"imports":{"three":"https://unpkg.com/three@0.160.0/build/three.module.js","three/addons/":"https://unpkg.com/three@0.160.0/examples/jsm/"}}
</script>
</head>
<body>
<div id="wrap"></div>
<div id="hud">
  <h1>__TITLE__</h1>
  <div class="sub">__SUB__</div>
  <div class="bar"><div id="fill" class="fill"></div></div>
  <div id="meta" class="meta">Loading…</div>
  <div class="row" id="grpBtns"><button id="fitBtn">Reset Camera</button></div>
  <div id="playBox" style="display:none">
    <h2>Motion (recorded from the MuJoCo sim)</h2>
    <select id="clipSel"></select>
    <div class="row" style="margin-top:6px">
      <button id="playBtn" class="on">Pause</button>
      <button data-sp="0.5">0.5×</button><button data-sp="1" class="on">1×</button><button data-sp="2">2×</button><button data-sp="4">4×</button>
    </div>
    <input id="scrub" type="range" min="0" max="1" step="1" value="0" style="margin-top:6px"/>
    <div id="tlbl" class="meta">0.0 s</div>
  </div>
  <details id="jointBox"><summary>Joints (drag to move; pauses playback)</summary><div id="jointList"></div></details>
</div>
<script type="module">
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';

const BODIES = __BODIES__;
const GEOMS = __GEOMS__;
const GROUPS = __GROUPS__;
const TRAJ_URL = __TRAJ__;

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b1120);
const camera = new THREE.PerspectiveCamera(42, window.innerWidth / window.innerHeight, 1, 15000);
camera.position.set(1100, 850, 1400);
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(window.innerWidth, window.innerHeight);
document.getElementById('wrap').appendChild(renderer.domElement);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
scene.add(new THREE.HemisphereLight(0xdbeafe, 0x1e293b, 1.35));
const dir = new THREE.DirectionalLight(0xffffff, 1.45); dir.position.set(900, 1600, 1100); scene.add(dir);
scene.add(new THREE.GridHelper(3200, 32, 0x334155, 0x1e293b));
const root = new THREE.Group(); root.rotation.x = -Math.PI / 2; scene.add(root);   // MuJoCo z-up -> three y-up

// ---- body tree: one group per body, world matrix set by forward kinematics from qpos ----
const bodyNodes = BODIES.map(() => { const g = new THREE.Group(); g.matrixAutoUpdate = false; root.add(g); return g; });
const worldNode = new THREE.Group(); root.add(worldNode);
const groupMeshes = {}; GROUPS.forEach(g => groupMeshes[g] = []);
const parentOf = (g) => g.b < 0 ? worldNode : bodyNodes[g.b];

// joint table (filled from the recording; else from the tree, in XML order = MuJoCo qpos order)
let nq = 0;
const JOINTS = [];
BODIES.forEach((b, bi) => b.j.forEach(j => { j.adr = nq; nq += (j.t === 'free' ? 7 : (j.t === 'ball' ? 4 : 1)); JOINTS.push({ ...j, bi }); }));
let qpos = new Float64Array(nq);
BODIES.forEach(b => b.j.forEach(j => { if (j.t === 'free') { qpos.set([b.p[0] / 1000, b.p[1] / 1000, b.p[2] / 1000, b.q[0], b.q[1], b.q[2], b.q[3]], j.adr); } }));

const _m = new THREE.Matrix4(), _t = new THREE.Matrix4(), _q = new THREE.Quaternion(), _v = new THREE.Vector3();
const world = BODIES.map(() => new THREE.Matrix4());
function fk() {
  for (let i = 0; i < BODIES.length; i++) {
    const b = BODIES[i];
    const M = world[i];
    if (b.par >= 0) M.copy(world[b.par]); else M.identity();
    M.multiply(_t.compose(_v.set(b.p[0], b.p[1], b.p[2]), _q.set(b.q[1], b.q[2], b.q[3], b.q[0]), new THREE.Vector3(1, 1, 1)));
    for (const j of b.j) {
      const a = j.adr;
      if (j.t === 'free') {
        M.compose(_v.set(qpos[a] * 1000, qpos[a + 1] * 1000, qpos[a + 2] * 1000), _q.set(qpos[a + 4], qpos[a + 5], qpos[a + 6], qpos[a + 3]), new THREE.Vector3(1, 1, 1));
      } else if (j.t === 'hinge') {
        M.multiply(_t.makeTranslation(j.p[0], j.p[1], j.p[2]));
        M.multiply(_t.makeRotationAxis(_v.set(j.ax[0], j.ax[1], j.ax[2]).normalize(), qpos[a]));
        M.multiply(_t.makeTranslation(-j.p[0], -j.p[1], -j.p[2]));
      } else if (j.t === 'slide') {
        _v.set(j.ax[0], j.ax[1], j.ax[2]).normalize().multiplyScalar(qpos[a] * 1000);
        M.multiply(_t.makeTranslation(_v.x, _v.y, _v.z));
      }
    }
    bodyNodes[i].matrix.copy(M);
    bodyNodes[i].matrixWorldNeedsUpdate = true;
  }
}

function matFor(c) {
  const tr = c[3] < 0.99;
  return new THREE.MeshStandardMaterial({ color: new THREE.Color(c[0], c[1], c[2]), metalness: 0.25, roughness: 0.55, transparent: tr, opacity: c[3], depthWrite: !tr });
}
function place(mesh, g) {
  if (g.t === 'cylft') {
    const a = new THREE.Vector3(...g.a), z = new THREE.Vector3(...g.z);
    mesh.position.copy(a.clone().add(z).multiplyScalar(0.5));
    mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), z.clone().sub(a).normalize());
  } else {
    mesh.position.set(g.p[0], g.p[1], g.p[2]);
    mesh.quaternion.set(g.q[1], g.q[2], g.q[3], g.q[0]);
    if (g.t === 'cyl') mesh.rotateX(Math.PI / 2);                  // three cylinders run along y, MuJoCo's along z
  }
  parentOf(g).add(mesh);
  groupMeshes[g.g].push(mesh);
}
GEOMS.forEach(g => {
  if (g.t === 'box') place(new THREE.Mesh(new THREE.BoxGeometry(g.s[0], g.s[1], g.s[2]), matFor(g.c)), g);
  else if (g.t === 'cyl') place(new THREE.Mesh(new THREE.CylinderGeometry(g.r, g.r, g.h, 20), matFor(g.c)), g);
  else if (g.t === 'cylft') {
    const h = new THREE.Vector3(...g.z).distanceTo(new THREE.Vector3(...g.a));
    place(new THREE.Mesh(new THREE.CylinderGeometry(g.r, g.r, h, 16), matFor(g.c)), g);
  } else if (g.t === 'sph') place(new THREE.Mesh(new THREE.SphereGeometry(g.r, 16, 12), matFor(g.c)), g);
});

const grpBtns = document.getElementById('grpBtns');
GROUPS.forEach(g => {
  const btn = document.createElement('button'); btn.className = 'on';
  btn.textContent = g === '0' ? 'Main (G0)' : (g === '1' ? 'Extra (G1)' : 'Covers (G' + g + ')');
  btn.onclick = () => { const vis = !btn.classList.contains('on'); btn.classList.toggle('on', vis); groupMeshes[g].forEach(m => m.visible = vis); };
  grpBtns.appendChild(btn);
});

function fitCamera() {
  root.updateMatrixWorld(true);
  const box = new THREE.Box3().setFromObject(root);
  if (box.isEmpty()) return;
  const ctr = new THREE.Vector3(), sz = new THREE.Vector3(); box.getCenter(ctr); box.getSize(sz);
  const d = Math.max(sz.x, sz.y, sz.z, 300) * 1.15;
  controls.target.copy(ctr); camera.position.set(ctr.x + d * 0.72, ctr.y + d * 0.52, ctr.z + d * 0.95); controls.update();
}
document.getElementById('fitBtn').onclick = fitCamera;

// ---- recording: clips of qpos (start + sparse per-frame changes), rebuilt into full frames ----
let clips = [], clipIdx = 0, frameF = 0, playing = true, speed = 1, frames = null, dt = 0.1;
const scrub = document.getElementById('scrub'), tlbl = document.getElementById('tlbl'), playBtn = document.getElementById('playBtn');
function buildFrames(c) {
  const out = []; let cur = Float64Array.from(remap(c.start));
  out.push(cur);
  for (const fr of c.frames) { cur = Float64Array.from(cur); for (const [i, v] of fr) { const k = MAP[i]; if (k >= 0) cur[k] = v; } out.push(cur); }
  return out;
}
let MAP = null;          // recording qpos index -> this viewer's qpos index (matched by joint name)
function remap(arr) { const q = Float64Array.from(qpos); arr.forEach((v, i) => { const k = MAP[i]; if (k >= 0) q[k] = v; }); return q; }
function selectClip(i) {
  clipIdx = i; frames = buildFrames(clips[i]); dt = clips[i].dt; frameF = 0;
  scrub.max = frames.length - 1; scrub.value = 0; applyFrame(0);
}
function applyFrame(k) {
  qpos.set(frames[Math.min(frames.length - 1, Math.max(0, k))]);
  tlbl.textContent = `${(k * dt).toFixed(1)} s / ${((frames.length - 1) * dt).toFixed(1)} s`;
  syncSliders();
}
async function loadTraj() {
  if (!TRAJ_URL) return;
  try {
    const r = await fetch(TRAJ_URL); if (!r.ok) return;
    const T = await r.json();
    const byName = {}; JOINTS.forEach(j => byName[j.name] = j);
    MAP = new Int32Array(T.nq).fill(-1);
    T.joints.forEach(rj => { const j = byName[rj.name]; if (!j) return; const n = rj.type === 'free' ? 7 : (rj.type === 'ball' ? 4 : 1); for (let k = 0; k < n; k++) MAP[rj.adr + k] = j.adr + k; });
    clips = T.clips;
    // slider ranges from the recorded motion
    const lo = new Float64Array(nq).fill(Infinity), hi = new Float64Array(nq).fill(-Infinity);
    const sel = document.getElementById('clipSel');
    clips.forEach((c, i) => {
      const o = document.createElement('option'); o.value = i; o.textContent = c.name; sel.appendChild(o);
      buildFrames(c).forEach(f => f.forEach((v, k) => { if (v < lo[k]) lo[k] = v; if (v > hi[k]) hi[k] = v; }));
    });
    JOINTS.forEach(j => { if (j.t !== 'hinge' && j.t !== 'slide') return; j.lo = lo[j.adr]; j.hi = hi[j.adr]; });
    sel.onchange = () => selectClip(Number(sel.value));
    document.getElementById('playBox').style.display = '';
    selectClip(0);
    buildSliders();
  } catch (e) { console.warn('no recording', e); }
}

// ---- joint sliders ----
const sliders = [];
function buildSliders() {
  const box = document.getElementById('jointList'); box.innerHTML = '';
  sliders.length = 0;
  JOINTS.filter(j => j.t === 'hinge' || j.t === 'slide').forEach(j => {
    let lo = j.lo, hi = j.hi;
    const span = j.t === 'hinge' ? Math.PI : 0.6;
    if (!isFinite(lo) || !isFinite(hi) || hi - lo < 1e-6) { lo = (isFinite(lo) ? lo : 0) - span / 2; hi = lo + span; }
    else { const m = (hi - lo) * 0.1; lo -= m; hi += m; }
    const row = document.createElement('div'); row.className = 'jrow';
    row.innerHTML = `<span title="${j.name}">${j.name}</span><input type="range" min="${lo}" max="${hi}" step="${(hi - lo) / 400}"/><b></b>`;
    const inp = row.querySelector('input'), val = row.querySelector('b');
    inp.oninput = () => { playing = false; playBtn.textContent = 'Play'; playBtn.classList.remove('on'); qpos[j.adr] = Number(inp.value); fmt(); };
    const fmt = () => { val.textContent = j.t === 'hinge' ? (qpos[j.adr] * 180 / Math.PI).toFixed(1) + '°' : (qpos[j.adr] * 1000).toFixed(0) + ' mm'; };
    sliders.push({ j, inp, fmt });
    box.appendChild(row);
  });
  syncSliders();
}
function syncSliders() { sliders.forEach(s => { s.inp.value = qpos[s.j.adr]; s.fmt(); }); }

playBtn.onclick = () => { playing = !playing; playBtn.textContent = playing ? 'Pause' : 'Play'; playBtn.classList.toggle('on', playing); };
document.querySelectorAll('[data-sp]').forEach(b => b.onclick = () => {
  speed = Number(b.dataset.sp); document.querySelectorAll('[data-sp]').forEach(x => x.classList.toggle('on', x === b));
});
scrub.oninput = () => { playing = false; playBtn.textContent = 'Play'; playBtn.classList.remove('on'); frameF = Number(scrub.value); applyFrame(frameF); };

// ---- meshes ----
const loader = new STLLoader();
const uniqueFiles = [...new Set(GEOMS.filter(g => g.t === 'mesh').map(g => g.f))];
const geomMap = new Map(); let loadedFiles = 0;
async function loadAllMeshes() {
  const queue = [...uniqueFiles];
  await Promise.all(Array.from({ length: 12 }, async () => {
    while (queue.length) {
      const f = queue.shift();
      await new Promise(res => loader.load('/meshes/' + f, (geom) => {
        geom.computeVertexNormals(); geomMap.set(f, geom); loadedFiles++;
        document.getElementById('fill').style.width = Math.round(loadedFiles / uniqueFiles.length * 100) + '%';
        document.getElementById('meta').textContent = `Loaded ${loadedFiles}/${uniqueFiles.length} meshes`;
        res();
      }, undefined, () => { loadedFiles++; res(); }));
    }
  }));
  GEOMS.filter(g => g.t === 'mesh').forEach(g => { const geom = geomMap.get(g.f); if (geom) place(new THREE.Mesh(geom, matFor(g.c)), g); });
  document.getElementById('meta').textContent = `${GEOMS.length} geoms, ${BODIES.length} bodies, ${JOINTS.length} joints`;
  fk(); fitCamera();
}
await loadTraj();
if (!clips.length) buildSliders();
fk();
loadAllMeshes();

window.addEventListener('resize', () => { camera.aspect = window.innerWidth / window.innerHeight; camera.updateProjectionMatrix(); renderer.setSize(window.innerWidth, window.innerHeight); });
const clock = new THREE.Clock();
(function animate() {
  requestAnimationFrame(animate);
  const d = Math.min(clock.getDelta(), 0.1);
  if (playing && frames) {
    frameF += d * speed / dt;
    if (frameF > frames.length - 1) {                       // next clip, then loop
      const sel = document.getElementById('clipSel');
      const nxt = (clipIdx + 1) % clips.length; sel.value = nxt; selectClip(nxt);
    }
    scrub.value = Math.floor(frameF); applyFrame(Math.floor(frameF));
  }
  fk();
  controls.update();
  renderer.render(scene, camera);
})();
</script>
</body>
</html>
"""


def build_v1_three_html() -> str:
    """Lazily generate the original v1 SCARA Picker + Single-Rail Lift Three.js viewer from archive/v1/source."""
    import base64
    import sys as _sys
    import numpy as _np

    v1_dir = os.path.join(ROOT, "archive", "v1", "source")
    if v1_dir not in _sys.path:
        _sys.path.insert(0, v1_dir)
    import assembly as A1  # noqa: E402
    from params import LEVELS as V1_LEVELS  # noqa: E402
    from csg import Solid as V1Solid  # noqa: E402

    items, _ = A1.build(0, "empty", T=V1_LEVELS[0], pin_y=-178.0)
    merged = {}
    for n, s, col in items:
        tris = s.triangles() if isinstance(s, V1Solid) else s
        if len(tris) == 0:
            continue
        merged.setdefault((col,), []).append(tris)

    meshes = []
    for (col,), tri_list in merged.items():
        arr = _np.concatenate(tri_list)
        q = _np.round(arr.reshape(-1, 3) / 0.05).astype(_np.int16)
        meshes.append({
            "c": [round(col[0] / 255.0, 3), round(col[1] / 255.0, 3), round(col[2] / 255.0, 3)],
            "p": base64.b64encode(q.tobytes()).decode("ascii"),
        })
    meshes_json = json.dumps(meshes, separators=(",", ":"))
    return f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>Original SCARA Picker Concept (v1)</title>
<style>
  *{{box-sizing:border-box;margin:0;padding:0}}
  body{{background:#0b1120;color:#e2e8f0;font:12.5px/1.4 system-ui,-apple-system,sans-serif;overflow:hidden}}
  #wrap{{position:fixed;inset:0}}
  #hud{{position:fixed;top:14px;left:14px;width:295px;background:rgba(15,23,42,.90);backdrop-filter:blur(10px);
       border:1px solid #334155;border-radius:10px;padding:12px 14px;z-index:10}}
  h1{{font-size:14px;font-weight:700;color:#f8fafc;margin-bottom:2px}}
  .sub{{font-size:11px;color:#94a3b8;margin-bottom:8px}}
  button{{background:#1e293b;color:#e2e8f0;border:1px solid #334155;border-radius:6px;padding:4px 8px;font-size:11px;cursor:pointer}}
</style>
<script type="importmap">
{{"imports":{{"three":"https://unpkg.com/three@0.160.0/build/three.module.js","three/addons/":"https://unpkg.com/three@0.160.0/examples/jsm/"}}}}
</script>
</head>
<body>
<div id="wrap"></div>
<div id="hud">
  <h1>Original SCARA Picker Concept (v1)</h1>
  <div class="sub">Single Vertical Lift Rail · Articulated Upper Arm &amp; Forearm Hook (archive/v1)</div>
  <button id="fitBtn">Reset Camera</button>
</div>
<script type="module">
import * as THREE from 'three';
import {{ OrbitControls }} from 'three/addons/controls/OrbitControls.js';

const MESHES = {meshes_json};
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b1120);
const camera = new THREE.PerspectiveCamera(42, window.innerWidth / window.innerHeight, 1, 15000);
const renderer = new THREE.WebGLRenderer({{ antialias: true }});
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(window.innerWidth, window.innerHeight);
document.getElementById('wrap').appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
scene.add(new THREE.HemisphereLight(0xdbeafe, 0x1e293b, 1.35));
const dir = new THREE.DirectionalLight(0xffffff, 1.45);
dir.position.set(900, 1600, 1100);
scene.add(dir);
scene.add(new THREE.GridHelper(3200, 32, 0x334155, 0x1e293b));

const root = new THREE.Group();
root.rotation.x = -Math.PI / 2;
scene.add(root);

MESHES.forEach((m) => {{
  const bin = atob(m.p);
  const bytes = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
  const i16 = new Int16Array(bytes.buffer);
  const f32 = new Float32Array(i16.length);
  for (let i = 0; i < i16.length; i++) f32[i] = i16[i] * 0.05;
  const geom = new THREE.BufferGeometry();
  geom.setAttribute('position', new THREE.BufferAttribute(f32, 3));
  geom.computeVertexNormals();
  const mat = new THREE.MeshStandardMaterial({{ color: new THREE.Color(m.c[0], m.c[1], m.c[2]), metalness: 0.25, roughness: 0.55 }});
  root.add(new THREE.Mesh(geom, mat));
}});

function fitCamera() {{
  const box = new THREE.Box3().setFromObject(root);
  const ctr = new THREE.Vector3(), sz = new THREE.Vector3();
  box.getCenter(ctr); box.getSize(sz);
  const d = Math.max(sz.x, sz.y, sz.z, 300) * 1.15;
  controls.target.copy(ctr);
  camera.position.set(ctr.x + d * 0.72, ctr.y + d * 0.52, ctr.z + d * 0.95);
  controls.update();
}}
fitCamera();
document.getElementById('fitBtn').onclick = fitCamera;

window.addEventListener('resize', () => {{
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
}});
(function animate() {{
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}})();
</script>
</body>
</html>"""


STAGES = [
    {
        "id": "01_legacy_v1",
        "step": "01",
        "title": "Original SCARA Picker Concept (v1)",
        "category": "legacy",
        "badge": "Legacy v1",
        "viewer_module": None,
        "v1_archive": True,
        "cad_files": ["archive/v1/source/assembly.py", "archive/v1/source/parts.py"],
        "sim_file": "archive/v1/source/export_viewer.py",
        "summary": "Earliest single-rail lift + articulated SCARA upper arm and forearm hook prototype before dual-leg turning.",
        "notes": "Parametric NumPy CSG assembly built lazily on demand from archive/v1/source.",
        "uses": [],
        "used_by": ["02_legacy_v2"],
        "mounts": {},
    },
    {
        "id": "02_legacy_v2",
        "step": "02",
        "title": "Early Dual-Leg Concept (v2)",
        "category": "legacy",
        "badge": "Legacy v2",
        "viewer_module": None,
        "xml_path": os.path.join(BUILD_DIR, "racklift5.xml"),
        "xml_sub": "Early dual-leg swivel robot + multi-bay rack concept (MuJoCo v2 XML)",
        "cad_files": ["sim/model.py", "sim/run.py", "generator/assembly_v2.py"],
        "sim_file": "sim/build/racklift5.xml",
        "summary": "First dual-rotor crossing turntable & multi-bay rack architecture prior to Compact Squid and T-slot redesigns.",
        "notes": "85 exported STL meshes + primitive geometry rendered dynamically from racklift5.xml.",
        "uses": ["01_legacy_v1"],
        "used_by": ["03_legacy_v3"],
        "mounts": {"/meshes/": os.path.join(BUILD_DIR, "meshes"), "/traj/": os.path.join(BUILD_DIR, "traj")},
        "traj": "v2.json",
    },
    {
        "id": "03_legacy_v3",
        "step": "03",
        "title": "Compact Squid Architecture (Pure v3)",
        "category": "legacy",
        "badge": "Legacy v3",
        "viewer_module": None,
        "xml_path": os.path.join(BUILD_DIR, "racklift5_v3_legacy.xml"),
        "xml_sub": "Pure pre-Option-1 Compact Squid robot, crossings, transfer cart & dock jack (commit e40004d: the original v3, before any of the Option 1 / crossing-pivot work)",
        "cad_files": ["git:e40004d/sim/model_v3.py", "git:e40004d/generator/assembly_v3.py"],
        "sim_file": "sim/build/racklift5_v3_legacy.xml",
        "summary": "Pure Compact Squid layout before today's changes: plain 50 mm bearing crossings, original hub, 2-stage telescopic arms, transfer cart & dock.",
        "notes": "521 STL parts built by sim/model_v3.py at commit e40004d (original v3: plain 50 mm crossing bearing, original hub), saved under meshes_v3_legacy without altering any repo files.",
        "uses": ["02_legacy_v2"],
        "used_by": ["10_option1_hub", "11_dual_legs", "12_option1_body", "13_option1_full"],
        "mounts": {"/meshes/": os.path.join(BUILD_DIR, "meshes_v3_legacy"), "/traj/": os.path.join(BUILD_DIR, "traj")},
        "traj": "v3_legacy.json",
    },
    {
        "id": "04_4632_370",
        "step": "04",
        "title": "4632-370 Worm Climber Kit",
        "category": "reusable",
        "badge": "Reusable Part",
        "viewer_module": "view_4632_370",
        "cad_files": ["cad/build_4632_370.py", "cad/build_4632_370_kit.py"],
        "sim_file": "sim/view_4632_370.py",
        "summary": "Reusable 4632-370 worm-gear climber motor, 14T Mod-1.5 pinion, bottom cradle, top cover, and M3 shaft retainer.",
        "notes": "Self-locking worm gearbox + exploded view + live 100 RPM drive testbed reused across all leg pods.",
        "uses": [],
        "used_by": ["07_swivel_v1", "08_swivel_v2", "09_tslot_v3", "10_option1_hub", "13_option1_full"],
        "mounts": {"/meshes/": os.path.join(BUILD_DIR, "meshes_4632_370")},
    },
    {
        "id": "05_td8125mg",
        "step": "05",
        "title": "TD-8125MG 25KG Swivel Servo",
        "category": "reusable",
        "badge": "Reusable Part",
        "viewer_module": "view_td8125mg",
        "cad_files": ["cad/build_td8125mg.py"],
        "sim_file": "sim/view_td8125mg.py",
        "summary": "Reusable waterproof 25 kg-cm digital servo CAD model with exact ears, case dimensions, and 25T spline output.",
        "notes": "Forms the rotary actuator envelope for every swivel leg concept and geared dual-bearing hub.",
        "uses": [],
        "used_by": ["07_swivel_v1", "08_swivel_v2", "09_tslot_v3", "10_option1_hub", "13_option1_full"],
        "mounts": {"/meshes/": os.path.join(BUILD_DIR, "meshes_td8125mg")},
    },
    {
        "id": "06_openbuilds_kit",
        "step": "06",
        "title": "OpenBuilds MT0134 + V-Wheel + 2020 Kit",
        "category": "reusable",
        "badge": "Reusable Part",
        "viewer_module": None,
        "cad_files": ["cad/build_openbuilds_gantry.py"],
        "sim_file": "sim/view_dashboard.py (embedded)",
        "summary": "Exact OpenBuilds MT0134 gantry plate (65.5x65.5x3 mm), Delrin V-wheels (Ø24) + spacers/shims, and 2020 T-slot rail.",
        "notes": "Standardized COTS rail & carriage interface imported directly into T-slot pods and crossing turntables.",
        "uses": [],
        "used_by": ["09_tslot_v3", "10_option1_hub", "11_dual_legs", "13_option1_full"],
        "mounts": {"/meshes/": os.path.join(MODELS_DIR, "openbuilds_t_slot")},
    },
    {
        "id": "07_swivel_v1",
        "step": "07",
        "title": "Swivel Pod v1 (Inline Stack)",
        "category": "prototype",
        "badge": "Prototype v1",
        "viewer_module": "view_swivel_pod",
        "cad_files": ["cad/build_rail_swivel_assembly.py"],
        "sim_file": "sim/view_swivel_pod.py",
        "summary": "First dual-motor rail-drive + 90-deg swivel pod stacking the TD-8125MG upright on top of the 4632-370 housing.",
        "notes": "Introduced Ø26 mm load-bearing collar so bending loads bypass the spline, though the servo traveled with the carriage.",
        "uses": ["04_4632_370", "05_td8125mg"],
        "used_by": ["08_swivel_v2"],
        "mounts": {"/meshes/": os.path.join(BUILD_DIR, "meshes_swivel_pod")},
    },
    {
        "id": "08_swivel_v2",
        "step": "08",
        "title": "Switched Swivel Pod v2 (Fixed Servo)",
        "category": "prototype",
        "badge": "Prototype v2",
        "viewer_module": "view_switched_swivel_pod",
        "cad_files": ["cad/build_switched_swivel_pod.py"],
        "sim_file": "sim/view_switched_swivel_pod.py",
        "summary": "Switched-sides pod architecture keeping the body and TD-8125MG stationary (zero servo wire twist).",
        "notes": "Carriage pod rotates 90 deg coaxially over the 14T climber pinion while the fixed servo tower holds position.",
        "uses": ["04_4632_370", "05_td8125mg", "07_swivel_v1"],
        "used_by": ["09_tslot_v3"],
        "mounts": {"/meshes/": os.path.join(BUILD_DIR, "meshes_switched_swivel_pod")},
    },
    {
        "id": "09_tslot_v3",
        "step": "09",
        "title": "2020 T-Slot V-Wheel Carriage v3",
        "category": "prototype",
        "badge": "Prototype v3",
        "viewer_module": "view_t_slot_swivel_carriage",
        "cad_files": ["cad/build_t_slot_swivel_carriage.py"],
        "sim_file": "sim/view_t_slot_swivel_carriage.py",
        "summary": "Migrated from round rail to CNC4551 2020 T-slot rail with 4x Delrin V-wheels, MT0134 plate, and sideways 4632-370.",
        "notes": "Solved rail guidance stiffness using off-the-shelf OpenBuilds V-wheels clamping a continuous in-slot rack.",
        "uses": ["04_4632_370", "05_td8125mg", "06_openbuilds_kit", "08_swivel_v2"],
        "used_by": ["10_option1_hub"],
        "mounts": {"/meshes/": os.path.join(BUILD_DIR, "meshes_t_slot_carriage")},
    },
    {
        "id": "10_option1_hub",
        "step": "10",
        "title": "Geared Dual-Bearing Swivel Leg + Crossing",
        "category": "option1",
        "badge": "Core Mechanism",
        "viewer_module": "view_t_slot_option1",
        "cad_files": ["cad/build_t_slot_option1.py", "cad/build_option1_crossing.py"],
        "sim_file": "sim/view_t_slot_option1.py",
        "summary": "Production swivel leg with M8 spindle, dual 608ZZ bearings, 20T:40T (2x torque) gear reduction, Hall stops, and 17.5-pitch crossing turntable.",
        "notes": "Carriage pod, motor cap, pinion cover, stationary servo frame, Hall mounts, and turntable rotor/bracket.",
        "uses": ["04_4632_370", "05_td8125mg", "06_openbuilds_kit", "09_tslot_v3", "03_legacy_v3"],
        "used_by": ["11_dual_legs", "13_option1_full"],
        "mounts": {"/meshes/": os.path.join(BUILD_DIR, "meshes_t_slot_option1")},
    },
    {
        "id": "11_dual_legs",
        "step": "11",
        "title": "Dual-Leg Climber on 2-Bay 2020 Rails",
        "category": "option1",
        "badge": "Architecture",
        "viewer_module": "view_option1_dual_legs",
        "cad_files": ["cad/build_option1_dual_legs.py"],
        "sim_file": "sim/view_option1_dual_legs.py",
        "summary": "Mirrored upper and lower swivel legs (240.33 mm hub spacing, 105 mm pinion stagger) across 4 turntables on the 40x25x2 column bar.",
        "notes": "Verified full rail climbing, 90-deg turntable crossing transitions, and horizontal bay travel before body payload integration.",
        "uses": ["10_option1_hub", "06_openbuilds_kit", "03_legacy_v3"],
        "used_by": ["13_option1_full"],
        "mounts": {
            "/meshes_pod/": os.path.join(BUILD_DIR, "meshes_t_slot_option1"),
            "/meshes_env/": os.path.join(BUILD_DIR, "meshes_option1_dual_legs"),
        },
    },
    {
        "id": "12_option1_body",
        "step": "12",
        "title": "Payload Chassis, Load Cell & Picker",
        "category": "option1",
        "badge": "Subassembly",
        "viewer_module": "view_option1_body",
        "cad_files": ["cad/build_option1_body.py"],
        "sim_file": "sim/view_option1_body.py",
        "summary": "Non-swiveling payload chassis: column collar, 5 kg load-cell truss boom, belt-doubled telescopic picker, and battery enclosure.",
        "notes": "Includes interactive 0..100% Exploded Assembly View, live shelf-pick animation, flush battery lid, and knee gusset.",
        "uses": ["03_legacy_v3"],
        "used_by": ["13_option1_full"],
        "mounts": {"/meshes_body/": os.path.join(BUILD_DIR, "meshes_option1_body")},
    },
    {
        "id": "13_option1_full",
        "step": "13",
        "title": "Complete Full Robot + Racks & Leg Hoods",
        "category": "option1",
        "badge": "Latest Full Build",
        "viewer_module": "view_option1_full",
        "cad_files": ["cad/build_option1_full.py", "cad/build_t_slot_option1.py"],
        "sim_file": "sim/view_option1_full.py",
        "summary": "Complete dual-leg robot, compact 2-bay racks, bolted M5/M6 rail standoffs, and twin stationary protective leg hoods with outside M3 nuts.",
        "notes": "0 collisions verified across all static hardware, -15..-75 deg intermediate swivel sweeps, and all 5 operational poses.",
        "uses": ["10_option1_hub", "11_dual_legs", "12_option1_body", "04_4632_370", "05_td8125mg", "06_openbuilds_kit"],
        "used_by": [],
        "mounts": {
            "/meshes_pod/": os.path.join(BUILD_DIR, "meshes_t_slot_option1"),
            "/meshes_full/": os.path.join(BUILD_DIR, "meshes_option1_full"),
        },
    },
]

STAGE_BY_ID = {s["id"]: s for s in STAGES}
_VIEWER_HTML_CACHE = {}


def load_stage_html(stage: dict) -> str:
    sid = stage["id"]
    if sid in _VIEWER_HTML_CACHE:
        return _VIEWER_HTML_CACHE[sid]

    if stage.get("v1_archive"):
        raw_html = build_v1_three_html()
    elif "xml_path" in stage:
        raw_html = build_mujoco_three_html(stage["xml_path"], stage["title"], stage["xml_sub"], stage.get("traj", ""))
    elif stage["viewer_module"] is None:
        raw_html = OPENBUILD_VIEWER_HTML
    else:
        mod_name = stage["viewer_module"]
        mod_path = os.path.join(HERE, f"{mod_name}.py")
        spec = importlib.util.spec_from_file_location(f"_dash_{mod_name}", mod_path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        raw_html = mod.HTML

    prefix = f"/stage/{sid}"
    # Rewrite root-relative STL routes (/meshes/, /meshes_pod/, /meshes_env/, /meshes_body/, /meshes_full/)
    rewritten = re.sub(
        r"""(['"])/(meshes(?:_[a-z]+)?/|traj/)""",
        rf"\1{prefix}/\2",
        raw_html,
    )
    _VIEWER_HTML_CACHE[sid] = rewritten
    return rewritten


def collect_stage_manifest() -> list:
    out = []
    for s in STAGES:
        stls = []
        for mount_url, folder in s["mounts"].items():
            if os.path.isdir(folder):
                for fname in sorted(os.listdir(folder)):
                    if fname.lower().endswith(".stl"):
                        stls.append(f"/stage/{s['id']}{mount_url}{fname}")
        item = {k: v for k, v in s.items() if k not in ("mounts", "viewer_module")}
        item["stl_count"] = len(stls)
        item["stl_urls"] = stls
        out.append(item)
    return out


DASHBOARD_HTML = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>RackLift5 · All-in-One Evolution Timeline Dashboard</title>
<style>
  :root{
    --bg:#090e1a;--panel:#0f172a;--panel2:#131d33;--border:#24324a;--text:#e2e8f0;--muted:#94a3b8;
    --accent:#38bdf8;--green:#10b981;--amber:#f59e0b;
  }
  *{box-sizing:border-box;margin:0;padding:0}
  html,body{height:100%;background:var(--bg);color:var(--text);font:13px/1.4 -apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif;overflow:hidden}
  .main{display:flex;height:100vh;width:100vw}

  /* Side-only Navigation & Timeline */
  aside{width:328px;background:var(--panel);border-right:1px solid var(--border);display:flex;flex-direction:column;flex-shrink:0;transition:width .2s}
  aside.collapsed{width:0;overflow:hidden;border-right:none}
  .side-top{padding:11px 12px 9px;border-bottom:1px solid var(--border);display:flex;flex-direction:column;gap:8px;flex-shrink:0}
  .brand-row{display:flex;align-items:center;justify-content:space-between;gap:8px}
  .brand{display:flex;align-items:center;gap:8px;min-width:0}
  .logo{width:26px;height:26px;border-radius:7px;background:linear-gradient(135deg,#38bdf8,#6366f1);display:grid;place-items:center;font-weight:800;color:#fff;font-size:12px;flex-shrink:0}
  .brand h1{font-size:13px;font-weight:700;color:#f8fafc;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
  .brand span{display:block;font-size:10.5px;color:var(--muted)}

  .tool-row{display:flex;align-items:center;gap:5px;flex-wrap:wrap}
  .abtn{background:#182338;color:#e2e8f0;border:1px solid var(--border);border-radius:6px;padding:4px 7px;font-size:11px;cursor:pointer;text-decoration:none;display:inline-flex;align-items:center;gap:4px}
  .abtn:hover{border-color:var(--accent);color:#fff}
  .abtn.on{background:#312e81;border-color:#818cf8;color:#fff}

  .status-line{display:flex;align-items:center;gap:6px;font-size:10.5px;color:#94a3b8}
  .dot{width:7px;height:7px;border-radius:50%;background:var(--amber);animation:pulse 1.2s infinite;flex-shrink:0}
  .dot.ready{background:var(--green);animation:none}
  @keyframes pulse{0%,100%{opacity:.4}50%{opacity:1}}

  .filters{display:grid;grid-template-columns:1fr 1fr;gap:4px}
  .fbtn{background:#152036;color:#cbd5e1;border:1px solid var(--border);border-radius:6px;padding:4px 7px;font-size:10.5px;cursor:pointer;text-align:center;transition:.15s}
  .fbtn:hover{border-color:#475569;color:#fff}
  .fbtn.active{background:#0284c7;border-color:#38bdf8;color:#fff;font-weight:600}

  .timeline{flex:1;overflow-y:auto;padding:10px 10px 20px 18px;position:relative}
  .timeline::before{content:"";position:absolute;left:22px;top:14px;bottom:14px;width:2px;background:#22304a}
  .timeline::-webkit-scrollbar{width:5px}
  .timeline::-webkit-scrollbar-thumb{background:#293752;border-radius:3px}

  .card{position:relative;margin:0 0 9px 14px;padding:9px 10px;border-radius:9px;background:var(--panel2);border:1px solid var(--border);cursor:pointer;transition:.15s}
  .card::before{content:"";position:absolute;left:-15px;top:13px;width:9px;height:9px;border-radius:50%;background:#1e293b;border:2px solid #475569}
  .card.live::before{border-color:#22c55e}
  .card:hover{border-color:#4b648f}
  .card.active{background:#172542;border-color:#38bdf8;box-shadow:0 0 0 1px rgba(56,189,248,.28)}
  .card.active::before{background:#38bdf8;border-color:#e0f2fe}
  .card.hidden{display:none}
  .card-top{display:flex;align-items:center;justify-content:space-between;gap:6px;margin-bottom:3px}
  .card-step{font-family:ui-monospace,SFMono-Regular,Menlo,monospace;font-size:10.5px;color:var(--accent);font-weight:700}
  .tag{font-size:9.5px;padding:1.5px 6px;border-radius:999px;font-weight:600}
  .tag-legacy{background:rgba(148,163,184,.16);color:#cbd5e1;border:1px solid rgba(148,163,184,.35)}
  .tag-reusable{background:rgba(16,185,129,.16);color:#34d399;border:1px solid rgba(16,185,129,.35)}
  .tag-prototype{background:rgba(245,158,11,.16);color:#fbbf24;border:1px solid rgba(245,158,11,.35)}
  .tag-option1{background:rgba(99,102,241,.2);color:#a5b4fc;border:1px solid rgba(129,140,248,.4)}
  .card h3{font-size:12.5px;font-weight:600;color:#f8fafc;margin-bottom:3px}
  .card p{font-size:11px;color:#94a3b8;line-height:1.35;margin-bottom:5px}
  .meta-row{display:flex;align-items:center;justify-content:space-between;font-size:10px;color:#64748b;gap:6px}
  .meta-row code{color:#94a3b8;font-size:10px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}

  .card-extra{display:none;margin-top:7px;padding-top:7px;border-top:1px solid #263754}
  .card.active .card-extra{display:block}
  .card-notes{font-size:10.5px;color:#cbd5e1;line-height:1.35;margin-bottom:6px}
  .dep-wrap{display:flex;flex-wrap:wrap;gap:4px}
  .dep-chip{font-size:10px;padding:2px 6px;border-radius:5px;background:#0f172a;border:1px solid #2b3c5a;color:#cbd5e1;cursor:pointer}
  .dep-chip:hover{border-color:var(--accent);color:#fff}
  .dep-chip b{color:#7dd3fc;font-weight:600}

  /* Full-height 100vh 3D Viewport */
  .stage-area{position:relative;flex:1;min-width:0;height:100vh;background:#070b14;overflow:hidden}
  .stage-frame{position:absolute;inset:0;width:100%;height:100%;border:none;background:#090e1a;visibility:hidden;pointer-events:none;z-index:1}
  .stage-frame.active{visibility:visible;pointer-events:auto;z-index:5}
  .loading-overlay{position:absolute;inset:0;display:flex;flex-direction:column;align-items:center;justify-content:center;gap:10px;background:#090e1a;color:#94a3b8;z-index:10;pointer-events:none;transition:opacity .2s}
  .loading-overlay.gone{opacity:0}
  .spinner{width:28px;height:28px;border:3px solid #1e293b;border-top-color:#38bdf8;border-radius:50%;animation:spin .8s linear infinite}
  @keyframes spin{to{transform:rotate(360deg)}}

  .reopen-tab{position:absolute;left:0;top:50%;transform:translateY(-50%);z-index:20;background:rgba(15,23,42,.92);color:#e2e8f0;
              border:1px solid var(--border);border-left:none;border-radius:0 8px 8px 0;padding:10px 6px;font-size:11px;cursor:pointer;display:none}
  .reopen-tab:hover{border-color:var(--accent);color:#fff}
  aside.collapsed + .stage-area .reopen-tab{display:block}
</style>
</head>
<body>
<div class="main">
  <aside id="aside">
    <div class="side-top">
      <div class="brand-row">
        <div class="brand">
          <div class="logo">R5</div>
          <div>
            <h1>RackLift5 Timeline</h1>
            <span id="stageCounter">13 Builds · v1 → Full Robot</span>
          </div>
        </div>
        <button id="collapseBtn" class="abtn" title="Hide sidebar">◀</button>
      </div>

      <div class="filters">
        <button class="fbtn active" data-filter="all">All (13)</button>
        <button class="fbtn" data-filter="legacy">Legacy (3)</button>
        <button class="fbtn" data-filter="reusable">Reusable (3)</button>
        <button class="fbtn" data-filter="prototype">Prototypes (3)</button>
        <button class="fbtn" style="grid-column:span 2" data-filter="option1">Full System (4)</button>
      </div>

      <div class="tool-row">
        <button id="prevBtn" class="abtn" title="Previous stage (Left arrow)">◀ Prev</button>
        <button id="nextBtn" class="abtn" title="Next stage (Right arrow)">Next ▶</button>
        <button id="poolBtn" class="abtn" title="Keep 1 live WebGL stage (lowest GPU/RAM) or cache up to 3 recent stages">Live: 1</button>
        <a id="popoutLink" class="abtn" target="_blank" rel="noopener" title="Open active stage in full tab">↗ Open</a>
      </div>

      <div class="status-line" title="Stages load dynamically on demand; hovering a card prefetches its STLs into browser cache">
        <span id="cacheDot" class="dot ready"></span>
        <span id="cacheText">On-demand · 1 WebGL active</span>
      </div>
    </div>

    <div id="timeline" class="timeline"></div>
  </aside>

  <section id="canvasStack" class="stage-area">
    <button id="reopenBtn" class="reopen-tab" title="Show timeline sidebar">▶</button>
    <div id="loadingOverlay" class="loading-overlay">
      <div class="spinner"></div>
      <span id="loadingText">Loading 3D workspace…</span>
    </div>
  </section>
</div>

<script>
let STAGES = [];
let activeId = null;
let activeFilter = 'all';
let maxLiveFrames = 1; // Dynamic on-demand: 1 active WebGL context by default (toggleable to 3-slot LRU)
const frames = new Map();
const lruOrder = [];
const prefetchedIds = new Set();

const asideEl = document.getElementById('aside');
const timelineEl = document.getElementById('timeline');
const canvasStack = document.getElementById('canvasStack');
const loadingOverlay = document.getElementById('loadingOverlay');
const loadingText = document.getElementById('loadingText');
const cacheDot = document.getElementById('cacheDot');
const cacheText = document.getElementById('cacheText');
const poolBtn = document.getElementById('poolBtn');

function tagClass(cat) {
  if (cat === 'legacy') return 'tag-legacy';
  if (cat === 'reusable') return 'tag-reusable';
  if (cat === 'prototype') return 'tag-prototype';
  return 'tag-option1';
}

function updateLiveIndicators() {
  document.querySelectorAll('.card').forEach((el) => {
    el.classList.toggle('live', frames.has(el.dataset.id));
  });
  cacheText.textContent = `On-demand · ${frames.size} WebGL active · ${prefetchedIds.size}/${STAGES.length} cached`;
}

async function prefetchStageStls(id) {
  if (prefetchedIds.has(id)) return;
  const st = STAGES.find((s) => s.id === id);
  if (!st) return;
  prefetchedIds.add(id);
  const queue = [...st.stl_urls];
  const workers = Array.from({ length: Math.min(6, queue.length || 1) }, async () => {
    while (queue.length) {
      const url = queue.shift();
      try { await fetch(url); } catch (_) {}
    }
  });
  await Promise.all(workers);
  updateLiveIndicators();
}

function touchLru(id) {
  const idx = lruOrder.indexOf(id);
  if (idx >= 0) lruOrder.splice(idx, 1);
  lruOrder.push(id);
  while (lruOrder.length > maxLiveFrames) {
    const evictId = lruOrder.shift();
    if (evictId === activeId) {
      lruOrder.push(evictId);
      break;
    }
    const fr = frames.get(evictId);
    if (fr) {
      fr.src = 'about:blank';
      fr.remove();
      frames.delete(evictId);
    }
  }
  updateLiveIndicators();
}

function ensureFrame(id) {
  if (frames.has(id)) {
    touchLru(id);
    return frames.get(id);
  }
  const fr = document.createElement('iframe');
  fr.className = 'stage-frame';
  fr.dataset.id = id;
  fr.src = '/stage/' + id + '/';
  fr.addEventListener('load', () => {
    if (id === activeId) loadingOverlay.classList.add('gone');
  });
  canvasStack.appendChild(fr);
  frames.set(id, fr);
  prefetchedIds.add(id);
  touchLru(id);
  return fr;
}

function selectStage(id) {
  const st = STAGES.find((s) => s.id === id);
  if (!st) return;
  activeId = id;
  window.location.hash = id;

  document.querySelectorAll('.card').forEach((c) => c.classList.toggle('active', c.dataset.id === id));
  const activeCard = document.querySelector(`.card[data-id="${id}"]`);
  if (activeCard) activeCard.scrollIntoView({ block: 'nearest', behavior: 'smooth' });

  document.getElementById('popoutLink').href = `/stage/${id}/`;

  if (!frames.has(id)) {
    loadingText.textContent = `Loading ${st.step} · ${st.title} (${st.stl_count} STLs)…`;
    loadingOverlay.classList.remove('gone');
  } else {
    loadingOverlay.classList.add('gone');
  }

  const targetFr = ensureFrame(id);
  frames.forEach((fr, fid) => {
    fr.classList.toggle('active', fid === id);
  });
  if (targetFr.contentDocument && targetFr.contentDocument.readyState === 'complete') {
    loadingOverlay.classList.add('gone');
  }
}

function applyFilter(filter) {
  activeFilter = filter;
  document.querySelectorAll('.fbtn').forEach((b) => b.classList.toggle('active', b.dataset.filter === filter));
  let shown = 0;
  STAGES.forEach((st) => {
    const match = filter === 'all' || st.category === filter;
    document.querySelector(`.card[data-id="${st.id}"]`)?.classList.toggle('hidden', !match);
    if (match) shown++;
  });
  document.getElementById('stageCounter').textContent = `${shown} Builds · Parts → Full Robot`;
  if (activeId) {
    const cur = STAGES.find((s) => s.id === activeId);
    if (cur && filter !== 'all' && cur.category !== filter) {
      const first = STAGES.find((s) => s.category === filter);
      if (first) selectStage(first.id);
    }
  }
}

async function init() {
  const res = await fetch('/api/stages');
  STAGES = await res.json();

  STAGES.forEach((st) => {
    const card = document.createElement('div');
    card.className = 'card';
    card.dataset.id = st.id;

    const depChips = [
      ...st.uses.map((tid) => ({ lbl: 'Uses', tid })),
      ...st.used_by.map((tid) => ({ lbl: 'Used by', tid })),
    ].map(({ lbl, tid }) => {
      const t = STAGES.find((x) => x.id === tid);
      return t ? `<button class="dep-chip" data-jump="${tid}">${lbl}: <b>${t.step} ${t.title.split(' ')[0]}</b></button>` : '';
    }).join('');

    card.innerHTML = `
      <div class="card-top">
        <span class="card-step">STAGE ${st.step}</span>
        <span class="tag ${tagClass(st.category)}">${st.badge}</span>
      </div>
      <h3>${st.title}</h3>
      <p>${st.summary}</p>
      <div class="meta-row">
        <code>${st.cad_files.join(', ')}</code>
        <span>${st.stl_count} STLs</span>
      </div>
      <div class="card-extra">
        <div class="card-notes">${st.notes}</div>
        ${depChips ? `<div class="dep-wrap">${depChips}</div>` : ''}
      </div>
    `;
    let hoverTimer = null;
    card.addEventListener('mouseenter', () => {
      hoverTimer = setTimeout(() => prefetchStageStls(st.id), 90);
    });
    card.addEventListener('mouseleave', () => {
      if (hoverTimer) { clearTimeout(hoverTimer); hoverTimer = null; }
    });
    card.addEventListener('click', (e) => {
      const jump = e.target.closest('button[data-jump]');
      if (jump) {
        e.stopPropagation();
        selectStage(jump.dataset.jump);
        return;
      }
      selectStage(st.id);
    });
    timelineEl.appendChild(card);
  });

  const hashId = window.location.hash.replace('#', '');
  const initialId = STAGES.some((s) => s.id === hashId) ? hashId : '13_option1_full';
  selectStage(initialId);
}

document.querySelectorAll('.fbtn').forEach((btn) => {
  btn.addEventListener('click', () => applyFilter(btn.dataset.filter));
});

document.getElementById('prevBtn').addEventListener('click', () => {
  const vis = STAGES.filter((s) => activeFilter === 'all' || s.category === activeFilter);
  const idx = vis.findIndex((s) => s.id === activeId);
  if (idx > 0) selectStage(vis[idx - 1].id);
});
document.getElementById('nextBtn').addEventListener('click', () => {
  const vis = STAGES.filter((s) => activeFilter === 'all' || s.category === activeFilter);
  const idx = vis.findIndex((s) => s.id === activeId);
  if (idx >= 0 && idx < vis.length - 1) selectStage(vis[idx + 1].id);
});
window.addEventListener('keydown', (e) => {
  if (e.target.tagName === 'INPUT') return;
  if (e.key === 'ArrowLeft') document.getElementById('prevBtn').click();
  if (e.key === 'ArrowRight') document.getElementById('nextBtn').click();
});

document.getElementById('collapseBtn').addEventListener('click', () => asideEl.classList.add('collapsed'));
document.getElementById('reopenBtn').addEventListener('click', () => asideEl.classList.remove('collapsed'));

poolBtn.addEventListener('click', () => {
  maxLiveFrames = maxLiveFrames === 1 ? 3 : 1;
  poolBtn.textContent = maxLiveFrames === 3 ? 'Keep: 3' : 'Live: 1';
  poolBtn.classList.toggle('on', maxLiveFrames === 3);
  if (activeId) touchLru(activeId);
});

init();
</script>
</body>
</html>
"""


def make_handler(manifest: list):
    manifest_bytes = json.dumps(manifest).encode("utf-8")

    class DashboardHandler(http.server.BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def _send_bytes(self, body: bytes, ctype: str, cache_control: str):
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", cache_control)
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = urllib.parse.urlparse(self.path).path

            if path == "/" or path == "/index.html":
                return self._send_bytes(DASHBOARD_HTML.encode("utf-8"), "text/html; charset=utf-8", "no-cache")

            if path == "/api/stages":
                return self._send_bytes(manifest_bytes, "application/json; charset=utf-8", "no-cache")

            if path.startswith("/stage/"):
                rest = path[len("/stage/"):]
                parts = rest.split("/", 1)
                sid = parts[0]
                sub = parts[1] if len(parts) > 1 else ""
                stage = STAGE_BY_ID.get(sid)
                if not stage:
                    self.send_error(404, "Unknown stage")
                    return

                if sub == "" or sub == "index.html":
                    html = load_stage_html(stage)
                    return self._send_bytes(html.encode("utf-8"), "text/html; charset=utf-8", "no-cache")

                # Match mounted STL directories (/meshes/, /meshes_pod/, /meshes_env/, /meshes_body/, /meshes_full/)
                sub_slash = "/" + sub
                for mount_prefix, folder in stage["mounts"].items():
                    if sub_slash.startswith(mount_prefix):
                        rel = sub_slash[len(mount_prefix):]
                        safe_dir = os.path.realpath(folder)
                        fpath = os.path.realpath(os.path.join(safe_dir, rel))
                        if not fpath.startswith(safe_dir + os.sep) or not os.path.isfile(fpath):
                            self.send_error(404, "Mesh not found")
                            return
                        with open(fpath, "rb") as f:
                            data = f.read()
                        return self._send_bytes(data, "application/octet-stream", "public, max-age=3600")

            self.send_error(404)

    return DashboardHandler


def main():
    parser = argparse.ArgumentParser(description="RackLift5 All-in-One Evolution Timeline Dashboard")
    parser.add_argument("--port", type=int, default=8860, help="Starting TCP port to bind (default 8860)")
    parser.add_argument("--no-open", action="store_true", help="Do not open browser automatically")
    args = parser.parse_args()

    manifest = collect_stage_manifest()
    handler_cls = make_handler(manifest)
    socketserver.ThreadingTCPServer.allow_reuse_address = True
    srv = None
    bound_port = args.port
    for port in range(args.port, args.port + 20):
        try:
            srv = http.server.ThreadingHTTPServer(("127.0.0.1", port), handler_cls)
            bound_port = port
            break
        except OSError:
            continue
    if srv is None:
        raise RuntimeError(f"No free port in {args.port}..{args.port + 19}")

    url = f"http://127.0.0.1:{bound_port}/"
    total_stls = sum(m["stl_count"] for m in manifest)
    print(f"RackLift5 All-in-One Timeline Dashboard ({len(manifest)} stages, {total_stls} STLs) -> {url}")
    if not args.no_open:
        threading.Timer(0.6, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
