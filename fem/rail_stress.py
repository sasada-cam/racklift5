"""v3 rail stress test: V-slot rail (6063-T5) under hub wheel loads, CalculiX (2nd-order tets).

pixi run python fem/rail_stress.py [case ...]      cases: H324 V694   (default: both)
Pipeline: build123d profile -> STEP -> gmsh.exe (tet10) -> own CalculiX .inp -> ccx -> .dat -> report (fem/out/).
Geometry = generator/assembly_v2.vslot_z profile (20x20, 6.2 mm slots, 6 mm deep). Units mm, N, MPa.
"""
import os, sys, re, json, subprocess, shutil
import numpy as np
import meshio
from build123d import Polygon, extrude, Plane, export_step

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out"); os.makedirs(OUT, exist_ok=True)
GMSH = os.path.join(os.path.dirname(sys.executable), "Library", "bin", "gmsh.exe")

E, NU, YIELD = 69000.0, 0.33, 110.0          # 6063-T5 (min yield ~110 MPa, UTS ~150)
H, S, D = 10.0, 3.1, 6.0
PROFILE = [(-H, -H), (-S, -H), (-S, -H + D), (S, -H + D), (S, -H), (H, -H), (H, -S), (H - D, -S), (H - D, S), (H, S),
           (H, H), (S, H), (S, H - D), (-S, H - D), (-S, H), (-H, H), (-H, S), (-H + D, S), (-H + D, -S), (-H, -S)]

# --- loads (N, per rail). 2 kg payload from `sim/legs.py --v3`: worst wheel 24 N, total weight (3.5+2) kg.
W_NOM = (3.5 + 2.0) * 9.81            # 54 N
FZ_NOM, FY_NOM = W_NOM / 2, 24.0      # per rail: share of weight, rail twist force (legs.py worst case)
LOADCASES = {"nominal (2 kg payload)": (1.0, 1.0), "dynamic x3 (jolt / e-stop)": (3.0, 3.0)}
LATERAL_ABUSE = 200.0                 # N, a person leans on / knocks the rail at its worst point

def build_step(L, path):
    pts = [(a, b) for a, b in PROFILE]
    solid = extrude(Polygon(*pts, align=None), L)    # profile in XY, extruded along +Z, centred on origin
    export_step(solid, path)

def mesh(step, msh, clmax):
    r = subprocess.run([GMSH, step, "-3", "-order", "2", "-clmax", str(clmax), "-clmin", "1.0", "-format", "msh2",
                        "-o", msh, "-v", "1"], capture_output=True, text=True)
    if r.returncode: raise SystemExit(r.stdout + r.stderr)
    m = meshio.read(msh)
    tets = m.cells_dict["tetra10"]
    return m.points, tets

def nodes_in(P, box):
    lo, hi = box
    return np.where(np.all((P >= lo - 1e-6) & (P <= hi + 1e-6), axis=1))[0]

def write_inp(path, P, T, fixed, loads, name):
    """fixed: node ids (all 3 dof); loads: list of (node ids, (fx,fy,fz) total force spread evenly)"""
    with open(path, "w") as f:
        f.write("*NODE, NSET=NALL\n")
        for i, p in enumerate(P, 1): f.write(f"{i}, {p[0]:.5f}, {p[1]:.5f}, {p[2]:.5f}\n")
        f.write("*ELEMENT, TYPE=C3D10, ELSET=EALL\n")
        for i, t in enumerate(T, 1):
            t = list(t)                                   # meshio already gives VTK/Abaqus order = ccx order
            f.write(f"{i}, " + ", ".join(str(int(n) + 1) for n in t) + "\n")
        f.write("*NSET, NSET=FIX\n"); f.write(",\n".join(str(int(n) + 1) for n in fixed) + "\n")
        f.write(f"*MATERIAL, NAME=AL\n*ELASTIC\n{E}, {NU}\n*SOLID SECTION, ELSET=EALL, MATERIAL=AL\n")
        f.write("*BOUNDARY\nFIX, 1, 3\n*STEP\n*STATIC\n*CLOAD\n")
        for ids, F in loads:
            for n in ids:
                for d in range(3):
                    if F[d]: f.write(f"{int(n) + 1}, {d + 1}, {F[d] / len(ids):.6f}\n")
        f.write("*NODE PRINT, NSET=NALL\nU\n*EL PRINT, ELSET=EALL\nS\n*END STEP\n")

def run_ccx(job):
    r = subprocess.run(["ccx", "-i", job], cwd=OUT, capture_output=True, text=True)
    if "Job finished" not in r.stdout: raise SystemExit(r.stdout[-1500:] + r.stderr[-800:])

def read_dat(path, P, T):
    txt = open(path).read()
    u = {}; ip = []
    sec = None
    for ln in txt.splitlines():
        if "displacements" in ln: sec = "u"; continue
        if "stresses" in ln: sec = "s"; continue
        a = ln.split()
        if not a or not re.match(r"^\d+$", a[0]): continue
        if sec == "u" and len(a) == 4: u[int(a[0])] = [float(x) for x in a[1:]]
        elif sec == "s" and len(a) == 8: ip.append((int(a[0]), [float(x) for x in a[2:]]))
    U = np.array([u[i] for i in range(1, len(P) + 1)])
    s = np.array([x[1] for x in ip]); el = np.array([x[0] for x in ip])
    sxx, syy, szz, sxy, sxz, syz = s.T
    vm = np.sqrt(0.5 * ((sxx - syy) ** 2 + (syy - szz) ** 2 + (szz - sxx) ** 2) + 3 * (sxy ** 2 + sxz ** 2 + syz ** 2))
    # element-centroid position for each integration point (4 per C3D10) -> use element centroid
    cen = P[T[:, :4]].mean(axis=1)[el - 1]
    return U, vm, cen

def section_props():
    """I about both axes of the modelled profile (numerical polygon moment)"""
    p = np.array(PROFILE); x, y = p[:, 0], p[:, 1]; x1, y1 = np.roll(x, -1), np.roll(y, -1)
    c = x * y1 - x1 * y; A = c.sum() / 2
    Ix = (c * (y ** 2 + y * y1 + y1 ** 2)).sum() / 12; Iy = (c * (x ** 2 + x * x1 + x1 ** 2)).sum() / 12
    return abs(A), abs(Ix), abs(Iy)

def analyse(name, L, supports, loads_def, clmax=3.5):
    """rail along z from 0..L (centre on x,y=0). supports: list of z positions (bolt patch 10 mm, all dof, on +y face).
    loads_def: list of (z, face, force-vector) with 'face' in {'top' (+x? no) ...}; patches 8x8 mm on the lip of the face"""
    step = os.path.join(OUT, f"{name}.step"); msh = os.path.join(OUT, f"{name}.msh")
    build_step(L, step)
    P, T = mesh(step, msh, clmax)
    print(f"[{name}] mesh: {len(P)} nodes, {len(T)} tet10", flush=True)
    res = {}
    fixed = np.unique(np.concatenate([nodes_in(P, (np.array([-H, H - 1e-3, z - 5]), np.array([H, H + 1e-3, z + 5])))
                                      for z in supports]))
    assert len(fixed), "no support nodes"
    for lcname, loads_list in loads_def.items():
        loads = []
        for z, face, F in loads_list:                     # patches on the lip (offset 6.5 from the slot axis), 8 mm along rail
            ids = []
            for off in (-6.5, 6.5):
                if face == "front":    # y = -H face: wheel flange pushing +y
                    box = (np.array([off - 3.5, -H - 1e-3, z - 4]), np.array([off + 3.5, -H + 1e-3, z + 4]))
                else:                  # x = +H face (top when the rail is horizontal): load pushes -x
                    box = (np.array([H - 1e-3, off - 3.5, z - 4]), np.array([H + 1e-3, off + 3.5, z + 4]))
                n = nodes_in(P, box); assert len(n), (face, z, off); ids.append(n)
            ids = np.unique(np.concatenate(ids)); loads.append((ids, F))
        job = f"{name}_{re.sub(r'[^a-z0-9]+', '_', lcname.lower()).strip('_')}"
        write_inp(os.path.join(OUT, job + ".inp"), P, T, fixed, loads, job)
        run_ccx(job)
        U, vm, cen = read_dat(os.path.join(OUT, job + ".dat"), P, T)
        mag = np.linalg.norm(U, axis=1)
        # exclude 8 mm round supports + load patches (concentrated-force singularities)
        keep = np.ones(len(vm), bool)
        for z in supports: keep &= np.abs(cen[:, 2] - z) > 8
        for z, _, _ in loads_list: keep &= np.abs(cen[:, 2] - z) > 8
        res[lcname] = dict(max_disp_mm=float(mag.max()), vm_max_all_MPa=float(vm.max()), vm_max_away_MPa=float(vm[keep].max()),
                           sf_yield_away=float(YIELD / vm[keep].max()), sf_yield_all=float(YIELD / vm.max()))
        meshio.write(os.path.join(OUT, job + ".vtu"), meshio.Mesh(P, [("tetra10", T)], point_data={"U": U}))
        print(f"  {lcname:30s} umax {mag.max():.4f} mm   VM away from loads {vm[keep].max():6.2f} MPa (SF {YIELD / vm[keep].max():5.1f}) "
              f"| incl. contact patches {vm.max():6.2f} MPa (SF {YIELD / vm.max():4.1f})", flush=True)
    return res

if __name__ == "__main__":
    want = sys.argv[1:] or ["H324", "V694"]
    A, Ix, Iy = section_props()
    print(f"profile: A={A:.1f} mm2  Ix={Ix:.0f} mm4  Iy={Iy:.0f} mm4  (modelled slotted section; stock 2020 extrusion is stiffer, so this is conservative)")
    results = {"section": dict(A=A, Ix=Ix, Iy=Iy), "material": dict(E=E, yield_MPa=YIELD)}
    # model frame: rail axis = z. face 'front' = y-min (tray side, carries the rack); face 'top' = x-max.
    if "H324" in want:
        L = 324.0; zm = L / 2
        # horizontal rail 324 mm: bolted to the crossing brackets at both ends (support patch on the +y (back) face)
        # wheel loads at worst position (midspan), two wheels 45 mm either side; vertical share on x-max face, twist on front face
        def lc(fs, fy_):
            return [(zm - 45, "top", (-fs * FZ_NOM / 2, 0, 0)), (zm + 45, "top", (-fs * FZ_NOM / 2, 0, 0)),
                    (zm - 45, "front", (0, fy_ * FY_NOM / 2, 0)), (zm + 45, "front", (0, fy_ * FY_NOM / 2, 0))]
        loads = {k: lc(*v) for k, v in LOADCASES.items()}
        loads["lateral abuse 200 N at midspan"] = [(zm, "front", (0, LATERAL_ABUSE, 0))]
        results["H324"] = analyse("H324", L, [8.0, L - 8.0], loads)
    if "V694" in want:
        L = 694.0
        # vertical rail: bolted to rail bridges at 3 heights (rail-local z): crossing bracket ~ 0 (z=329 abs: rail starts 386),
        # bridges at abs 650 and 955; free top end (cantilever past the last bridge)
        sup = [5.0, 650 - 386.0, 955 - 386.0]              # crossing bracket at the foot, bridges at 264 and 569
        zu, zl = 417.0, 417.0 - 259.0
        def lcv(fs):
            return [(zu, "front", (0, fs * FY_NOM, 0)), (zl, "front", (0, -fs * FY_NOM, 0))]
        loads = {"nominal (2 kg payload)": lcv(1.0), "dynamic x3 (jolt / e-stop)": lcv(3.0),
                 "lateral abuse 200 N at midspan": [(417.0, "front", (0, LATERAL_ABUSE, 0))]}
        results["V694"] = analyse("V694", L, sup, loads)
    json.dump(results, open(os.path.join(OUT, "results.json"), "w"), indent=1)
