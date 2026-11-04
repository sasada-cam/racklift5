"""RackLift5 v2 -> exact B-rep CAD files (build123d / OpenCascade).

    python cad/export_brep.py            # everything
    python cad/export_brep.py parts      # only the printable parts
    python cad/export_brep.py reach      # only one assembly pose (reach | travel | turning)

Output (models/):
  print/<name>.step + .stl         print orientation, on the bed (z=0), centred in x/y  -> slicer
  design/<name>.step               the part in its own design frame (axes of params.py)  -> FEA
  assembly/RackLift5_v2_<pose>.step  every item positioned, named and coloured, grouped
                                     Rack+rails / Junction turntables / Robot body / Hand+tray / SCARA arm
  parts_report.csv                 volume, PETG mass, size, face count, validity per part
Edit generator/params.py and re-run to regenerate everything.
"""
import os, sys, csv, time
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "brep"))
import racklift_brep as R
import build123d as bd
from csg import Solid

OUT = os.path.abspath(os.path.join(HERE, "..", "models"))
PETG_DENSITY = 1.27e-3          # g/mm^3
GROUP_LABEL = {"static": "Rack + rails (fixed)", "rotor": "Junction turntables", "body": "Robot body",
               "tray": "Hand + tray", "arm": "SCARA arm"}


def ensure(d):
    os.makedirs(d, exist_ok=True); return d


def to_bed(s: Solid, M):
    s = s.transform(M)
    lo, hi = s.bbox()
    return s.translate(-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2])


def export_parts():
    dp, dd = ensure(os.path.join(OUT, "print")), ensure(os.path.join(OUT, "design"))
    rows = []
    for name, (fn, M, qty, grp) in R.PRINTED.items():
        t = time.time()
        s = fn()
        sh = s.shape; sh.label = name
        bd.export_step(sh, os.path.join(dd, name + ".step"))
        p = to_bed(s, M).shape; p.label = name
        bd.export_step(p, os.path.join(dp, name + ".step"))
        bd.export_stl(p, os.path.join(dp, name + ".stl"), tolerance=0.005, angular_tolerance=0.1)
        bb = p.bounding_box()
        rows.append(dict(group=grp, part=name, qty=qty, valid=sh.is_valid, solids=len(sh.solids()),
                         faces=len(sh.faces()), volume_mm3=round(sh.volume, 1),
                         petg_g_solid=round(sh.volume * PETG_DENSITY, 1),
                         size_x=round(bb.size.X, 1), size_y=round(bb.size.Y, 1), size_z=round(bb.size.Z, 1)))
        print(f"{name:28s} ok  {time.time() - t:.1f}s", flush=True)
    with open(os.path.join(OUT, "parts_report.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)


def build_pose(pose):
    A, p = R.A, R.params
    B_all = [(b, l) for b in range(2) for l in range(5)]
    others = [x for x in B_all if x != (0, 2)]
    if pose == "reach":
        return A.build((p.RAIL_X[0], p.LEVELS[2] - p.TRAY_TOP_REL), 0, [0, -90], p.BASKET_Y0 + 15, B_all)
    if pose == "travel":
        return A.build(((p.RAIL_X[0] + p.RAIL_X[1]) / 2, p.JZ), -90, [-90, -90], -178.0, others + ['tray'])
    if pose == "turning":
        return A.build(p.JUNCTIONS[0], -45, [-45, -90], -178.0, others + ['tray'])
    raise ValueError(pose)


def export_assembly(pose):
    t = time.time()
    items = build_pose(pose)
    groups = {}
    count = {}
    for name, s, col, g in items:
        if s.shape is None:
            continue
        sh = s.shape
        count[name] = count.get(name, 0) + 1
        sh.label = f"{name} {count[name]}" if sum(1 for n, *_ in items if n == name) > 1 else name
        sh.color = bd.Color(col[0] / 255, col[1] / 255, col[2] / 255)
        groups.setdefault(g, []).append(sh)
    subs = [bd.Compound(children=v, label=GROUP_LABEL.get(g, g)) for g, v in groups.items()]
    asm = bd.Compound(children=subs, label=f"RackLift5_v2_{pose}")
    path = os.path.join(ensure(os.path.join(OUT, "assembly")), f"RackLift5_v2_{pose}.step")
    bd.export_step(asm, path)
    n = sum(len(v) for v in groups.values())
    print(f"assembly {pose}: {n} bodies -> {path}  ({time.time() - t:.1f}s)", flush=True)


if __name__ == "__main__":
    what = sys.argv[1:] or ["parts", "reach", "travel", "turning"]
    if "parts" in what:
        export_parts()
    for pose in ("reach", "travel", "turning"):
        if pose in what:
            export_assembly(pose)
