"""RackLift5 v3: EVERY unique component of the latest rack + robot as its own CAD file, plus a parts list (BOM).

    python cad/export_v3_components.py

Output (all in mm):
    models/v3/components/<category>/<ID>_<name>.step|.stl   one file per unique part, bounding-box corner at the origin
    models/v3/components/BOM_v3.csv                          ID, category, part, qty, size, volume, suggested process
    models/v3/components/README.md                           the same list as a table

Bought parts (motors, servos, bearings, extrusions, wheels, belts, the basket ...) are simplified envelopes of the real part, as in
the assembly.  Quantities are counted from the full assembly (3 rows x 4 racks, 3 docks, 1 cart, 1 robot).  The assembly itself is
exported by export_v3.py (models/v3/assembly/*.step); this script gives the individual parts.
"""
import collections, csv, os, re, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "brep"))
sys.path.insert(0, HERE)
import racklift_brep as R          # noqa: E402  loads the B-rep csg backend + v2 modules
import assembly_v3 as A3           # noqa: E402
import export_brep as E            # noqa: E402
import build123d as bd             # noqa: E402

OUT = os.path.abspath(os.path.join(HERE, "..", "models", "v3", "components"))

# ordered rules: first match wins
CATEGORY = [
    (r"^cover:", "Covers", "COV"),
    (r"^basket", "Baskets", "BAS"),
    (r"^jack:|raised floor", "Dock (jack + raised floor)", "DOC"),
    (r"lock solenoid|cart lock|cart docking bar", "Locks (robot + cart)", "LCK"),
    (r"^cart|trolley|floor guide|transfer|beam", "Transfer cart + beam", "CRT"),
    (r"slotted angle rack", "Rack", "RAC"),
    (r"crossing|turntable|rail piece|V-slot rail|in-slot rack|rail bridge|rail end", "Rails + crossings", "RAI"),
    (r"hub|V wheel|spacer|climber|swivel axle|column bar|swivel motor|battery", "Robot body + hubs", "ROB"),
    (r"telescopic|finger|stage|belt|servo|pulley|idler|cross shaft|arm|tray|load cell|GT2|limit switch|basket stop|turn|front post|lift carriage",
     "Tray + picker + drives", "PIC"),
]
PROCESS = [
    (r"^cover:", "3D print (PETG), 2.4 mm walls, closed face down"),
    (r"lock solenoid bracket|finger shaft bearing|pivot lug|belt anchor", "3D print (PETG)"),
    (r"column bar", "Aluminium bar 28 x 24 (cut to length, drill)"),
    (r"in-slot rack|transfer rack segment|climber pinion|turn drive pinion|transfer pinion", "Print fine (0.12 mm, 100 % infill) or order nylon (SLS)"),
    (r"printed", "3D print (PETG)"),
    (r"load cell|limit switch", "Buy (sensor / switch)"),
    (r"slotted angle rack", "Buy (slotted-angle rack)"),
    (r"plywood|raised floor: top", "Plywood (cut to size)"),
    (r"battens", "Timber battens (cut)"),
    (r"GT2 belt", "Buy (GT2 belt, cut to length)"),
    (r"^arm drive cross shaft|finger shaft: |bottom pivot shaft|top pivot shaft|centre pin shaft|roller axle|idler axle|climber shaft"
     r"|swivel axle|slider cross tube|output shaft|cart lock pin|solenoid bolt|solenoid plunger|spacer", "Steel rod / tube (cut, turn)"),
    (r"pulley|idler|finger drive gear|coupling", "Print, or buy a GT2 pulley / gear / coupling"),
    (r"servo \d bracket", "Aluminium plate / bar (laser cut, drill)"),
    (r"motor|servo|solenoid \(|stepper", "Buy (actuator)"),
    (r"bearing", "Buy (bearing)"),
    (r"lead screw", "Buy (TR8x2 lead screw)"),
    (r"roller guide bar|slider guide rail", "Steel / aluminium flat bar (cut, drill)"),
    (r"V wheel|V-wheels|roller", "Buy (wheel / roller)"),
    (r"PTFE", "Buy (PTFE pad)"),
    (r"telescopic arm", "Aluminium flat bar 13 x 8 (cut to length)"),
    (r"scissor arm", "Aluminium flat bar 14 x 8 (cut, drill)"),
    (r"V-slot|rail piece|beam|rail end arm|cart frame|guide rail|post", "Aluminium extrusion (cut to length)"),
    (r"plate|bracket", "Aluminium plate / bar (laser cut, drill)"),
    (r"battery", "Buy (battery + electronics box)"),
    (r"basket", "Buy (basket)"),
    (r"gear|pinion", "Print fine or order nylon (SLS)"),
]


def classify(name, table, default):
    for rule in table:
        if re.search(rule[0], name):
            return rule[1:] if len(rule) > 2 else rule[1]
    return default


def base_name(n):
    return re.sub(r" \((row|hub|dock|corner|turret|face) \d+\)$", "", n)


def slug(n):
    s = re.sub(r"[^A-Za-z0-9]+", "_", n).strip("_")
    return s[:52]


def sources():
    """canonical-orientation parts from the local-frame builders of assembly_v3"""
    A = A3
    out = []

    def add(items):
        for it in items:
            out.append((it[0], it[1]))
    for f in (A.robot_covers_local, A.dock_cover_local, A.row_static_local, A.xfer_static, A.dock_static_local, A.crossing_local, A.cart_local, A.cart_lock_local,
              A.hub_local, A.hub_pinion_local, A.robot_local, A.arm_drive_local, A.idler_local, A.servo1_local, A.fservo_local,
              A.arm_mid_local, A.arm_in_local, A.turn_carriage_local, A.turn_pinion_local, A.dock_bolt_local,
              A.release_plunger_local, A.jack_platform_local, A.jack_arm1_local, A.jack_arm2_local, A.jack_slider_local,
              A.jack_screw_local):
        add(f())
    for sx in A.SIDES:
        add(A.fshaft_local(sx)); add(A.ftube_mid_local(sx)); add(A.finger_local(sx))
        for k in (0, 1):
            add(A.dpulley_local(sx, k))
    return out


def key_of(name, solid):
    return (base_name(name), round(solid.volume / 1000.0, 3))


def main():
    t0 = time.time()
    os.makedirs(OUT, exist_ok=True)
    # counts from the full assembly (one pose has every row, dock, the cart and the robot)
    items = A3.pose("pick")
    count = collections.Counter()
    placed = {}
    for n, s, c, g in items:
        if s.shape is None or n.startswith("basket") or n.startswith("stage-2 belt"):
            continue                                    # baskets: one stand-in below; stage-2 belts are straight stand-ins
        k = key_of(n, s)
        count[k] += 1
        placed.setdefault(k, s)
    canon = {}
    for n, s in sources():
        if s.shape is None:
            continue
        canon.setdefault(key_of(n, s), s)
    rows = []
    for k, q in count.items():
        s = canon.get(k, placed[k])
        rows.append((k[0], s, q))
    bs = A3.basket_solid()
    rows.append(("basket (stand-in for the real basket)", bs, len(A3.BASKET_IDS)))
    # parts that are cut-to-fit and have no useful solid
    extras = [("GT2 belt, stage 2 (6 mm), cut to the loop length", "Tray + picker + drives", 4, "Buy (GT2 belt, cut to length)"),
              ("Cover screws: M3 x 16 chipboard (wood) screw, pan head - front drive cover into the plywood", "Covers", 2, "Buy (hardware)"),
              ("Cover screws: M3 x 12 machine screw, pan head - turn-drive cap (2) and the 3 jack motor covers (12)", "Covers", 14, "Buy (hardware)"),
              ("Cover nuts: M3 hex nut, standard or nylon-insert - caught in the printed slots / pockets", "Covers", 14, "Buy (hardware)"),
              ("Cover washers: M3 flat washer", "Covers", 14, "Buy (hardware)"),
              ("Fasteners: M3 / M4 / M5 screws, nuts, heat-set inserts, 4 mm dowel pins", "Hardware", 0, "Buy (assorted; count from the drawings)")]

    # number + export, grouped by category
    cats = collections.defaultdict(list)
    PREFIX = {}
    for name, s, q in rows:
        cat, pre = classify(name, CATEGORY, ("Robot body + hubs", "ROB"))
        PREFIX[cat] = pre
        cats[cat].append((name, s, q))
    bom = []
    n_files = 0
    for cat in sorted(cats):
        cdir = E.ensure(os.path.join(OUT, slug(cat)))
        for i, (name, s, q) in enumerate(sorted(cats[cat], key=lambda r: r[0]), 1):
            cid = f"{PREFIX[cat]}{i:02d}"
            lo, hi = s.bbox()
            part = s.translate(-lo[0], -lo[1], -lo[2])
            sh = part.shape
            sh.label = name
            fn = f"{cid}_{slug(name)}"
            printed = bool(re.search(r"printed|pinion|in-slot rack|pulley", name))
            bd.export_step(sh, os.path.join(cdir, fn + ".step"))
            bd.export_stl(sh, os.path.join(cdir, fn + ".stl"), tolerance=0.005 if printed else 0.05,
                          angular_tolerance=0.1 if printed else 0.4)
            n_files += 1
            size = hi - lo
            bom.append([cid, cat, name, q, f"{size[0]:.0f} x {size[1]:.0f} x {size[2]:.0f}", f"{s.volume / 1000:.1f}",
                        classify(name, PROCESS, "Buy / specify"), f"{slug(cat)}/{fn}.step"])
    for name, cat, q, proc in extras:
        bom.append(["-", cat, name, q if q else "as needed", "-", "-", proc, "-"])
    with open(os.path.join(OUT, "BOM_v3.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["ID", "category", "part", "qty", "size mm (x y z)", "volume cm3", "suggested process / source", "CAD file"])
        w.writerows(bom)
    with open(os.path.join(OUT, "README.md"), "w") as f:
        f.write("# RackLift5 v3 - component list\n\n"
                f"{n_files} unique parts, one STEP + STL each (mm, bounding-box corner at the origin; bought parts are simplified "
                "envelopes). Quantities are for the whole system: 3 rows x 4 racks, 3 docks, 1 cart, 1 robot. "
                "Generated by `cad/export_v3_components.py`; see also `models/v3/assembly/` for the assembled poses.\n")
        last = None
        for r in bom:
            if r[1] != last:
                f.write(f"\n## {r[1]}\n\n| ID | part | qty | size mm | process / source |\n|---|---|---|---|---|\n")
                last = r[1]
            f.write(f"| {r[0]} | {r[2]} | {r[3]} | {r[4]} | {r[6]} |\n")
    print(f"{n_files} unique components -> {OUT}  ({time.time() - t0:.0f}s)")
    for cat in sorted(cats):
        print(f"  {cat:34s} {len(cats[cat]):3d} parts, {sum(q for _, _, q in cats[cat]):4d} pieces")


if __name__ == "__main__":
    main()
