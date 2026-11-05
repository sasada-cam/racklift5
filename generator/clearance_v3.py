"""v3 collision check (compact Squid layout): hubs, column, battery, tray, telescopic arms and fingers against the rack, the rails and the
crossing turntables, and against each other, through every pick step, every hub swivel, climbing and travel.
    python clearance_v3.py [filter]"""
import sys, time; sys.path.insert(0, '.')
import numpy as np
import assembly_v3 as A3
from params_v3 import *
from interfere import interference, bb

SKIP = [("V wheel", "V-slot"), ("V wheel", "rail piece"), ("spacer", "V-slot"), ("spacer", "rail piece"),
        ("printed climber pinion", "printed in-slot rack"), ("printed climber pinion", "rail piece"),
        ("printed climber pinion", "V-slot"),
        ("swivel axle", "printed column"), ("swivel axle", "JGY-370 swivel motor"),
        # the finger shaft telescopes (rod in tube in tube) and runs through its bearings and the fingers' collars
        ("finger shaft", "finger shaft"), ("finger shaft", "printed front post"), ("printed flip finger", "finger shaft"),
        # belts lie on their pulleys / in their clamps and anchors; stage-2 pulleys turn on axles through the middle bar
        ("stage-2 belt", "stage-2 pulley"), ("stage-2 belt", "belt anchor"), ("stage-2 belt", "stage-2 belt"),
        ("stage-2 pulley", "telescopic arm, middle bar"), ("GT2 belt loop (stage 1)", "printed stage-1 belt clamp"),
        ("basket (moving)", "printed flip finger"),
        # the robot's pinion meshes the transfer rack; the cart rides its track and is guided on the rack beam
        ("turn drive pinion", "printed transfer rack"), ("cart wheels", "V-slot cart track"),
        ("cart guide arm", "transfer rack beam")]
FIXED = ("static", "rotor")


def skip(a, b):
    return any((a.startswith(x) and b.startswith(y)) or (b.startswith(x) and a.startswith(y)) for x, y in SKIP)


def check(items, h=1.0):
    T = [(n, s.triangles(), g) for n, s, c, g in items]
    boxes = {id(t): bb(t) for _, t, _ in T}
    hits = []

    def pair(a, b):
        (na, ta, _), (nb, tb, _) = a, b
        if skip(na, nb):
            return
        a0, a1 = boxes[id(ta)]; b0, b1 = boxes[id(tb)]
        if np.any(a1 < b0) or np.any(b1 < a0):
            return
        v, p = interference(ta, tb, h)
        if v > 0.5:
            hits.append((na, nb, round(v, 1), np.round(p, 0)))
    fixed = [x for x in T if x[2] in FIXED]
    mov = [x for x in T if x[2] not in FIXED]
    for m in mov:
        for f in fixed:
            pair(m, f)
    for i, a in enumerate(mov):
        for b in mov[i + 1:]:
            if a[2] != b[2] or a[0].startswith("basket (moving)") or b[0].startswith("basket (moving)"):
                if {a[0].split(" (")[0], b[0].split(" (")[0]} & {"basket", "plywood tray 12 mm"} and "moving" not in a[0] + b[0]:
                    continue
                pair(a, b)
    return hits


def cases():
    """floor of racks: picks (row 1 bay 1 at every shelf, a middle bay, the last bay of row 2), hub swivels + climb,
    travel along every row, onto the transfer cart and across to the other rows"""
    B = [(b, l) for b in range(N_BAYS_ALL) for l in range(len(LEVELS))]
    S = A3.state
    out = []

    def rv(j, lo, up):
        return [[lo, up] if k == j else [-90.0, -90.0] for k in range(N_BAYS_ALL)]

    def picks(j, levels, F):
        jx = A3.bay_rail_x(j)
        V = rv(j, 0.0, 0.0)
        for l in levels:
            cz = cz_for_level(l)
            o = [x for x in B if x != (j, l)]
            tm = (TIP_SHELF + TIP_PULLED) / 2
            mid = o + [('y', j, l, tm - REAR_OFF + 1)]
            for tag, arm, f, bk in (("arms reach along the basket", TIP_SHELF, 0, B),
                                    ("fingers half way", TIP_SHELF, 45, B),
                                    ("fingers down, basket captured", TIP_SHELF, 90, B),
                                    ("pulling", tm, 90, mid),
                                    ("pulled onto the tray", TIP_PULLED, 90, o + ['tray']),
                                    ("arms stowed, basket on tray", TIP0, 0, o + ['tray']),
                                    ("fingers down round the tray basket", TIP_TRAY, 90, o + ['tray']),
                                    ("pushing", tm, 90, mid),
                                    ("pushed back", TIP_PUSHED, 90, B),
                                    ("fingers up, arms out", TIP_SHELF, 0, B)):
                out.append((f"bay{j+1} L{l+1} {tag}", S(jx, cz, 0, 0, V, arm, f, ry=F[1, 3]), bk))
    picks(0, range(len(LEVELS)), A3.row_M(0))
    picks(1, [2], A3.row_M(0))
    o = [x for x in B if x != (0, 2)] + ['tray']
    jx = A3.bay_rail_x(0)
    for a in (-90.0, -45.0, 0.0):                       # lower hub swivels with its piece, upper hub stays
        out.append((f"bay1 lower hub swivel {a:.0f}", S(jx, CZ_HOME, -90, a, rv(0, a, -90.0)), o))
    for a in (-45.0, 0.0):                              # then the upper hub
        out.append((f"bay1 upper hub swivel {a:.0f}", S(jx, CZ_HOME, a, 0, rv(0, 0.0, a)), o))
    for dz in (60.0, 130.0, 260.0):
        out.append((f"bay1 climb +{dz:.0f}", S(jx, CZ_HOME + dz, 0, 0, rv(0, 0.0, 0.0)), o))
    # row 1 -> onto the transfer cart -> pinion down -> across the aisles to row 2 and row 3 -> along those rows
    for x in np.linspace(ROW_RAIL_X[0] + SEG_V3, X_CART, 9):
        out.append((f"travel row1 x={x:.0f}", S(x), o))
    out.append(("on the cart at row 1, pinion lowered", S(X_CART, lift=0.0), o))
    for y in np.linspace(0, (N_ROWS - 1) * ROW_P, 4 * (N_ROWS - 1) + 1)[1:]:
        out.append((f"transfer y={y:.0f}", S(X_CART, ry=y), o))
    for r in range(1, N_ROWS):
        for x in np.linspace(ROW_RAIL_X[0] + SEG_V3, X_CART, 6):
            out.append((f"travel row{r+1} x={x:.0f}", S(x, ry=r * ROW_P), o))
    picks(ROW_BAYS * 2 - 1, [0], A3.row_M(1))
    return out


if __name__ == "__main__":
    sel = sys.argv[1] if len(sys.argv) > 1 else ""
    t = time.time(); n = bad = 0
    for name, S, bk in cases():
        if sel and sel not in name:
            continue
        hits = check(A3.build(S, bk)); n += 1; bad += bool(hits)
        if hits:
            print(f"{name:40s} HIT", flush=True)
            for hh in hits[:6]:
                print("     ", hh)
    print(f"{n} poses, {bad} with collisions  ({time.time()-t:.0f}s)")
