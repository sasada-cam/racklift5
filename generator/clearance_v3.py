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
        ("swivel axle", "aluminium column bar"), ("swivel axle", "JGY-370 swivel motor"),
        # the finger shaft telescopes (rod in tube in tube) and runs through its bearings and the fingers' collars
        ("finger shaft", "finger shaft"), ("finger shaft", "printed front post"), ("printed flip finger", "finger shaft"),
        # belts lie on their pulleys / in their clamps and anchors; stage-2 pulleys turn on axles through the middle bar
        ("stage-2 belt", "stage-2 pulley"), ("stage-2 belt", "belt anchor"), ("stage-2 belt", "stage-2 belt"),
        ("stage-2 pulley", "telescopic arm, middle bar"), ("GT2 belt loop (stage 1)", "printed stage-1 belt clamp"),
        ("basket (moving)", "printed flip finger"),
        # the robot's pinion meshes the transfer rack; the cart hangs on its beam, the lock pin sits in its socket
        ("turn drive pinion", "printed transfer rack"), ("trolley V-wheels", "transfer beam"),
        ("floor guide rollers", "floor guide rail"), ("cart lock pin", "cart lock socket"), ("cart lock pin", "cart frame"),
        # solenoids are bolted to the battery box / column; the bolt sits in its socket; the pin's tab passes the cart frame
        ("lock solenoid", "battery"), ("lock solenoid", "aluminium column bar"), ("lock solenoid", "lock solenoid")]
FIXED = ("static", "rotor")


def skip(a, b):
    if a.startswith("jack:") and b.startswith("jack:"):         # the jack's own pins, shafts and screws are one assembly
        return True
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


def lock_cases(B):
    """the locking sequence on the cart: robot arrives (plunger 2 mm from the tab), docking bolt out, pin released, and back"""
    S = A3.state
    o = [x for x in B if x != (0, 2)] + ['tray']
    out = []
    for tag, lock, bolt in (("arrived, nothing extended", 0.0, 0.0), ("docking bolt in its socket", 0.0, BOLT_STROKE),
                            ("bolt in, pin released by the plunger", LOCK_STROKE, BOLT_STROKE),
                            ("pin half way", LOCK_STROKE / 2, BOLT_STROKE)):
        for ry in (0.0, ROW_P):
            out.append((f"cart lock row{int(ry / ROW_P) + 1}: {tag}", S(X_CART, ry=ry, lock=lock, bolt=bolt), o))
    out.append(("cart lock: robot approaching the cart (x=C-30, nothing extended)", S(X_CART - 30, lock=0.0, bolt=0.0), o))
    return out


def dock_cases(B):
    """docking stations (right of the cart, under the tray): push a basket onto the raised platform, pull one back, lower the jack
    with a basket on it, and ride the cart across every row over lowered docks with baskets"""
    S = A3.state
    out = []
    J0 = [JACK_DOWN] * N_ROWS

    def jup(r, z=JACK_UP):
        j = list(J0); j[r] = z
        return j

    def rest(*rows, z=JACK_UP):
        return [('dockrest', r, z if not isinstance(z, dict) else z[r]) for r in rows]
    tm = (TIP_TRAY + TIP_PUSHED) / 2
    for r in (0, N_ROWS - 1):                       # the dock of the first and of the last row (the middle one is the same)
        ry, xc = r * ROW_P, DOCK_X_C
        n = f"dock{r+1}"
        for tag, arm, f, bk in (("arrives, basket on the tray, jack up", TIP0, 0, B + ['tray']),
                                ("arm tips to the tray basket", TIP_TRAY, 0, B + ['tray']),
                                ("fingers down round the tray basket", TIP_TRAY, 90, B + ['tray']),
                                ("pushing the basket across the gap", tm, 90, B + [('y', 0, 0, 0)]),
                                ("pushed onto the platform", TIP_PUSHED, 90, B + [('dock', r, JACK_UP, BASKET_Y0)]),
                                ("fingers up, arms out", TIP_SHELF, 0, B + rest(r)),
                                ("arms stowed, basket on the platform", TIP0, 0, B + rest(r))):
            if tag.startswith("pushing"):
                bk = B + [('dock', r, JACK_UP, tm - REAR_OFF)]
            out.append((f"{n} {tag}", S(xc, ry=ry, arm=arm, f=f, jack=jup(r)), bk))
        for z in (JACK_UP - 40, JACK_UP - 80, JACK_DOWN):                  # jack lowers the basket (arms stowed)
            out.append((f"{n} jack at {z:.0f} with the basket", S(xc, ry=ry, jack=jup(r, z)), B + rest(r, z=z)))
        out.append((f"{n} pulling a returned basket", S(xc, ry=ry, arm=tm - 0, f=90, jack=jup(r)),
                    B + [('dock', r, JACK_UP, tm - 3 - BASKET_D)]))
        out.append((f"{n} returned basket pulled onto the tray", S(xc, ry=ry, arm=TIP_PULLED, f=90, jack=jup(r)), B + ['tray']))
    # riding the cart across the aisles: the tray, servo 1 and the belts under it pass over the lowered docks and the baskets on
    # them (basket top 150 mm, tray bottom 166 mm, servo 1 bottom 156 mm)
    for y in np.linspace(0, (N_ROWS - 1) * ROW_P, 4 * (N_ROWS - 1) + 1)[1:]:
        out.append((f"transfer y={y:.0f} over lowered docks with baskets", S(X_CART, ry=y), B + rest(*range(N_ROWS), z=JACK_DOWN)))
        out.append((f"transfer y={y:.0f} with a tray basket over lowered docks with baskets", S(X_CART, ry=y),
                    B + ['tray'] + rest(*range(N_ROWS), z=JACK_DOWN)))
    return out


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
    out += lock_cases(B)
    out += dock_cases(B)
    return out


def swept_over_docks(verbose=True):
    """Sampling the cart's ride can miss a thin wall, so check the sweep directly: everything that rides the cart (robot, tray,
    servos, belts, cart) moves in y only, so (1) over a basket's footprint the lowest point of ALL of it must stay above the
    basket top, and (2) over the whole dock (deck, platform, motor) above their tops.  Returns the smaller margin in mm (> 2)."""
    B = [(b, l) for b in range(N_BAYS_ALL) for l in range(len(LEVELS))]
    items = A3.build(A3.state(X_CART, ry=ROW_P), B + ['tray'])
    zones = {"basket": (PXC - 130.0 - 2, PXC + 130.0 + 2, JACK_DOWN + BASKET_CLR + BASKET_H),
             "dock (deck, platform, motor)": (DECK_X[0], DECK_X[1], JACK_ZS + 21.0)}
    worst = 1e9
    for zn, (x0, x1, top) in zones.items():
        low, who = 1e9, ""
        for n, sld, c, g in items:
            if (g in ("static", "rotor") and not n.startswith("cart")) or g.startswith("jack"):
                continue                                   # fixed things are not swept (the docks themselves are fixed)
            t = sld.triangles().reshape(-1, 3)
            if t[:, 0].max() < x0 or t[:, 0].min() > x1:
                continue                                   # not over this zone
            if t[:, 2].min() < low:
                low, who = t[:, 2].min(), n
        if verbose:
            print(f"swept clearance over the {zn}: lowest part riding the cart = {low:.0f} mm ({who}), top {top:.0f} mm "
                  f"-> margin {low - top:.0f} mm")
        worst = min(worst, low - top)
    return worst


def swept_row_ends(verbose=True):
    """The cart carries the robot past every row's lock socket and rail-end arms.  Everything that rides the cart moves in y
    only, so if its (x, z) footprint never overlaps theirs, nothing can touch at any y.  Returns the overlaps found."""
    B = [(b, l) for b in range(N_BAYS_ALL) for l in range(len(LEVELS))]
    items = A3.build(A3.state(X_CART, ry=ROW_P), B + ['tray'])
    fixed = {}
    for n, sld, c, g in items:
        if g == "static" and (n.startswith("cart lock socket") or n.startswith("rail end arm")):
            t = sld.triangles().reshape(-1, 3)
            if t[:, 0].max() > X_CART - 200:
                fixed.setdefault(n.split(" (row")[0], []).append(t)
    ride = []
    for n, sld, c, g in items:
        if g == "static" or g.startswith("jack") or n.startswith("cart lock pin") or (g == "rotor" and not n.startswith("cart")):
            continue                                       # fixed things, the docks, the pin (it enters the socket by design)
        ride.append((n, sld.triangles().reshape(-1, 3)))
    found = []
    for fn, tl in fixed.items():
        for t in tl[:1]:                                     # rows are identical (shifted in y only)
            fx0, fx1, fz0, fz1 = t[:, 0].min(), t[:, 0].max(), t[:, 2].min(), t[:, 2].max()
            for n, r_ in ride:
                if r_[:, 0].max() > fx0 and r_[:, 0].min() < fx1 and r_[:, 2].max() > fz0 and r_[:, 2].min() < fz1:
                    found.append((fn, n))
    if verbose:
        print(f"swept (x, z) overlap of the ride-along parts with the rows' lock socket / rail-end arms: {len(found)}"
              + "".join(f"\n    {a}  <->  {b}" for a, b in found[:6]))
    return found


if __name__ == "__main__":
    swept_row_ends()
    m = swept_over_docks()
    if m <= 2.0:
        print("  SWEPT CLEARANCE FAIL")
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
