"""v3 collision check: legs, swivel hubs, level beam and slide against the v2 rack/rails and each other, at every pick
and through every step of the junction walk.        python clearance_v3.py [filter]"""
import sys, time; sys.path.insert(0, '.')
import numpy as np
import assembly_v3_legs as A3
from params_v3_legs import *
from interfere import interference, bb

SKIP = [("V wheel", "V-slot"), ("V wheel", "rail piece"), ("spacer", "V-slot"), ("spacer", "rail piece"),
        ("printed climber pinion", "printed in-slot rack"), ("printed climber pinion", "rail piece"),
        ("printed climber pinion", "V-slot"),
        ("swivel bearing", "printed leg"), ("swivel bearing", "JGY-370 swivel motor"),
        ("centre shaft", "printed leg"), ("printed leg pinion", "printed leg gear"),
        ("printed slide carriage", "V-slot 2020 slide rail"), ("printed slide carriage", "GT2 belt"),
        ("spring-loaded hook pin", "printed spring barrel"), ("spring-loaded hook pin", "printed pin arm"),
        ("printed spring barrel", "printed pin arm"), ("SG90 pin lock servo", "printed pin arm"),
        ("basket (on tray)", "plywood tray"), ("basket (on tray)", "printed basket stop"), ("basket (on tray)", "limit switch"),
        ("spring-loaded hook pin", "basket (on tray)")]
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
            if a[2] != b[2]:
                pair(a, b)
    return hits


def cases():
    B = [(b, l) for b in range(N_BAYS) for l in range(len(LEVELS))]
    out = []
    for j, (jx, jz) in enumerate(JUNCTIONS):
        rv = lambda r: [r if k == j else -90 for k in range(N_BAYS)]
        for l in range(len(LEVELS)):
            v = vertical_state(jx, cz_for_level(l))
            others = [x for x in B if x != (j, l)]
            for tag, pin, lift, bk in (("reach", PIN_PULL, 0, B), ("ride over", BASKET_Y0, PIN_LIFT, B),
                                        ("pulled", PIN_STOW, 0, others + ['tray']), ("pushed", PIN_PUSH, 0, B)):
                out.append((f"bay{j+1} L{l+1} {tag}", A3.state(v["cx"], v["cz"], v["a1"], v["a2"], 0, 0, rv(0), pin, lift), bk))
        K = WALK
        seq = [(K[0], -90, -90, -90), (K[0], -45, -90, -45), (K[0], 0, -90, 0)]
        for u in (0.25, 0.5, 0.75, 1.0):
            seq.append(((K[0][0] + (K[1][0] - K[0][0]) * u, K[0][1] + (K[1][1] - K[0][1]) * u), 0, -90, 0))
        seq += [(K[1], 0, -90, -45), (K[1], 0, -90, -90)]
        for u in (0.25, 0.5, 0.75, 1.0):
            seq.append(((K[1][0] + (K[2][0] - K[1][0]) * u, K[1][1] + (K[2][1] - K[1][1]) * u), 0, -90, -90))
        seq += [(K[2], 0, -45, -45), (K[2], 0, 0, 0)]
        others = [x for x in B if x != (0, 2)] + ['tray']
        for k, ((a, b), h1, h2, r) in enumerate(seq):
            s = walk_state(jx, jz, a, b)
            out.append((f"J{j+1} walk {k:2d} a={a:.0f} b={b:.0f}", A3.state(s["cx"], s["cz"], s["a1"], s["a2"], h1, h2, rv(r)), others))
        for cz in (jz + L_LEG, jz + L_LEG + 100, cz_for_level(0)):
            v = vertical_state(jx, cz)
            out.append((f"J{j+1} climb cz={cz:.0f}", A3.state(v["cx"], v["cz"], v["a1"], v["a2"], 0, 0, rv(0)), others))
    others = [x for x in B if x != (0, 2)] + ['tray']
    x0, x1 = JUNCTIONS[0][0] + TRAVEL_HALF, JUNCTIONS[1][0] + TRAVEL_HALF
    for x in np.linspace(x0, x1, 6):
        t = travel_state(x)
        out.append((f"travel cx={x:.0f}", A3.state(t["cx"], t["cz"], t["a1"], t["a2"], -90, -90, [-90, -90]), others))
    return out


if __name__ == "__main__":
    sel = sys.argv[1] if len(sys.argv) > 1 else ""
    t = time.time(); bad = 0; n = 0
    for name, S, bk in cases():
        if sel and sel not in name:
            continue
        hits = check(A3.build(S, bk)); n += 1
        bad += bool(hits)
        if hits:
            print(f"{name:34s} {len(hits)} HIT", flush=True)
            for hh in hits[:6]:
                print("     ", hh)
    print(f"{n} poses, {bad} with collisions  ({time.time()-t:.0f}s)")
