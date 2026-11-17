"""Exact B-rep check of a crossing turntable (rail piece + slewing ring + rotor screws + stop pin) against its aluminium
bracket and the fixed rails, over the whole swivel 0 .. -90 deg, for a first-bay bracket and a bracket with a left stem.
Also proves the hard stops: just past either end (+3 / -93 deg) the stop pin must hit the slot end.

    pixi run python cad/check_crossing_turntable.py
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "brep"))
import racklift_brep as R          # noqa: E402,F401  (B-rep backend first)
sys.path.insert(0, os.path.join(HERE, "..", "generator"))
import assembly_v3 as A3           # noqa: E402
from params_v3 import *            # noqa: E402,F401,F403
from csg import rot, box           # noqa: E402

TOL = 0.01                          # mm^3
ANGLES = list(range(0, -91, -5))
zc = CROSS_Z[0]
g = SEG_V3 + GAP_V3
fails = 0


def piece_at(ang):
    M = rot('y', ang)
    M[2, 3] = zc
    return [(n, s.transform(M)) for n, s, _ in A3.crossing_local()]


fixed_rails = [("fixed vertical rails", A3.A.vslot_z(zc + g, zc + 200) + A3.A.vslot_z(zc - 200, zc - g)),
               ("fixed horizontal rails", A3.A.vslot_z(-200, -g).rotate('y', 90).translate(0, 0, zc)
                + A3.A.vslot_z(g, 200).rotate('y', 90).translate(0, 0, zc))]

for has_left in (False, True):
    br = A3.crossing_bracket(0.0, zc, has_left)
    others = [("bracket", br)] + fixed_rails
    worst = 0.0
    for ang in ANGLES:
        for n, s in piece_at(ang):
            for m, o in others:
                v = (s * o).volume
                worst = max(worst, v)
                if v > TOL:
                    fails += 1
                    print(f"  COLLISION  {'left stem' if has_left else 'first bay'}  {ang:4d} deg  {n} <-> {m}: {v:.3f} mm^3")
    print(f"[{'bracket with left stem' if has_left else 'first-bay bracket':22s}] {len(ANGLES)} angles x 4 parts x 3: worst {worst:.4f} mm^3")
    # hard stops: 0.5 deg past either end (0.31 mm at the pin, more than the 0.2 mm slot play) the pin must hit the slot end
    for ang in (0.5, -90.5):
        pin = [s for n, s in piece_at(ang) if n.startswith("turntable stop pin")][0]
        v = (pin * br).volume
        ok = v > 0.01
        fails += 0 if ok else 1
        print(f"    stop at {ang:+.1f} deg: pin <-> slot end overlap {v:6.2f} mm^3 -> {'BLOCKED (ok)' if ok else 'NOT BLOCKED'}")

print("RESULT:", "OK" if fails == 0 else f"{fails} problems")
sys.exit(1 if fails else 0)
