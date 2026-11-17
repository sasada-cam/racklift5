"""Exact B-rep pairwise collision check for the complete Rail-Drive + 90-deg Swivel Pod.

Checks all 36 pairs across the 9 assembly bodies at 0 deg, 45 deg, and 90 deg swivel angles.
"""

import itertools
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import build123d as bd
import build_rail_swivel_assembly as R
import build_td8125mg as S


def check_at_angle(deg: float) -> int:
    parts = R.build_assembled_parts()
    if deg != 0.0:
        pivot = bd.Location((R.SERVO_X_OFF + S.SHAFT_X, 0, 0))
        rot = bd.Location((0, 0, 0), (0, 0, deg))
        for k in ("servo_spline", "swivel_leg_hub"):
            parts[k] = parts[k].moved(pivot.inverse()).moved(rot).moved(pivot)

    items = list(parts.items())
    collisions = 0
    for (n1, s1), (n2, s2) in itertools.combinations(items, 2):
        inter = s1.intersect(s2)
        vol = 0.0
        sols = []
        if inter is not None:
            sols = inter.solids() if hasattr(inter, "solids") else []
            vol = sum(s.volume for s in sols)
        if vol > 1e-4:
            collisions += 1
            bb = bd.Compound(children=sols).bounding_box()
            print(
                f"[{deg:3.0f} deg] COLLISION {n1:16s} <-> {n2:16s} | vol={vol:8.3f} mm^3 | "
                f"X[{bb.min.X:.2f},{bb.max.X:.2f}] Y[{bb.min.Y:.2f},{bb.max.Y:.2f}] Z[{bb.min.Z:.2f},{bb.max.Z:.2f}]"
            )
    if collisions == 0:
        print(f"[{deg:3.0f} deg] OK: all {len(items) * (len(items) - 1) // 2} pairs have vol = 0.000 mm^3")
    return collisions


def main():
    total = 0
    for deg in (0.0, 45.0, 90.0):
        total += check_at_angle(deg)
    print(f"\nTotal collisions across 0/45/90 deg: {total}")
    sys.exit(1 if total else 0)


if __name__ == "__main__":
    main()
