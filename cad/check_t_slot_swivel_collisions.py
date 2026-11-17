"""Exact B-rep pairwise collision check for the T-Slot Rail-Drive + 90-deg Swivel Carriage.

Checks all 55 pairs of the 11 carriage/body parts at 0, 45, and 90 deg swivel angles,
plus checks carriage vs 2020 T-Slot Rail & In-Slot Rack clearance at 0 deg.
"""

import itertools
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import build123d as bd
import build_t_slot_swivel_carriage as T


def transform_part(part, loc):
    if isinstance(part, bd.Compound):
        return bd.Compound(children=[s.moved(loc) for s in part.solids()])
    return part.moved(loc)


def check_at_angle(deg: float) -> int:
    parts = T.build_assembled_parts()
    # Keep carriage + body parts (check rail clearance separately at 0 deg)
    t_rail = parts.pop("t_rail")
    t_rack = parts.pop("t_rack")

    if deg != 0.0:
        rot = bd.Location((0, 0, 0), (0, 0, deg))
        rotating_keys = (
            "gantry_plate", "v_wheels", "motor_housing", "motor_shaft",
            "pinion", "retainer", "carriage_pod", "alum_horn", "servo_spline",
        )
        for k in rotating_keys:
            parts[k] = transform_part(parts[k], rot)

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
        print(f"[{deg:3.0f} deg] OK: all {len(items) * (len(items) - 1) // 2} carriage/body pairs have vol = 0.000 mm^3")

    if deg == 0.0:
        for rn, rsolid in (("t_rail", t_rail), ("t_rack", t_rack)):
            for pn in ("gantry_plate", "carriage_pod", "motor_housing", "retainer"):
                inter = rsolid.intersect(parts[pn])
                sols = inter.solids() if (inter is not None and hasattr(inter, "solids")) else []
                vol = sum(s.volume for s in sols)
                if vol > 1e-4:
                    collisions += 1
                    bb = bd.Compound(children=sols).bounding_box()
                    print(
                        f"[rail   ] COLLISION {rn:16s} <-> {pn:16s} | vol={vol:8.3f} mm^3 | "
                        f"X[{bb.min.X:.2f},{bb.max.X:.2f}] Y[{bb.min.Y:.2f},{bb.max.Y:.2f}] Z[{bb.min.Z:.2f},{bb.max.Z:.2f}]"
                    )
                else:
                    print(f"[rail   ] OK        {rn:16s} <-> {pn:16s} | vol = 0.000 mm^3")
    return collisions


def main():
    total = 0
    for deg in (0.0, 45.0, 90.0):
        total += check_at_angle(deg)
    print(f"\nTotal collisions: {total}")
    sys.exit(1 if total else 0)


if __name__ == "__main__":
    main()
