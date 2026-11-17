"""Exact B-rep pairwise collision (boolean intersection) check for the 4632-370 kit.

Checks every pair among:
  - motor_housing (gearbox + 370 motor can + screws + terminals)
  - output_shaft
  - top_cover
  - bottom_cradle
  - climber_pinion
  - shaft_retainer

Reports exact intersection volume (mm^3) and collision bounding box for any overlapping pair.
"""

import itertools
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import build123d as bd
import build_4632_370 as M
import build_4632_370_kit as K


def as_solid(shape):
    if isinstance(shape, bd.Compound):
        solids = shape.solids()
        return bd.Compound(children=solids) if solids else shape
    return shape


def main():
    shaft_loc = bd.Location((K.SHAFT_OFFSET_X, 0, K.GB_H))
    pinion_loc = bd.Location((K.SHAFT_OFFSET_X, 0, K.GB_H + 5.0))
    retainer_loc = bd.Location((K.SHAFT_OFFSET_X, 0, K.GB_H + 16.0))

    housing = bd.Compound(
        children=[
            M.build_gearbox(),
            M.build_cover_screws(),
            M.build_motor_can(),
            M.build_terminals(),
        ]
    )
    parts = [
        ("motor_housing", housing),
        ("output_shaft", M.build_output_shaft()),
        ("top_cover", K.build_cover()),
        ("bottom_cradle", K.build_bottom_cradle()),
        ("climber_pinion", K.build_climber_pinion().moved(pinion_loc)),
        ("shaft_retainer", K.build_shaft_retainer().moved(retainer_loc)),
    ]

    collisions = 0
    for (n1, s1), (n2, s2) in itertools.combinations(parts, 2):
        inter = s1.intersect(s2)
        vol = 0.0
        if inter is not None:
            sols = inter.solids() if hasattr(inter, "solids") else []
            vol = sum(s.volume for s in sols)
        if vol > 1e-4:
            collisions += 1
            bb = bd.Compound(children=sols).bounding_box()
            print(
                f"COLLISION  {n1:16s} <-> {n2:16s} | vol = {vol:8.3f} mm^3 | "
                f"bbox X[{bb.min.X:.2f},{bb.max.X:.2f}] Y[{bb.min.Y:.2f},{bb.max.Y:.2f}] Z[{bb.min.Z:.2f},{bb.max.Z:.2f}]"
            )
        else:
            print(f"OK         {n1:16s} <-> {n2:16s} | vol = 0.000 mm^3")

    print(f"\nTotal colliding pairs: {collisions}")
    sys.exit(1 if collisions else 0)


if __name__ == "__main__":
    main()
