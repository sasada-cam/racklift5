"""Option 1 swivel pod ON a real rail crossing (the turntable ported from generator/assembly_v3.py into the pod's frame).

The pod alone (build_t_slot_option1.py) sits on a plain straight rail, so its 90 deg swivel had nothing to turn.  Here the
rail under the pod is the crossing: a short rail piece + in-slot rack on a preloaded 608ZZ duplex pivot (M8), turning WITH the carriage (the
V-wheels clamp it), between four fixed rail arms (+-X = one rail direction, +-Y = the other).

Pod frame (same as build_t_slot_option1.py): rail along X at 0 deg, rail centre Z = -12.475, bottom face Z = -22.475,
swivel axis = Z through (0, 0), carriage turns +deg.

  turning with the carriage : crossing rail piece, its rack (one comb with the arms' racks: tooth SPACE at the centre,
                              whole pitches, 1.0 mm gaps), rotor disc (2 x M5 button head + T-nuts in the piece's bottom
                              slot) with the M8 bolt's head in its hex pocket, 2 x 608ZZ + spacers + disc spring + nyloc, stop pin (M5 + 10 mm sleeve, T-nut) 36 mm out on the -X side
  fixed                     : 4 rail arms + racks, turned pivot housing (spigot in the bracket's 27 mm bore, 4 x M4 on the
                              36 mm circle, 2 x 608ZZ seats + shoulder), aluminium bracket plate (quarter slot for the stop pin: its ends are the 0 / 90 deg stops)

    pixi run python cad/build_option1_crossing.py                 # 17.5-pitch piece (needed by this pod: its side pod
                                                                  # hangs below the rail top out to r = 81.4 mm)
    pixi run python cad/build_option1_crossing.py --pitches 11.5  # v3's piece today: 39 collisions (pod, housing, pinion)
    pixi run python cad/build_option1_crossing.py --export        # + STEP/STL + viewer meshes (sim/view_t_slot_option1.py)
"""
import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import build123d as bd                      # noqa: E402
import build_openbuilds_gantry as G         # noqa: E402
import build_t_slot_swivel_carriage as T    # noqa: E402
import build_t_slot_option1 as O            # noqa: E402

OUT_DIR = os.path.abspath(os.path.join(HERE, "..", "models", "components", "t_slot_option1_crossing"))
SIM_MESH_DIR = O.SIM_MESH_DIR

P = math.pi * 1.5                 # rack pitch (module 1.5)
GAP = 1.0                         # piece <-> arm gap (as v3)
RACK_E = (P - GAP) / 2.0          # rack end face this far past its last tooth centre -> pitch P across the gap
ARM_L = 90.0                      # length of each fixed arm shown
Z_RB = -G.RAIL_CENTER_DROP - G.RAIL_W / 2.0      # rail bottom face, -22.475
sys.path.insert(0, os.path.join(HERE, "..", "generator"))
import params_v3 as PV                      # noqa: E402  the crossing pivot (preloaded 608ZZ duplex) - one definition for both models
SCREW_X = PV.TT_SCREW_Z           # 2 x M5 button-head rotor screws on the piece axis
PIN_R, SLEEVE_R = 36.0, 5.0       # stop pin radius / sleeve radius
SLOT_W = SLEEVE_R + 0.2           # 0.2 mm play -> 0.3 deg
PLATE_T = 10.0                    # bracket plate (same as v3's bearing plate)
PLATE_Z1 = Z_RB - 8.0             # bracket top face (2 mm under the rotor disc)
PLATE_Z0 = PLATE_Z1 - PLATE_T

ALIGN_MIN = (bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN)


def cyl(r, z0, z1, x=0.0, y=0.0):
    return bd.Cylinder(r, z1 - z0, align=ALIGN_MIN).moved(bd.Location((x, y, z0)))


def box(x0, x1, y0, y1, z0, z1):
    return bd.Box(x1 - x0, y1 - y0, z1 - z0, align=(bd.Align.MIN,) * 3).moved(bd.Location((x0, y0, z0)))


def rot_z(shape, deg, cx=0.0, cy=0.0):
    piv = bd.Location((cx, cy, 0))
    loc = piv * bd.Location((0, 0, 0), (0, 0, deg)) * piv.inverse()
    if isinstance(shape, bd.Compound):
        return bd.Compound(children=[s.moved(loc) for s in shape.solids()])
    return shape.moved(loc)


_RACK = {}


def comb(x0, x1):
    """in-slot rack between x0 and x1 cut from ONE comb (tooth space at X = 0, tooth centres at (m + 1/2) P)"""
    span = max(abs(x0), abs(x1)) + 2 * P
    n = 2 * math.ceil(span / P)                      # even -> tooth space at 0
    if n not in _RACK:
        _RACK[n] = T.build_in_slot_rack(n * P)
    return _RACK[n] & box(x0, x1, -10, 10, -12, 0)


def rail(x0, x1):
    return G.build_t_slot_2020(x1 - x0).moved(bd.Location(((x0 + x1) / 2, 0, -G.RAIL_CENTER_DROP)))


def tnut(x, z_face, up=1.0):
    """M5 drop-in T-nut for the 2020 slot, inside the slot at x on the rail's face z_face (slot depth along +up*Z):
    plate 10 x 10 behind the 6 mm lips (depth 1.6 .. 2.9) + wedge into the slot's 45 deg taper (to depth 4.4), M5 hole"""
    d0, d1, d2 = 1.6, 2.9, 4.4
    z = lambda d: z_face + up * d
    plate = box(x - 5.0, x + 5.0, -5.0, 5.0, min(z(d0), z(d1)), max(z(d0), z(d1)))
    with bd.BuildPart() as w:
        with bd.BuildSketch(bd.Plane.XZ.offset(0)) as sk:
            bd.Polygon((-5.0, z(d1)), (5.0, z(d1)), (3.5, z(d2)), (-3.5, z(d2)), align=None)
        bd.extrude(amount=5.0, both=True)
    wedge = w.part.moved(bd.Location((0, 0, 0))).rotate(bd.Axis.Z, 90).moved(bd.Location((x, 0, 0)))
    return (plate + wedge) - cyl(2.5, min(z(0), z(d2)) - 1, max(z(0), z(d2)) + 1, x)


def crossing_parts(seg):
    """seg = piece half length.  returns (turning, fixed) dicts of solids at 0 deg"""
    g = seg + GAP
    D = lambda d: Z_RB - d                                   # depth behind the piece's bottom face -> pod Z
    d0, d1, rr = PV.TT_ROTOR
    rotor = cyl(rr, D(d1), D(d0)) - cyl(4.1, D(d1) - 1, D(d0) + 1)
    rotor = rotor - bd.extrude(bd.RegularPolygon(7.65, 6, align=None).moved(bd.Location((0, 0, D(5.6)))), amount=6.6)
    screws = []
    for x in (-SCREW_X, SCREW_X):
        rotor = rotor - cyl(2.75, D(d1) - 1, D(d0) + 1, x) - cyl(PV.TT_ACCESS_R, D(d1) - 1, D(d1 - 3.0), x)
        screws += [cyl(2.5, D(d1 - 2.75), Z_RB + 4.4, x), cyl(4.75, D(d1), D(d1 - 2.75), x)]   # (up into the T-nut)
    stack = {}
    for name, rings, _ in PV.TT_STACK:
        sol = None
        for a, b, ro, ri in rings:
            hexa = "nyloc" in name or ("bolt" in name and ri == 0 and ro > 5)
            ring = bd.extrude(bd.RegularPolygon(ro, 6, align=None).moved(bd.Location((0, 0, D(b)))), amount=b - a) if hexa else cyl(ro, D(b), D(a))
            if ri > 0:
                ring = ring - cyl(ri, D(b) - 1, D(a) + 1)
            sol = ring if sol is None else sol + ring
        stack[name] = sol
    pivot = bd.Compound(children=[v for k, v in stack.items() if "608" not in k])
    bearings = bd.Compound(children=[v for k, v in stack.items() if "608" in k])
    H = PV.TT_HOUSING
    housing = cyl(H["spigot"][2], D(H["spigot"][1]), D(H["spigot"][0])) + cyl(H["flange"][2], D(H["flange"][1]), D(H["flange"][0])) \
        + cyl(H["body"][2], D(H["body"][1]), D(H["body"][0]))
    for i, (a, b, r) in enumerate(H["bore"]):
        housing = housing - cyl(r, D(b) - (1 if i == 2 else 0.01), D(a) + (1 if i == 0 else 0.01))
    m4 = [(PV.TT_M4_R * math.cos(math.radians(a)), PV.TT_M4_R * math.sin(math.radians(a))) for a in (45, 135, 225, 315)]
    hs = []
    for x, y in m4:
        housing = housing - cyl(2.25, D(H["flange"][1]) - 1, D(H["flange"][0]) + 1, x, y)
        hs += [cyl(2.0, D(H["flange"][1]), D(H["flange"][0] - 7), x, y), cyl(3.5, D(H["flange"][1]) - 4, D(H["flange"][1]), x, y)]
    # stop pin: M5 x 25 socket cap from below -> washer -> 10 x 5.2 x 20 steel sleeve -> up through the slot lips into a T-nut
    # inside the piece's bottom T-slot (anchored in the slot, not just stuck on the rail's face)
    pin = (cyl(SLEEVE_R, Z_RB - 20, Z_RB, -PIN_R) - cyl(2.6, Z_RB - 21, Z_RB + 1, -PIN_R)) + cyl(6, Z_RB - 21, Z_RB - 20, -PIN_R) \
        + cyl(4.25, Z_RB - 24, Z_RB - 21, -PIN_R) + cyl(2.5, Z_RB - 21, Z_RB + 4.4, -PIN_R)
    tnuts = [tnut(x, Z_RB) for x in (-PIN_R, -SCREW_X, SCREW_X)]
    turning = {
        "xp_rail": rail(-seg, seg),
        "xp_rack": comb(-seg, seg),
        "xp_rotor": rotor,
        "xp_rotor_screws": bd.Compound(children=screws),
        "xp_pivot_608zz": bearings,
        "xp_pivot_m8_stack": pivot,
        "xp_stop_pin": pin,
        "xp_tnuts": bd.Compound(children=tnuts),
    }
    # fixed: arms along +-X, and the same turned 90 deg for +-Y (the comb is symmetric, so the phase holds)
    arm_r = [rail(g, g + ARM_L), rail(-g - ARM_L, -g)]
    arm_k = [comb(g, g + ARM_L), comb(-g - ARM_L, -g)]
    plate = box(-48, 48, -48, 48, PLATE_Z0, PLATE_Z1) - cyl(PV.TT_SPIGOT_HOLE_R, PLATE_Z0 - 1, PLATE_Z1 + 1)
    for x, y in m4:
        plate = plate - cyl(1.65, PLATE_Z0 - 1, PLATE_Z1 + 1, x, y)
    # quarter slot: the pin goes from (-36, 0) at 0 deg to (0, -36) at 90 deg
    ring = (cyl(PIN_R + SLOT_W, PLATE_Z0 - 1, PLATE_Z1 + 1) - cyl(PIN_R - SLOT_W, PLATE_Z0 - 2, PLATE_Z1 + 2)) \
        & box(-PIN_R - SLOT_W - 1, 0, -PIN_R - SLOT_W - 1, 0, PLATE_Z0 - 1, PLATE_Z1 + 1)
    plate = plate - ring - cyl(SLOT_W, PLATE_Z0 - 1, PLATE_Z1 + 1, -PIN_R, 0) - cyl(SLOT_W, PLATE_Z0 - 1, PLATE_Z1 + 1, 0, -PIN_R)
    fixed = {
        "fx_rails_x": bd.Compound(children=arm_r),
        "fx_racks_x": bd.Compound(children=arm_k),
        "fx_rails_y": rot_z(bd.Compound(children=arm_r), 90),
        "fx_racks_y": rot_z(bd.Compound(children=arm_k), 90),
        "fx_pivot_housing": housing,
        "fx_housing_screws": bd.Compound(children=hs),
        "fx_bracket": plate,
    }
    return turning, fixed


POD_TURN = ("gantry_plate", "v_wheels", "motor_housing", "motor_shaft", "pinion", "retainer", "bearing_pinion", "carriage_pod", "motor_cap", "pinion_cover", "fasteners_turn")
POD_SERVO = ("gear_20t", "alum_horn", "servo_spline", "fasteners_gear")
# contacts that are meant to be there: wheels roll on the piece, the pinion meshes its rack (tooth phase not modelled here)
SKIP = {("v_wheels", "xp_rail"), ("pinion", "xp_rack"), ("pinion", "xp_rail"), ("motor_shaft", "xp_rail")}


def vol(a, b):
    ba, bb = a.bounding_box(), b.bounding_box()
    if (ba.min.X > bb.max.X or bb.min.X > ba.max.X or ba.min.Y > bb.max.Y or bb.min.Y > ba.max.Y
            or ba.min.Z > bb.max.Z or bb.min.Z > ba.max.Z):
        return 0.0
    r = a.intersect(b)
    return sum(s.volume for s in r.solids()) if r is not None and hasattr(r, "solids") else 0.0


def pose(pod, turning, fixed, deg):
    """everything at carriage angle deg: (moving-with-carriage, servo, stationary pod, fixed crossing)"""
    mv = {k: rot_z(pod[k], deg) for k in POD_TURN}
    mv.update({k: rot_z(s, deg) for k, s in turning.items()})
    sv = {k: rot_z(pod[k], -2 * deg, O.SERVO_PIVOT_X, 0) for k in POD_SERVO}
    st = {k: s for k, s in pod.items() if k not in POD_TURN + POD_SERVO and k not in ("t_rail", "t_rack")}
    return mv, sv, st, fixed


def check(pitches, angles):
    seg = pitches * P + RACK_E
    pod = O.build_assembled_parts()
    turning, fixed = crossing_parts(seg)
    print(f"\n=== crossing piece {pitches} pitches + {RACK_E:.3f} = half length {seg:.2f} mm "
          f"(pinion at {T.PINION_X} mm, tip radius {T.PINION_X + 12.0:.1f} mm) ===")
    bad = 0
    for deg in angles:
        mv, sv, st, fx = pose(pod, turning, fixed, deg)
        hits = []
        movers = {**mv, **sv, **st}
        for a, sa in movers.items():                  # everything that is not the fixed crossing <-> the fixed crossing
            for b, sb in fx.items():
                v = vol(sa, sb)
                if v > 1e-3:
                    hits.append((a, b, v))
        for a in turning:                             # the turning piece <-> the pod's own parts
            for b, sb in {**{k: v for k, v in mv.items() if k in POD_TURN}, **sv, **st}.items():
                if (b, a) in SKIP:
                    continue
                v = vol(mv[a], sb)
                if v > 1e-3:
                    hits.append((a, b, v))
        bad += len(hits)
        print(f"[{deg:5.1f} deg] " + ("OK" if not hits else "; ".join(f"{a} <-> {b} {v:.2f} mm^3" for a, b, v in hits)))
    # hard stops: 0.5 deg past either end the stop pin must hit the slot end
    for deg in (-0.5, 90.5):
        pin = rot_z(turning["xp_stop_pin"], deg)
        v = vol(pin, fixed["fx_bracket"])
        ok = v > 0.01
        bad += 0 if ok else 1
        print(f"  stop at {deg:+.1f} deg: pin <-> slot end {v:.2f} mm^3 -> {'BLOCKED (ok)' if ok else 'NOT BLOCKED'}")
    print(f"RESULT ({pitches} pitches): " + ("OK - 0 collisions" if bad == 0 else f"{bad} problems"))
    return bad, pod, turning, fixed


def export(pod, turning, fixed):
    os.makedirs(OUT_DIR, exist_ok=True)
    os.makedirs(SIM_MESH_DIR, exist_ok=True)
    for k, s in {**turning, **fixed}.items():
        bd.export_step(s, os.path.join(OUT_DIR, f"{k}.step"))
        bd.export_stl(s, os.path.join(OUT_DIR, f"{k}.stl"), tolerance=0.01, angular_tolerance=0.1)
        bd.export_stl(s, os.path.join(SIM_MESH_DIR, f"{k}.stl"), tolerance=0.01, angular_tolerance=0.1)
    pod_parts = [s for k, s in pod.items() if k not in ("t_rail", "t_rack")]
    asm = bd.Compound(children=pod_parts + list(turning.values()) + list(fixed.values()), label="Option1_on_crossing")
    bd.export_step(asm, os.path.join(OUT_DIR, "opt1_on_crossing_assembly.step"))
    print(f"exported -> {OUT_DIR} and {SIM_MESH_DIR}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pitches", type=float, default=None, help="piece half length in pitches (x.5); default 17.5")
    ap.add_argument("--export", action="store_true")
    a = ap.parse_args()
    angles = [0.0] + [float(d) for d in range(5, 90, 5)] + [90.0]
    runs = [a.pitches] if a.pitches else [17.5]
    total = 0
    for p in runs:
        bad, pod, turning, fixed = check(p, angles)
        total += bad if p == runs[-1] else 0
    if a.export:
        export(pod, turning, fixed)
    sys.exit(1 if total else 0)


if __name__ == "__main__":
    main()
