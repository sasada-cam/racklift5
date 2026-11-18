"""Dual-Leg Option 1 Robot + Complete 2-Bay 2020 Railing Layout.

Places BOTH Option 1 swivel pods (upper Hub 1 at Z = 330.33 mm, lower Hub 2 at Z = 90.0 mm,
spaced by D_HUB = 51 * P = 240.33 mm) onto a single 28 x 24 x 340 mm aluminium column bar,
mounted on a complete 2-bay 2020 T-slot railing set with exact v3 cut lengths, continuous-pitch
in-slot racks, 4x 17.5-pitch crossing turntables, and reference 5-shelf slotted-angle bays.

Coordinates (v3 world frame, mm):
  X = along horizontal rails (Bay 0 vertical rail at X = 0, Bay 1 vertical rail at X = 438.25)
  Y = into rack (rails centred at RAIL_Y = -50.0, front face Y = -60, back face Y = -40; rack front Y = 0)
  Z = up from floor (Z = 0 floor, lower horizontal rail JZ_LO = 90.0, upper JZ_UP = 330.33, top RAIL_TOP = 1080.0)

Usage:
  pixi run python cad/build_option1_dual_legs.py
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
GEN_DIR = os.path.abspath(os.path.join(HERE, "..", "generator"))
sys.path.insert(0, HERE)
sys.path.insert(0, GEN_DIR)

import build123d as bd                      # noqa: E402
import build_t_slot_option1 as O            # noqa: E402
import build_option1_crossing as XC         # noqa: E402
import params_v3 as PV                      # noqa: E402

OUT_DIR = os.path.abspath(os.path.join(HERE, "..", "models", "components", "option1_dual_legs"))
SIM_MESH_DIR = os.path.abspath(os.path.join(HERE, "..", "sim", "build", "meshes_option1_dual_legs"))

# --- Exact v3 bay & rail constants (mm) ---
P = PV.P                                    # 4.712389 mm rack pitch (MOD = 1.5)
GAP = PV.GAP_V3                             # 1.0 mm crossing gap
SEG = XC.SEG_V3 if hasattr(XC, "SEG_V3") else PV.SEG_V3   # 84.32 mm half-length (17.5 pitches + RACK_E)
G_HALF = SEG + GAP                          # 85.32 mm from crossing center to fixed rail start
BAY_P = PV.BAY_P                            # 93 * P = 438.25 mm
JZ_LO = PV.JZ_LO                            # 90.0 mm lower horizontal rail
D_HUB = PV.D_HUB                            # 51 * P = 240.33 mm hub spacing
JZ_UP = PV.JZ_UP                            # 330.33 mm upper horizontal rail
RAIL_Y = PV.RAIL_Y                          # -50.0 mm rail centerline Y
RAIL_TOP = PV.RAIL_TOP_V3                   # 1080.0 mm vertical rail top
COL_W, COL_D, COL_WALL = 40.0, 25.0, 2.0    # standard 40 x 25 x 2.0 mm hollow aluminium rectangular box bar (~224 g) — clear edge margin for ±12 mm bolts + washers
COL_Y1 = PV.COL_Y[1]                        # -145.5 (flush against P04 rear top plate)
COL_Y0 = COL_Y1 - COL_D                     # -170.5 (open front face for M4 x 35 through-bolts)
# Lower pod MIRRORED (as the v3 model: HUB_BASE lower = 180 deg about the hub's y axis): its pinion sits on the other side
# of its hub, so the two pinions are 105 mm apart along the rail and never cross a 1 mm crossing cut at the same time
# (worst case the other pinion is 32 mm inside solid rack; both-pods-the-same had 56 climb / 393 travel positions where
# both were at a cut).  Its body mount then points UP toward the upper one, so the column starts at z 74 instead of 10.
MIRROR_LOWER = True
HUB_BASE = {"upper": 0.0, "lower": 180.0 if MIRROR_LOWER else 0.0}
HUB_SIGN = {"upper": 1.0, "lower": -1.0 if MIRROR_LOWER else 1.0}      # pod +X -> hub +z (upper) / -z (mirrored lower)
COL_Z0, COL_Z1 = (JZ_LO - 16.0 if MIRROR_LOWER else 10.165), 350.165   # hollow aluminium box bar spanning both P04 mounts

BAY_XS = (0.0, BAY_P)                       # Bay 0 & Bay 1 vertical rail X positions
LEFT_END_X = -180.0
RIGHT_END_X = BAY_P + 311.75                # 750.0 mm

# Rail segment catalog: (name, axis, length_mm, start_coord, fixed_coords)
RAIL_CUT_LIST = [
    ("Turntable crossing pieces (17.5 pitches, 4x)", "turn", 2 * SEG),
    ("Vertical middle bridge rails (Z 175.32..245.01, 2x)", "V", (JZ_UP - G_HALF) - (JZ_LO + G_HALF)),
    ("Vertical upper climb rails (Z 415.65..1080.0, 2x)", "V", RAIL_TOP - (JZ_UP + G_HALF)),
    ("Horizontal left approach tails (X -180.0..-85.32, 2x)", "H", (-G_HALF) - LEFT_END_X),
    ("Horizontal bay span rails (X 85.32..352.93, 2x)", "H", (BAY_P - G_HALF) - (G_HALF)),
    ("Horizontal right overhang rails (X 523.57..750.0, 2x)", "H", RIGHT_END_X - (BAY_P + G_HALF)),
]

# Pod frame -> Hub frame (proper rotation, det=+1):
#   x_hub = -Y_pod
#   y_hub = -62.475 - Z_pod  (rail center Z_pod = -12.475 -> y_hub = -50.0)
#   z_hub = +X_pod           (Pod rail +X -> vertical +Z)
LOC_POD_TO_HUB = bd.Plane(origin=(0.0, -62.475, 0.0), x_dir=(0, 0, 1), z_dir=(0, -1, 0)).location


def to_hub(shape, loc_hub: bd.Location = bd.Location((0, 0, 0))):
    """Map a Pod-frame Shape/Compound into Hub frame using LOC_POD_TO_HUB and optional hub world Location."""
    full = loc_hub * LOC_POD_TO_HUB
    if isinstance(shape, bd.Compound):
        return bd.Compound(children=[s.moved(full) for s in shape.solids()])
    return shape.moved(full)


def hub_world_loc(z_hub: float, rot_y_deg: float = 0.0, x_hub: float = 0.0) -> bd.Location:
    """World transform of a hub centered at (x_hub, -50, z_hub) tilted rot_y_deg about global Y through the rail."""
    # Since LOC_POD_TO_HUB already puts rail center at y=-50, rotate around (x_hub, RAIL_Y, z_hub) then translate (x_hub, 0, z_hub).
    return bd.Location((x_hub, 0.0, z_hub)) * bd.Location((0, RAIL_Y, 0), (0, rot_y_deg, 0)) * bd.Location((0, -RAIL_Y, 0))


def build_column_bar() -> bd.Part:
    """Standard 40 x 25 x 2.0 mm hollow aluminium rectangular box bar (340 mm long, ~224 g) with 8x Ø4.5 front-to-back holes
    and 0.6 mm engraved orientation/spec markings on its faces."""
    with bd.BuildPart() as col:
        with bd.Locations((0.0, (COL_Y0 + COL_Y1) / 2.0, COL_Z0)):
            bd.Box(COL_W, COL_D, COL_Z1 - COL_Z0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
            bd.Box(COL_W - 2 * COL_WALL, COL_D - 2 * COL_WALL, COL_Z1 - COL_Z0 + 2.0,
                   align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
        # 8x Ø4.5 mm through-holes matching both P04 top plates (X = +-12, Z_rel = -62, -2) for front-inserted M4 x 35 bolts
        for tag, zh in (("lower", JZ_LO), ("upper", JZ_UP)):
            for zp in (-62.0, -2.0):
                for sx in (-12.0, 12.0):
                    with bd.Locations(bd.Location((sx, COL_Y0 - 1.0, zh + HUB_SIGN[tag] * zp), (-90, 0, 0))):
                        bd.Cylinder(radius=2.25, height=COL_D + 2.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
    part = col.part
    yc = (COL_Y0 + COL_Y1) / 2.0
    zc = (COL_Z0 + COL_Z1) / 2.0
    part = O.engrave(part, "AL 40x25x2 FRONT", (0.0, COL_Y0, zc), (0, 0, 1), (0, -1, 0), 4.5)
    part = O.engrave(part, "TOP H1", (0.0, COL_Y0, COL_Z1 - 8.0), (1, 0, 0), (0, -1, 0), 3.4)
    part = O.engrave(part, "BOT H2", (0.0, COL_Y0, COL_Z0 + 8.0), (1, 0, 0), (0, -1, 0), 3.4)
    part = O.engrave(part, "REAR -> LEGS", (0.0, COL_Y1, zc), (0, 0, 1), (0, 1, 0), 4.0)
    part = O.engrave(part, "+X", (COL_W / 2.0, yc, zc), (0, 1, 0), (1, 0, 0), 4.5)
    part = O.engrave(part, "-X", (-COL_W / 2.0, yc, zc), (0, -1, 0), (-1, 0, 0), 4.5)
    part.label = "Option1_Column_Hollow_30x25x2"
    part.color = bd.Color(0.68, 0.72, 0.78)
    return part


def build_anti_crush_inserts() -> bd.Compound:
    """Two 25.8 x 20.8 x 80 mm internal PETG anti-crush plugs (P07-B01) seated inside the hollow tube ends behind H13 through-bolts."""
    plug_w = COL_W - 2 * COL_WALL - 0.2      # 25.8 mm (0.1 mm slip clearance per side)
    plug_d = COL_D - 2 * COL_WALL - 0.2      # 20.8 mm
    y_plug_front = (COL_Y0 + COL_Y1) / 2.0 - plug_d / 2.0
    plugs = []
    for tag, zh in (("lower", JZ_LO), ("upper", JZ_UP)):
        z0 = zh - 72.0 if HUB_SIGN[tag] > 0 else zh - 8.0                   # plug behind that pod's 4 column bolts
        with bd.BuildPart() as pl:
            with bd.Locations((0.0, (COL_Y0 + COL_Y1) / 2.0, z0)):
                bd.Box(plug_w, plug_d, 80.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
            for zp in (-62.0, -2.0):
                for sx in (-12.0, 12.0):
                    with bd.Locations(bd.Location((sx, COL_Y0 - 1.0, zh + HUB_SIGN[tag] * zp), (-90, 0, 0))):
                        bd.Cylinder(radius=2.25, height=COL_D + 2.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
        plug = O.engrave(pl.part, f"P07-{O.BATCH}", (0.0, y_plug_front, z0 + 40.0), (1, 0, 0), (0, -1, 0), 4.0)
        plugs.append(plug)
    cmp = bd.Compound(children=plugs, label="Option1_Column_AntiCrush_Inserts")
    cmp.color = bd.Color(0.95, 0.58, 0.16)
    return cmp


def build_sensors_and_magnets():
    """Combined non-contact Hall sensor + magnet layout + printable slip-on corner pocket (P08):
      1. Swivel angle confirmation (top joint, P04 <-> P01):
           - 1x Ø3 x 1.8 mm magnet on P01 blue arm hub rim (-9.4, +9.4, Z_pod 71.5..73.3)
           - 2x A3144 digital Hall switches on P04 underside at 0 deg (-9.4, +9.4) & 90 deg (-9.4, -9.4), Z_pod 74.8..77.3
      2. Crossing arrival differential pair (near rail plane):
           - U-shaped slip-on PETG pocket (P08) wrapping MT0134's (-X, +Y) corner edge & slot (-27.70, +19.85),
             locked with 1x H17 M3 x 12 top socket cap bolt + 1x H22 captive M3 hex nut underneath at (-27.70, +21.60),
             dropping to (-42.0, +42.0, Z_pod -28.3) with 2x 49E Hall ICs (8 mm apart)
           - 2x Ø5 x 1.2 mm NdFeB magnets on each M12 fx_bracket at (-42, +42) (vertical) and (-42, -42) (horizontal)
    """
    # Stationary on P04 underside (does not turn with leg): P09 bridge holding 2x A3144 at (-11.0, ±11.0)
    # Clears M4 captive hex nuts (X >= -5.65) by 3.35 mm, r=11 boss by >2 mm, and servo cage (X <= -18.25)
    with bd.BuildPart() as sfix_bp:
        with bd.Locations((-13.6, 0.0, 76.9)):
            bd.Box(2.4, 24.0, 1.5, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
        for sy_s in (11.0, -11.0):
            with bd.Locations((-11.0, sy_s, 75.0)):
                bd.Box(4.0, 4.4, 3.4, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
            with bd.Locations((-12.3, sy_s, 76.9)):
                bd.Box(3.0, 4.4, 1.5, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
    sensors_fixed = bd.Compound(children=[sfix_bp.part], label="Sensors_Swivel_A3144_Pair")

    # P08 monoblock slip-on pocket wrapping MT0134 (-X, +Y) corner edge + outer slot (-27.70, +19.85), clamped by M3 x 12 + hex nut at (-27.85, +21.60)
    sx_b, sy_b = -27.85, 21.60
    with bd.BuildPart() as pkt:
        # Continuous pocket + wide cantilever shoulder wrapping MT0134 corner cleanly (Z = -4.2 .. 6.2)
        with bd.Locations((-35.8, 15.0, -4.2)):
            bd.Box(14.3, 20.8, 10.4, align=(bd.Align.MIN, bd.Align.MIN, bd.Align.MIN))
        with bd.Locations((-40.5, 34.5, -4.2)):
            with bd.BuildSketch():
                bd.RectangleRounded(17.0, 21.0, 2.5)
            bd.extrude(amount=10.4)
        # Full-width top clamping block rising cleanly above P01 deck (Z = 8.50) so M3 head seats flat
        with bd.Locations((-29.75, 21.6, 5.5)):
            with bd.BuildSketch():
                bd.RectangleRounded(12.1, 10.2, 2.5)
            bd.extrude(amount=4.2)
        # Continuous tapered gusset spine dropping smoothly from pocket shoulder to the 49E crossing tip (no stair notches)
        with bd.BuildSketch(bd.Plane(origin=(-49.0, 0.0, 0.0), x_dir=(0, 1, 0), z_dir=(1, 0, 0))):
            with bd.BuildLine():
                bd.Polyline(
                    (25.0, 6.2),
                    (45.0, 6.2),
                    (45.0, -26.8),
                    (38.5, -26.8),
                    (30.5, -4.2),
                    (25.0, -4.2),
                    (25.0, 6.2),
                )
            bd.make_face()
        bd.extrude(amount=14.0)
        # Cut exact MT0134 corner + 0.13 mm slip clearance so plate drops snugly inside pocket
        with bd.Locations((0.0, 0.0, -0.12)):
            with bd.BuildSketch():
                bd.RectangleRounded(65.76, 65.76, 3.13)
            bd.extrude(amount=3.24, mode=bd.Mode.SUBTRACT)
        # Cut P01 deck + 0.15 mm clearance
        with bd.Locations((5.5, 0.0, 2.98)):
            bd.Box(63.3, 52.3, 5.64, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
        # Cut Ø10 V-wheel spacer clearance on bottom jaw
        with bd.Locations((-20.0, 19.85, -5.0)):
            bd.Cylinder(radius=5.12, height=6.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
        # M3 thru-bolt bore + side-entry captive M3 hex nut slot (-X entry, flat facing +X spacer)
        with bd.Locations((sx_b, sy_b, -2.65)):
            bd.Cylinder(radius=1.65, height=13.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
        with bd.Locations((sx_b, sy_b, -2.60)):
            with bd.BuildSketch():
                bd.RegularPolygon(radius=5.70 / math.sqrt(3.0), side_count=6, rotation=30.0)
            bd.extrude(amount=2.50, mode=bd.Mode.SUBTRACT)
        with bd.Locations((-36.5, sy_b, -2.60)):
            bd.Box(9.0, 5.70, 2.50, align=(bd.Align.MIN, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)

    wing = pkt.part
    # Magnet ear on P01 hub outer rim (-11.0, +11.0) + Ø3 x 1.8 mm swivel magnet (clears r=15 hub & -18.25 servo cage during 0..90 deg sweep)
    with bd.BuildPart() as mlug:
        with bd.Locations((-11.0, 11.0, 68.5)):
            bd.Cylinder(radius=2.1, height=3.2, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
        with bd.Locations((0.0, 0.0, 60.5)):
            bd.Cylinder(radius=15.08, height=12.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
    mag_swiv = (
        mlug.part
        + bd.Cylinder(1.5, 1.8, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN)).moved(bd.Location((-11.0, 11.0, 71.7)))
    )
    ic_a = bd.Box(4.0, 3.0, 1.5, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN)).moved(bd.Location((-46.0, 42.0, -28.3)))
    ic_b = bd.Box(4.0, 3.0, 1.5, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN)).moved(bd.Location((-38.0, 42.0, -28.3)))
    sensors_turn = bd.Compound(children=[mag_swiv, wing, ic_a, ic_b], label="Sensors_Crossing_49E_Pair_And_Swivel_Mag")

    # Fastening hardware: 1x H17 M3 x 12 DIN 912 socket cap screw (top) + 1x H22 M3 DIN 934 hex nut (captive bottom)
    m3_bolt = (
        bd.Cylinder(2.75, 3.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN)).moved(bd.Location((sx_b, sy_b, 9.70)))
        + bd.Cylinder(1.34, 12.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MAX)).moved(bd.Location((sx_b, sy_b, 9.70)))
    )
    with bd.BuildPart() as hn:
        with bd.Locations((sx_b, sy_b, -2.55)):
            with bd.BuildSketch():
                bd.RegularPolygon(radius=5.50 / math.sqrt(3.0), side_count=6, rotation=30.0)
            bd.extrude(amount=2.40)
        with bd.Locations((sx_b, sy_b, -3.0)):
            bd.Cylinder(1.50, 3.5, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
    sensors_hardware = bd.Compound(children=[m3_bolt, hn.part], label="Sensors_Pocket_M3_Hardware")

    # World-frame target magnets mounted on all 4 M12 crossing brackets (Z_pod = -30.475 .. -29.3 -> 1.0 mm air gap below 49E)
    cross_mags = []
    for bx in BAY_XS:
        for zc in (JZ_LO, JZ_UP):
            loc = hub_world_loc(zc, 0.0, bx)
            for mx, my in ((-42.0, 42.0), (-42.0, -42.0)):
                m = bd.Cylinder(2.5, 1.2, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN)).moved(bd.Location((mx, my, -30.475)))
                cross_mags.append(to_hub(m, loc))
    crossing_magnets = bd.Compound(children=cross_mags, label="Crossing_Target_Magnets_NdFeB")
    return sensors_fixed, sensors_turn, sensors_hardware, crossing_magnets, wing


def build_horiz_rail_and_rack(x0: float, x1: float, zc: float):
    """Horizontal 2020 T-slot rail + exact-comb in-slot rack from x0 to x1 at Z = zc, Y = -50."""
    m_horiz = bd.Plane(origin=(0.0, -62.475, zc), x_dir=(1, 0, 0), z_dir=(0, -1, 0)).location
    r = XC.rail(x0, x1).moved(m_horiz)
    k = XC.comb(x0, x1).moved(m_horiz)
    return r, k


def build_vert_rail_and_rack(z0: float, z1: float, xc: float):
    """Vertical 2020 T-slot rail + exact-comb in-slot rack from z0 to z1 at X = xc, Y = -50 (tooth phase synced to JZ_LO)."""
    loc = hub_world_loc(JZ_LO, 0.0, xc) * LOC_POD_TO_HUB
    r = XC.rail(z0 - JZ_LO, z1 - JZ_LO).moved(loc)
    k = XC.comb(z0 - JZ_LO, z1 - JZ_LO).moved(loc)
    return r, k


def build_complete_railing_set():
    """Return fixed rails, fixed racks, fixed crossing hardware, and turning crossing templates."""
    v_rails, v_racks, h_rails, h_racks, fx_cross = [], [], [], [], []
    # Base crossing geometry at 17.5 pitches
    turning_tpl, fixed_tpl = XC.crossing_parts(SEG)

    # Vertical lines at Bay 0 (X = 0) and Bay 1 (X = 438.25)
    for bx in BAY_XS:
        for z0, z1 in ((JZ_LO + G_HALF, JZ_UP - G_HALF), (JZ_UP + G_HALF, RAIL_TOP)):
            r, k = build_vert_rail_and_rack(z0, z1, bx)
            v_rails.append(r)
            v_racks.append(k)
        # Crossing fixed housings & brackets at (bx, JZ_LO) and (bx, JZ_UP)
        for zc in (JZ_LO, JZ_UP):
            loc = hub_world_loc(zc, 0.0, bx)
            for fk in ("fx_pivot_housing", "fx_housing_screws", "fx_bracket"):
                fx_cross.extend(s.moved(loc * LOC_POD_TO_HUB) for s in (
                    fixed_tpl[fk].solids() if isinstance(fixed_tpl[fk], bd.Compound) else [fixed_tpl[fk]]
                ))

    # Horizontal lines at JZ_LO (90.0) and JZ_UP (330.33)
    h_spans = (
        (LEFT_END_X, -G_HALF),
        (G_HALF, BAY_P - G_HALF),
        (BAY_P + G_HALF, RIGHT_END_X),
    )
    for zc in (JZ_LO, JZ_UP):
        for x0, x1 in h_spans:
            r, k = build_horiz_rail_and_rack(x0, x1, zc)
            h_rails.append(r)
            h_racks.append(k)

    # Reference 2-bay slotted-angle rack skeleton behind Y = 0 (38 x 38 x 2 posts + 5 shelf ledges)
    rack_solids = []
    bay_centers = (230.0, 230.0 + BAY_P)
    for bc in bay_centers:
        for px in (bc - 153.0 - 38.0, bc + 153.0):
            for py in (0.0, 230.0 - 38.0):
                rack_solids.append(bd.Box(38.0, 38.0, 1000.0, align=(bd.Align.MIN,) * 3).moved(bd.Location((px, py, 0.0))))
        for z_lvl in PV.LEVELS:
            rack_solids.append(bd.Box(382.0, 230.0, 3.0, align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.MAX)).moved(
                bd.Location((bc, 0.0, z_lvl))))

    return {
        "rails_vert": bd.Compound(children=v_rails, label="Vertical_2020_Rails"),
        "racks_vert": bd.Compound(children=v_racks, label="Vertical_InSlot_Racks"),
        "rails_horiz": bd.Compound(children=h_rails, label="Horizontal_2020_Rails"),
        "racks_horiz": bd.Compound(children=h_racks, label="Horizontal_InSlot_Racks"),
        "crossings_fixed": bd.Compound(children=fx_cross, label="Crossings_Fixed_Brackets_Housings"),
        "rack_bays": bd.Compound(children=rack_solids, label="Ref_2Bay_Rack_Frames"),
        "turning_tpl": turning_tpl,
    }


def check_dual_legs_clearance():
    """Verify both legs + column_bar + rails have zero clashes at vertical (0 deg) and horizontal (-90 deg)."""
    pod_parts = O.build_assembled_parts()
    col = build_column_bar()
    inserts = build_anti_crush_inserts()
    s_fix, s_turn, s_hw, c_mags, _ = build_sensors_and_magnets()
    env = build_complete_railing_set()

    turn_keys = ("gantry_plate", "v_wheels", "motor_housing", "motor_shaft", "retainer",
                 "bearing_pinion", "carriage_pod", "motor_cap", "pinion_cover")
    fixed_keys = ("body_mount", "bearing_bot", "bearing_top", "m8_hardware", "servo_body", "fasteners_fixed")
    gear_keys = ("gear_20t", "alum_horn", "servo_spline")

    def build_pose(rot_y_deg: float, x_off: float = 0.0, z_off: float = 0.0):
        hub_bodies = {}
        for tag, zh in (("upper", JZ_UP), ("lower", JZ_LO)):
            loc_fix = hub_world_loc(zh + z_off, HUB_BASE[tag], x_off)
            loc_turn = hub_world_loc(zh + z_off, HUB_BASE[tag] + rot_y_deg, x_off)
            for k in fixed_keys:
                hub_bodies[f"{tag}_{k}"] = to_hub(pod_parts[k], loc_fix)
            for k in turn_keys:
                hub_bodies[f"{tag}_{k}"] = to_hub(pod_parts[k], loc_turn)
            # Servo gear turns -2 * pod_rot about (SERVO_PIVOT_X, 0, 0) in Pod frame
            piv = bd.Location((O.SERVO_PIVOT_X, 0, 0))
            pod_ang = -rot_y_deg
            loc_g = loc_fix * LOC_POD_TO_HUB * piv * bd.Location((0, 0, 0), (0, 0, -2.0 * pod_ang)) * piv.inverse()
            for k in gear_keys:
                s = pod_parts[k]
                hub_bodies[f"{tag}_{k}"] = bd.Compound(children=[q.moved(loc_g) for q in s.solids()]) if isinstance(s, bd.Compound) else s.moved(loc_g)
        return hub_bodies

    def overlap(a, b):
        ba, bb = a.bounding_box(), b.bounding_box()
        if (ba.min.X > bb.max.X or bb.min.X > ba.max.X or ba.min.Y > bb.max.Y or bb.min.Y > ba.max.Y
                or ba.min.Z > bb.max.Z or bb.min.Z > ba.max.Z):
            return 0.0
        inter = a.intersect(b)
        return sum(s.volume for s in inter.solids()) if inter is not None and hasattr(inter, "solids") else 0.0

    bad = []
    # Check column <-> every body + upper <-> lower hub + both hubs <-> fixed rails/brackets at 0 deg (vertical) and -90 deg (horiz)
    for label, rot_y, dx, dz in (
        ("vertical 0 deg @ bay0", 0.0, 0.0, 0.0),
        ("vertical climb Z+150", 0.0, 0.0, 150.0),
        ("horiz -90 deg @ bay0", -90.0, 0.0, 0.0),
        ("horiz travel X+220", -90.0, 220.0, 0.0),
    ):
        col_m = col.moved(bd.Location((dx, 0.0, dz)))
        ins_m = bd.Compound(children=[s.moved(bd.Location((dx, 0.0, dz))) for s in inserts.solids()])
        hb = build_pose(rot_y, dx, dz)
        upper_all = [(k, v) for k, v in hb.items() if k.startswith("upper_")]
        lower_all = [(k, v) for k, v in hb.items() if k.startswith("lower_")]
        v_ins = overlap(col_m, ins_m)
        if v_ins > 1e-3:
            bad.append((label, "column", "anti_crush_inserts", v_ins))
        # Column + anti-crush inserts vs both hubs (excluding threaded H13 shaft/washer contact against P04/front)
        for k, s in hb.items():
            if k.endswith("fasteners_fixed"):
                continue
            for tag_c, sc in (("column", col_m), ("inserts", ins_m)):
                v = overlap(sc, s)
                if v > 1e-3:
                    bad.append((label, tag_c, k, v))
        # Check Option B M4 x 35 bolts + steel/rubber washers + hex nuts against P04 body_mount and servo_body specifically
        z_tube_front = O.TOP_PLATE_Z0 + 4.5 + COL_D
        col_fast = bd.Compound(children=[
            O._cz(4.5, z_tube_front, z_tube_front + 1.2, x, y) +
            O._cz(4.5, z_tube_front + 1.2, z_tube_front + 2.0, x, y) +
            O._cz(3.5, z_tube_front + 2.0, z_tube_front + 6.0, x, y) +
            O._cz(2.0, z_tube_front + 2.0 - 35.0, z_tube_front + 2.0, x, y) +
            O._hexz(4.04, O.TOP_PLATE_Z0 - 3.2, O.TOP_PLATE_Z0, x, y, rot=30.0)
            for x, y in O.COLUMN_SCREWS
        ])
        for zh in (JZ_LO, JZ_UP):
            tag = "upper" if zh == JZ_UP else "lower"
            cf = to_hub(col_fast, hub_world_loc(zh + dz, HUB_BASE[tag], dx))
            for k_tgt in (f"{tag}_body_mount", f"{tag}_servo_body"):
                v_f = overlap(cf, hb[k_tgt])
                if v_f > 1e-3:
                    bad.append((label, f"{tag}_H13_column_bolts", k_tgt, v_f))
        # Upper hub vs lower hub
        for ku, su in upper_all:
            for kl, sl in lower_all:
                if ku.endswith("fasteners_fixed") or kl.endswith("fasteners_fixed"):
                    continue
                v = overlap(su, sl)
                if v > 1e-3:
                    bad.append((label, ku, kl, v))
        # Hubs vs fixed rails / brackets (crossing turntables live at crossings)
        for rk in ("rails_vert", "rails_horiz", "crossings_fixed"):
            rs = env[rk]
            for k, s in hb.items():
                if k.endswith(("motor_shaft", "v_wheels")) and ("rails" in rk):
                    continue  # V-wheels ride the rails; fixed rail clears pod
                v = overlap(s, rs)
                if v > 1e-3:
                    bad.append((label, k, rk, v))
        # Check sensors + M3 pocket hardware <-> fixed rails / brackets / column / inserts / legs
        for tag_zh, zh in (("upper", JZ_UP), ("lower", JZ_LO)):
            sf_w = to_hub(s_fix, hub_world_loc(zh + dz, HUB_BASE[tag_zh], dx))
            st_w = to_hub(s_turn, hub_world_loc(zh + dz, HUB_BASE[tag_zh] + rot_y, dx))
            sh_w = to_hub(s_hw, hub_world_loc(zh + dz, HUB_BASE[tag_zh] + rot_y, dx))
            for k_tgt in ("rails_vert", "rails_horiz", "crossings_fixed"):
                v1, v2, v3 = overlap(sf_w, env[k_tgt]), overlap(st_w, env[k_tgt]), overlap(sh_w, env[k_tgt])
                if v1 > 1e-3: bad.append((label, f"{tag_zh}_sensors_fixed", k_tgt, v1))
                if v2 > 1e-3: bad.append((label, f"{tag_zh}_sensors_turn", k_tgt, v2))
                if v3 > 1e-3: bad.append((label, f"{tag_zh}_sensors_hardware", k_tgt, v3))
            if overlap(st_w, sf_w) > 1e-3:
                bad.append((label, f"{tag_zh}_sensors_turn", f"{tag_zh}_sensors_fixed", overlap(st_w, sf_w)))
            if overlap(sh_w, st_w) > 1e-3:
                bad.append((label, f"{tag_zh}_sensors_hardware", f"{tag_zh}_sensors_turn", overlap(sh_w, st_w)))
            if overlap(st_w, c_mags) > 1e-3:
                bad.append((label, f"{tag_zh}_sensors_turn", "crossing_magnets", overlap(st_w, c_mags)))
            if overlap(st_w, col_m) > 1e-3:
                bad.append((label, f"{tag_zh}_sensors_turn", "column", overlap(st_w, col_m)))
            for pk_chk in ("carriage_pod", "gantry_plate", "v_wheels", "pinion_cover"):
                v_p = overlap(st_w, hb[f"{tag_zh}_{pk_chk}"])
                v_h = overlap(sh_w, hb[f"{tag_zh}_{pk_chk}"])
                if v_p > 1e-3:
                    bad.append((label, f"{tag_zh}_sensors_turn", f"{tag_zh}_{pk_chk}", v_p))
                if v_h > 1e-3:
                    bad.append((label, f"{tag_zh}_sensors_hardware", f"{tag_zh}_{pk_chk}", v_h))
            for pk_chk in ("body_mount", "servo_body", "bearing_top"):
                v_f = overlap(sf_w, hb[f"{tag_zh}_{pk_chk}"])
                if v_f > 1e-3:
                    bad.append((label, f"{tag_zh}_sensors_fixed", f"{tag_zh}_{pk_chk}", v_f))
        print(f"[{label:24s}] collisions so far: {len(bad)}")

    if bad:
        for item in bad:
            print("  COLLISION:", item)
    else:
        print("RESULT: OK - 0 collisions between upper leg, lower leg, column bar, and complete rails")
    return len(bad) == 0


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    os.makedirs(SIM_MESH_DIR, exist_ok=True)

    print("=== Complete 2-Bay 2020 Railing Cut List ===")
    for name, axis, length in RAIL_CUT_LIST:
        print(f"  {name:52s} -> {length:6.2f} mm")
    print(f"  {'Hub center spacing D_HUB (51 * P)':52s} -> {D_HUB:6.2f} mm (JZ_LO={JZ_LO}, JZ_UP={JZ_UP:.2f})")
    print(f"  {'Bay pitch BAY_P (93 * P)':52s} -> {BAY_P:6.2f} mm")
    print(f"  {'Hollow aluminium column bar (40 x 25 x 2.0)':52s} -> {COL_Z1 - COL_Z0:6.2f} mm (~224 g)")

    col = build_column_bar()
    inserts = build_anti_crush_inserts()
    s_fix, s_turn, s_hw, c_mags, sensor_bracket = build_sensors_and_magnets()
    env = build_complete_railing_set()
    export_map = {
        "column_bar": col,
        "anti_crush_inserts": inserts,
        "sensors_pod_fixed": s_fix,
        "sensors_pod_turn": s_turn,
        "sensors_hardware": s_hw,
        "crossing_magnets": c_mags,
        "rails_vert": env["rails_vert"],
        "racks_vert": env["racks_vert"],
        "rails_horiz": env["rails_horiz"],
        "racks_horiz": env["racks_horiz"],
        "crossings_fixed": env["crossings_fixed"],
        "rack_bays": env["rack_bays"],
    }
    for k, shape in export_map.items():
        bd.export_stl(shape, os.path.join(SIM_MESH_DIR, f"{k}.stl"), tolerance=0.05, angular_tolerance=0.2)
    bd.export_stl(inserts.solids()[0], os.path.join(OUT_DIR, "opt1_column_anticrush_plug.stl"), tolerance=0.01, angular_tolerance=0.1)
    bd.export_stl(sensor_bracket, os.path.join(OUT_DIR, "opt1_crossing_sensor_bracket.stl"), tolerance=0.01, angular_tolerance=0.1)
    print(f"Exported {len(export_map)} dual-leg environment meshes to {SIM_MESH_DIR}")

    # Export assembled Dual-Leg + Complete Railing STEP (0 deg vertical bay-0 nominal)
    pod_parts = O.build_assembled_parts()
    asm_children = [col, inserts, c_mags, env["rails_vert"], env["racks_vert"], env["rails_horiz"], env["racks_horiz"],
                    env["crossings_fixed"], env["rack_bays"]]
    for tag, zh in (("lower", JZ_LO), ("upper", JZ_UP)):
        loc = hub_world_loc(zh, HUB_BASE[tag], 0.0)
        asm_children.append(to_hub(s_fix, loc))
        asm_children.append(to_hub(s_turn, loc))
        asm_children.append(to_hub(s_hw, loc))
        for pk, ps in pod_parts.items():
            if pk in ("t_rail", "t_rack"):
                continue
            asm_children.append(to_hub(ps, loc))
    for bx in BAY_XS:
        for zc in (JZ_LO, JZ_UP):
            loc = hub_world_loc(zc, 0.0, bx)
            for tk, ts in env["turning_tpl"].items():
                asm_children.append(ts.moved(loc * LOC_POD_TO_HUB))
    step_path = os.path.join(OUT_DIR, "opt1_dual_legs_on_rails.step")
    bd.export_step(bd.Compound(children=asm_children, label="Option1_Dual_Legs_Complete_Rails"), step_path)
    print(f"Exported STEP assembly -> {step_path}")

    ok = check_dual_legs_clearance()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
