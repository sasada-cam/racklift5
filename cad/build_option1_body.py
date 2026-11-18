"""Option 1 Separate Body & Picker Assembly (3D-Printed Ribbed Tray + Y-Shape Structural Joints).

Builds the non-swivelling robot payload body decoupled from the legs:
  - Body Attachment Spine (B01): 3-sided U-collar + P07-BODY anti-crush plug + 4x M4 bolts/EPDM/nuts
    clamping onto the 40 x 25 x 2.0 mm hollow aluminium column mid-span (Z = 138..226 mm, strictly
    Y <= -145.6 so H1/H2 pods pass behind freely).
  - Right (+X) Y-Gusseted Tray Boom (B02) -> 5 kg Bar Load Cell -> Y-Spread Live Saddle -> 3D-Printed
    Ribbed PETG Tray Platform (B04, 180 x 175 mm, 1-piece printable on 220x220 bed, top Z = 178.0 mm):
      * Proportional prototype footprint (no plywood slab), paired with 2-stage telescopic arms (145 mm
        bars) + 4x flip fingers + 140 x 130 x 80 mm prototype basket + matching prototype rack bay.
  - Left (-X) Compact Counterweight Enclosure + Y-Fork Cradle (B03): slim 78 x 46 x 66 mm battery/PCB box
    firmly cupped by a dual-branch Y-shaped pocket wrapping front, bottom, and rear walls.

Usage:
  pixi run python cad/build_option1_body.py
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
import params_v3 as PV                      # noqa: E402

OUT_DIR = os.path.abspath(os.path.join(HERE, "..", "models", "components", "option1_body"))
SIM_MESH_DIR = os.path.abspath(os.path.join(HERE, "..", "sim", "build", "meshes_option1_body"))

# --- Shared Robot & Column Constants (mm, world frame at poseX=0, poseZ=0) ---
JZ_LO = PV.JZ_LO                            # 90.0 mm
JZ_UP = PV.JZ_UP                            # 330.33 mm
CZ_HOME = PV.CZ_HOME                        # 210.165 mm
TRAY_TOP_Z = PV.LEVELS[0]                   # 178.0 mm (Shelf 1 top)
TRAY_RIB_Z0 = 168.0                         # bottom of printed perimeter frame / ribs (10 mm structural depth)
TRAY_DECK_T = 4.0                           # top flat 3D-printed skin Z = 174..178 mm

COL_W, COL_D, COL_WALL = 40.0, 25.0, 2.0
COL_Y1 = PV.COL_Y[1]                        # -145.5
COL_Y0 = COL_Y1 - COL_D                     # -170.5
COL_YC = (COL_Y0 + COL_Y1) / 2.0            # -158.0
COL_Z0, COL_Z1 = 10.165, 350.165

# Body Attachment Collar (B01) sits strictly in the leg-free mid-span
BODY_Z0, BODY_Z1 = 128.0, 226.0
BODY_COL_BOLTS_Z = (140.0, 214.0)
BODY_SIDE_BOLTS_Z = (146.0, 210.0)

# Compact Prototype Tray (B04: 180 x 175 mm printable flat deck) & Prototype Basket / Rack
BXR = 180.0                                 # prototype bay center X (trimmed to match compact 3D-printed tray)
TRAY_W = 180.0                              # X = 90.0 .. 270.0 mm
TRAY_D = 175.0                              # Y = -245.0 .. -70.0 mm
TRAY_X0, TRAY_X1 = BXR - TRAY_W / 2.0, BXR + TRAY_W / 2.0
TRAY_Y0, TRAY_Y1 = -245.0, -70.0

BASKET_W, BASKET_D, BASKET_H = 140.0, 130.0, 80.0
BASKET_TRAY_Y0 = -215.0                     # sits at Y = -215 .. -85 on the 175 mm tray
ARM_DX_IN, ARM_DX_OUT = 74.0, 87.0          # 13 mm wide telescopic bars at BXR ± (74..87), just outside 140 mm basket
ARM_W = ARM_DX_OUT - ARM_DX_IN              # 13.0 mm
ARM_L = 145.0                               # each bar 145 mm long
ARM_TIP0 = TRAY_Y1 - 5.0                    # -75.0 stowed tip
ARM_MID_Y0 = ARM_TIP0 - ARM_L               # -220.0
EXTEND_MAX = 205.0                          # max reach stroke into compact 180 mm deep Shelf 1 bay
FS_ZT = 62.0                                # finger shaft center above tray top (clears 80 mm? Wait: 62 + 32 > 80 -> 94!)
FINGER_L, FINGER_T = 26.0, 4.0
REAR_OFF = BASKET_D + FINGER_T + 2.0        # 136.0 mm capture span for 130 mm basket
RACK_HALF_OPEN = 92.0                       # compact prototype rack bay clear opening half-width (post at BXR ± 92..130)


def _box(x0, x1, y0, y1, z0, z1):
    return bd.Box(x1 - x0, y1 - y0, z1 - z0, align=(bd.Align.MIN,) * 3).moved(bd.Location((x0, y0, z0)))


def _cyl_axis(r, c0, c1, axis, cx, cy):
    if axis == "z":
        return bd.Cylinder(r, c1 - c0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN)).moved(bd.Location((cx, cy, c0)))
    if axis == "y":
        return bd.Cylinder(r, c1 - c0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN)).moved(bd.Location((cx, c0, cy), (-90, 0, 0)))
    return bd.Cylinder(r, c1 - c0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN)).moved(bd.Location((c0, cx, cy), (0, 90, 0)))


def _extrude_xy_poly(pts, z0, z1):
    poly = pts if pts[-1] == pts[0] else [*pts, pts[0]]
    with bd.BuildPart() as bp:
        with bd.BuildSketch(bd.Plane.XY.offset(z0)):
            with bd.BuildLine():
                bd.Polyline(*poly)
            bd.make_face()
        bd.extrude(amount=z1 - z0)
    return bp.part


def _extrude_xz_poly(pts, y0, y1):
    poly = pts if pts[-1] == pts[0] else [*pts, pts[0]]
    # x_dir=(1,0,0), z_dir=(0,-1,0) -> y_dir = z_dir x x_dir = (0,0,+1), so sketch Y aligns with +Z
    with bd.BuildPart() as bp:
        with bd.BuildSketch(bd.Plane(origin=(0.0, y1, 0.0), x_dir=(1, 0, 0), z_dir=(0, -1, 0))):
            with bd.BuildLine():
                bd.Polyline(*poly)
            bd.make_face()
        bd.extrude(amount=y1 - y0)
    return bp.part


def build_column_stub_and_plug():
    """Reference 40 x 25 x 2.0 x 340 mm hollow aluminium column + P07-BODY internal anti-crush plug."""
    col = _box(-COL_W / 2, COL_W / 2, COL_Y0, COL_Y1, COL_Z0, COL_Z1) - _box(
        -COL_W / 2 + COL_WALL, COL_W / 2 - COL_WALL, COL_Y0 + COL_WALL, COL_Y1 - COL_WALL, COL_Z0 - 1.0, COL_Z1 + 1.0
    )
    for zp in (JZ_LO - 62.0, JZ_LO - 2.0, JZ_UP - 62.0, JZ_UP - 2.0, *BODY_COL_BOLTS_Z):
        for sx in (-12.0, 12.0):
            col = col - _cyl_axis(2.25, COL_Y0 - 1.0, COL_Y1 + 1.0, "y", sx, zp)
    col = O.engrave(col, "BODY ZONE", (0.0, COL_Y0, 232.0), (1, 0, 0), (0, -1, 0), 3.6)

    with bd.BuildPart() as plg:
        with bd.Locations((0.0, COL_YC, (BODY_Z0 + BODY_Z1) / 2.0)):
            bd.Box(COL_W - 2 * COL_WALL - 0.2, COL_D - 2 * COL_WALL - 0.2, BODY_Z1 - BODY_Z0)
        for zp in BODY_COL_BOLTS_Z:
            for sx in (-12.0, 12.0):
                with bd.Locations(bd.Location((sx, COL_Y0, zp), (-90, 0, 0))):
                    bd.Cylinder(2.25, COL_D + 2.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
                with bd.Locations(bd.Location((sx, COL_Y1 - 9.0, zp), (-90, 0, 0))):
                    with bd.BuildSketch():
                        bd.RegularPolygon(radius=7.3 / math.sqrt(3.0), side_count=6, rotation=30.0)
                    bd.extrude(amount=3.5, mode=bd.Mode.SUBTRACT)
                with bd.Locations((sx, COL_Y1 - 7.25, zp)):
                    bd.Box(7.3, 3.5, 16.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
    plug = O.engrave(plg.part, f"P07-B{O.BATCH[1:]}", (0.0, COL_Y0 + COL_WALL + 0.1, 187.0), (1, 0, 0), (0, -1, 0), 4.0)
    return col, plug


def build_body_column_collar():
    """B01 Body Attachment U-Collar: wraps front + left + right faces of the 40 x 25 x 2.0 mm column
    (strictly Y <= -145.6 so ZERO protrusion behind COL_Y1 = -145.5 where H1/H2 swivel pods pass).
    """
    gap = 0.15
    x_out = COL_W / 2 + 6.5                              # 26.5
    y_out0, y_out1 = COL_Y0 - 6.5, COL_Y1 - 0.1          # -177.0 .. -145.6

    with bd.BuildPart() as bp:
        with bd.Locations((0.0, (y_out0 + y_out1) / 2.0, BODY_Z0)):
            bd.Box(2 * x_out, y_out1 - y_out0, BODY_Z1 - BODY_Z0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
        # Side mounting flanges (rear half Y = -164.0 .. -145.6 so B02/B03 Y-forks bolt onto front face Y = -164.15)
        for sx in (-1, 1):
            fx0 = sx * x_out if sx > 0 else -x_out - 12.0
            fx1 = x_out + 12.0 if sx > 0 else -x_out
            with bd.Locations(((fx0 + fx1) / 2, (-164.0 + y_out1) / 2.0, BODY_Z0 + 4.0)):
                bd.Box(abs(fx1 - fx0), y_out1 - (-164.0), BODY_Z1 - BODY_Z0 - 8.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
        with bd.Locations((0.0, COL_YC, BODY_Z0 - 1.0)):
            bd.Box(COL_W + 2 * gap, COL_D + 2 * gap, BODY_Z1 - BODY_Z0 + 2.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
        for zp in BODY_COL_BOLTS_Z:
            for sx in (-12.0, 12.0):
                with bd.Locations(bd.Location((sx, y_out0 - 1.0, zp), (-90, 0, 0))):
                    bd.Cylinder(2.25, 16.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
        for sx in (-1, 1):
            wx = sx * (x_out + 6.0)
            for zp in BODY_SIDE_BOLTS_Z:
                with bd.Locations(bd.Location((wx, -165.0, zp), (-90, 0, 0))):
                    bd.Cylinder(2.25, 22.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)

    collar = O.engrave(bp.part, f"B01-{O.BATCH}", (0.0, y_out0, 187.0), (1, 0, 0), (0, -1, 0), 4.2)

    fast_list = []
    for zp in BODY_COL_BOLTS_Z:
        for sx in (-12.0, 12.0):
            epdm = _cyl_axis(4.5, y_out0 - 1.2, y_out0, "y", sx, zp)
            wash = _cyl_axis(4.5, y_out0 - 2.0, y_out0 - 1.2, "y", sx, zp)
            head = _cyl_axis(3.5, y_out0 - 6.0, y_out0 - 2.0, "y", sx, zp)
            shank = _cyl_axis(2.0, y_out0 - 2.0, COL_Y1 - 5.8, "y", sx, zp)
            with bd.BuildPart() as hn:
                with bd.Locations(bd.Location((sx, COL_Y1 - 8.9, zp), (-90, 0, 0))):
                    with bd.BuildSketch():
                        bd.RegularPolygon(radius=7.0 / math.sqrt(3.0), side_count=6, rotation=30.0)
                    bd.extrude(amount=3.1)
            fast_list.append(epdm + wash + head + shank + hn.part)
    for sx in (-1, 1):
        wx = sx * (x_out + 6.0)
        for zp in BODY_SIDE_BOLTS_Z:
            bolt = _cyl_axis(3.5, -180.0, -176.0, "y", wx, zp) + _cyl_axis(2.0, -176.0, -145.6, "y", wx, zp)
            fast_list.append(bolt)
    return collar, bd.Compound(children=fast_list, label="Body_Attachment_Fasteners")


def build_tray_boom_and_loadcell():
    """B02 Y-Gusseted Boom + Sturdy Sub-Base + OP0019 Twin-Wishbone Load Cell Assembly centered at BXR = 180."""
    x_flange = COL_W / 2 + 6.5                      # 26.5
    lc_x0, lc_x1 = 140.0, 220.0                     # 80 mm TAL220 centered squarely under B04 (BXR = 180.0)
    lc_yc = -158.0
    lc_z0, lc_z1 = 145.5, 158.2                     # 7.5 mm gap above sturdy base (138.0), ~9.8 mm under B04 (168.0)

    # 1. B02: Y-gusseted cantilever boom from B01 +X wing -> sturdy 106 x 114 mm sub-base + fixed-end OP0019 pedestal
    flange = _box(x_flange + 0.15, 38.5, -176.0, -164.15, BODY_Z0 + 4.0, BODY_Z1 - 4.0)
    # Tapered Y-boom extending from X = 38.65..124.0 at Z = 132..140, plus upper diagonal XZ gusset from Z=222 down to X=145
    boom_arm = _extrude_xy_poly(
        [(38.65, -176.0), (124.0, -190.0), (124.0, -126.0), (38.65, -144.0)],
        132.0, 138.0
    )
    diag_gusset = _extrude_xz_poly(
        [(26.65, 222.0), (38.5, 222.0), (88.0, 164.0), (132.0, 140.0), (132.0, 132.0), (26.65, 132.0)],
        -174.0, -164.5
    )
    # Sturdy rectangular base platform centered under tray (X = 122..228, Y = -215..-101, Z = 132..138) + perimeter rim
    base_plate = _box(122.0, 228.0, -215.0, -101.0, 132.0, 138.0)
    # Integrated OP0019 lower wishbone pedestal clamping fixed left end (X = 138..167, Z = 138..145.4) + wide spread Y-buttress ribs
    pedestal_wish = _extrude_xy_poly(
        [(136.0, -208.0), (154.0, -208.0), (166.0, -168.0), (166.0, -148.0), (154.0, -108.0), (136.0, -108.0), (144.0, -148.0), (144.0, -168.0)],
        138.0, lc_z0 - 0.1
    )
    b02_raw = flange + boom_arm + diag_gusset + base_plate + pedestal_wish
    for zp in BODY_SIDE_BOLTS_Z:
        b02_raw = b02_raw - _cyl_axis(2.25, -180.0, -160.0, "y", 32.5, zp)
    # Two M5 thru-bores going UP from under sturdy base (X = 145, 160) into TAL220 fixed end + counterbore pockets
    for hx in (145.0, 160.0):
        b02_raw = b02_raw - _cyl_axis(2.65, 131.0, lc_z0 + 1.0, "z", hx, lc_yc)
    b02_part = O.engrave(b02_raw, f"B02-{O.BATCH}", (80.0, -174.0, 135.0), (1, 0, 0), (0, -1, 0), 3.0)

    # 2. Exact TAL220 5 kg Bar Load Cell (80 x 12.7 x 12.7 mm, X = 140..220, centered at BXR = 180)
    with bd.BuildPart() as lc:
        with bd.Locations((lc_x0, lc_yc - 6.35, lc_z0)):
            bd.Box(lc_x1 - lc_x0, 12.7, 12.7, align=(bd.Align.MIN, bd.Align.MIN, bd.Align.MIN))
        for hx in (145.0, 160.0):                     # 2x M5 fixed end
            with bd.Locations((hx, lc_yc, lc_z0 - 1.0)):
                bd.Cylinder(2.55, 14.7, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
        for hx in (200.0, 215.0):                     # 2x M4 live end
            with bd.Locations((hx, lc_yc, lc_z0 - 1.0)):
                bd.Cylinder(2.05, 14.7, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
        for hx in (173.0, 187.0):                     # twin shear-beam bores
            with bd.Locations(bd.Location((hx, lc_yc - 7.0, (lc_z0 + lc_z1) / 2), (-90, 0, 0))):
                bd.Cylinder(3.9, 14.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
        with bd.Locations((173.0, lc_yc - 7.0, (lc_z0 + lc_z1) / 2 - 2.3)):
            bd.Box(14.0, 14.0, 4.6, align=(bd.Align.MIN, bd.Align.MIN, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
    potting = _box(170.0, 190.0, lc_yc - 4.0, lc_yc + 4.0, lc_z1, lc_z1 + 0.6)
    hx711 = _box(172.0, 206.0, -206.0, -184.0, 138.1, 142.1)  # seated flat on sturdy B02 base plate clear of pedestal

    # 3. B05 Upper OP0019-Style Live Wishbone Spider (bolts onto TAL220 live end 200/215, spreads to 4 corners under B04)
    # Pedestal drop ONLY over live right end (X = 195..221, Z = 158.3..162.0) leaving >3.5 mm over shear zone/fixed end
    live_pad = _box(195.0, 221.0, lc_yc - 7.0, lc_yc + 7.0, lc_z1 + 0.10, 162.0)
    wish_arms = _extrude_xy_poly(
        [(131.0, -209.0), (144.0, -209.0), (195.0, -166.0), (202.0, -181.0), (215.0, -181.0), (221.0, -166.0),
         (221.0, -150.0), (215.0, -135.0), (202.0, -135.0), (195.0, -150.0), (144.0, -107.0), (131.0, -107.0),
         (186.0, -150.0), (186.0, -166.0)],
        161.8, TRAY_RIB_Z0 - 0.15
    )
    b05_raw = live_pad + wish_arms
    for hx in (200.0, 215.0):                         # 2x M4 bolts DOWN from top of B05 into TAL220 live holes (+ head counterbore)
        b05_raw = b05_raw - _cyl_axis(2.20, lc_z1, TRAY_RIB_Z0, "z", hx, lc_yc) - _cyl_axis(3.80, 164.6, TRAY_RIB_Z0 + 0.2, "z", hx, lc_yc)
    live_pts = ((137.5, -202.0), (137.5, -114.0), (208.5, -174.0), (208.5, -142.0))
    for tx, ty in live_pts:
        # 4x M4 thru-bores connecting B05 live wishbone UP into captive M4 hex nuts in B04 underside
        b05_raw = b05_raw - _cyl_axis(2.20, 161.0, TRAY_RIB_Z0, "z", tx, ty)
    b05_part = O.engrave(b05_raw, f"B05-{O.BATCH}", (221.0, -158.0, 164.8), (0, 1, 0), (1, 0, 0), 2.6)

    # 4. Fastening Hardware: 2x M5x20 UP into TAL220 fixed end + 2x M4x16 DOWN into live end + 4x M4x14 UP into B04 nuts
    lc_hw = []
    for hx in (145.0, 160.0):
        lc_hw.append(_cyl_axis(4.25, 128.0, 131.95, "z", hx, lc_yc) + _cyl_axis(2.45, 132.0, lc_z1 - 0.5, "z", hx, lc_yc))
    for hx in (200.0, 215.0):
        lc_hw.append(_cyl_axis(3.5, 164.8, 167.8, "z", hx, lc_yc) + _cyl_axis(1.95, lc_z0 + 0.5, 164.8, "z", hx, lc_yc))
    for tx, ty in live_pts:
        bolt_u = _cyl_axis(3.4, 159.2, 161.75, "z", tx, ty) + _cyl_axis(1.95, 161.8, TRAY_RIB_Z0 + 5.6, "z", tx, ty)
        with bd.BuildPart() as n4:
            with bd.Locations((tx, ty, TRAY_RIB_Z0 + 2.4)):
                with bd.BuildSketch():
                    bd.RegularPolygon(radius=7.0 / math.sqrt(3.0), side_count=6, rotation=0.0)
                bd.extrude(amount=3.1)
        lc_hw.append(bolt_u + (n4.part - _cyl_axis(2.0, TRAY_RIB_Z0 + 2.0, TRAY_RIB_Z0 + 6.0, "z", tx, ty)))

    return b02_part, b05_part, bd.Compound(children=[lc.part, potting, hx711], label="LoadCell_5kg_HX711"), bd.Compound(children=lc_hw, label="LoadCell_OP0019_Hardware")


def build_battery_pod():
    """B03 Compact Counterweight Enclosure (78 x 46 x 66 mm) cupped inside a Y-shaped pocket cradle onto B01 -X wing."""
    bx0, bx1 = -126.0, -48.0                        # compact counterweight footprint (reduced ~40% vs v3 brick)
    by0, by1 = -168.0, -122.0                       # 46 mm deep
    bz0, bz1 = 154.0, 220.0                         # 66 mm tall, safely inside mid-span (H2 ends 110, H1 starts 254)
    x_flange = -(COL_W / 2 + 6.5)                   # -26.5

    with bd.BuildPart() as b03:
        # Hollow compact electronics + 3S Li-ion enclosure
        with bd.Locations((bx0, by0, bz0)):
            bd.Box(bx1 - bx0, by1 - by0, bz1 - bz0, align=(bd.Align.MIN,) * 3)
        with bd.Locations((bx0 + 3.2, by0 + 3.2, bz0 + 3.5)):
            bd.Box((bx1 - bx0) - 6.4, (by1 - by0) - 6.4, bz1 - bz0, align=(bd.Align.MIN,) * 3, mode=bd.Mode.SUBTRACT)

    # Strong Y-shaped pocket cradle connecting B01 -X wing (X = -38.5..-26.65) to front, rear, and bottom of B03
    wing_plate = _box(-38.5, x_flange - 0.15, -176.0, -164.15, BODY_Z0 + 4.0, BODY_Z1 - 4.0)
    # Bottom Y-fork shelf wrapping underside of enclosure (Z = 144..154.05, stepping cleanly around B01 -X wing)
    y_fork_bot = _extrude_xy_poly(
        [(-38.65, -176.0), (-26.65, -176.0), (-26.65, -164.15), (-38.65, -164.15), (-38.65, -145.4), (-48.0, -118.0), (bx0 + 18.0, -118.0), (bx0 + 18.0, -168.0)],
        144.0, bz0 + 0.05
    )
    # Upper + Front Y-pocket cheek gripping full height of enclosure's right + front edges (strictly <= bz1 so B03L lid sits flat above)
    y_fork_front = _extrude_xz_poly(
        [(-38.5, bz1), (-26.65, bz1), (-26.65, 144.0), (-74.0, 144.0), (-74.0, bz0 + 30.0)],
        -172.0, by0 + 1.0
    )
    # Rear diagonal Y-arm bracing the rear (-122) outer wall cleanly down onto y_fork_bot
    y_fork_rear = _extrude_xz_poly(
        [(-48.0, 206.0), (-48.0, 144.0), (-76.0, 144.0), (-76.0, 194.0)],
        by1 - 1.0, by1 + 4.0
    )
    b03_full = b03.part + wing_plate + y_fork_bot + y_fork_front + y_fork_rear
    # Outer M3 screw-on rim ears (for separate flat-printable top lid B03L) + wire exit notch on +X rim
    for ex, ey in ((bx0 - 3.5, -145.0), (bx1 + 3.5, -145.0)):
        b03_full = b03_full + _box(ex - 4.5, ex + 4.5, ey - 6.0, ey + 6.0, bz1 - 8.0, bz1) - _cyl_axis(1.55, bz1 - 9.0, bz1 + 1.0, "z", ex, ey)
    b03_full = b03_full - _box(bx0 + 3.2, bx1 - 3.2, by0 + 3.2, by1 - 3.2, bz0 + 3.5, bz1 + 2.0)
    b03_full = b03_full - _box(bx1 - 4.0, bx1 + 1.0, -152.0, -138.0, bz1 - 6.0, bz1 + 1.0)  # rear wire slot to column/HX711
    for zp in BODY_SIDE_BOLTS_Z:
        b03_full = b03_full - _cyl_axis(2.25, -180.0, -160.0, "y", -32.5, zp)
    b03_part = O.engrave(b03_full, f"B03-{O.BATCH}", (-87.0, by0, 187.0), (1, 0, 0), (0, -1, 0), 4.0)

    # Separate 3D-printable flat top cover lid B03L (zero-support flat print, shields battery/PCB from top loads)
    lid = (
        _box(bx0 - 8.0, bx1 + 8.0, by0, by1, bz1 + 0.15, bz1 + 3.65)
        + _box(bx0 + 3.4, bx1 - 3.4, by0 + 3.4, by1 - 3.4, bz1 - 1.5, bz1 + 0.25)
    )
    for ex, ey in ((bx0 - 3.5, -145.0), (bx1 + 3.5, -145.0)):
        lid = lid - _cyl_axis(1.70, bz1 - 2.0, bz1 + 4.5, "z", ex, ey)
    lid_part = O.engrave(lid, f"B03L-{O.BATCH}", (-87.0, -145.0, bz1 + 3.65), (1, 0, 0), (0, 1, 0), 4.0)

    bat_pack = _box(bx0 + 5.0, bx1 - 5.0, by0 + 5.0, by1 - 18.0, bz0 + 5.0, bz1 - 12.0)
    ctrl_pcb = _box(bx0 + 7.0, bx1 - 7.0, by1 - 15.0, by1 - 11.0, bz0 + 10.0, bz1 - 6.0)
    return b03_part, lid_part, bd.Compound(children=[bat_pack, ctrl_pcb], label="Battery_Pack_And_PCB")


def build_tray_and_picker():
    """B04 Compact 3D-Printed Ribbed PETG Tray (180 x 175 mm) + 2-Stage Telescopic Arms + Flip Fingers."""
    z_t = TRAY_TOP_Z

    # 3D-printed ribbed PETG tray (replaces heavy plywood slab; prints 1-piece on 220x220 bed!)
    with bd.BuildPart() as b04:
        # Top flat 4.0 mm deck plate (Z = 174..178 mm)
        with bd.Locations((TRAY_X0, TRAY_Y0, z_t - TRAY_DECK_T)):
            bd.Box(TRAY_W, TRAY_D, TRAY_DECK_T, align=(bd.Align.MIN,) * 3)
        # Perimeter stiffening skirt (8 mm wide x 8 mm deep, Z = 166..174.1 mm)
        for rx0, rx1, ry0, ry1 in (
            (TRAY_X0, TRAY_X1, TRAY_Y0, TRAY_Y0 + 8.0),
            (TRAY_X0, TRAY_X1, TRAY_Y1 - 8.0, TRAY_Y1),
            (TRAY_X0, TRAY_X0 + 8.0, TRAY_Y0 + 8.0, TRAY_Y1 - 8.0),
            (TRAY_X1 - 8.0, TRAY_X1, TRAY_Y0 + 8.0, TRAY_Y1 - 8.0),
        ):
            with bd.Locations((rx0, ry0, TRAY_RIB_Z0)):
                bd.Box(rx1 - rx0, ry1 - ry0, (z_t - TRAY_DECK_T) - TRAY_RIB_Z0 + 0.1, align=(bd.Align.MIN,) * 3)
        # Structural X & Y cross-ribs spreading load smoothly from the Y-fork load-cell mount (X = 100..136, Y = -188..-128)
        for ry_c in (COL_YC, -110.0, -205.0):
            with bd.Locations((TRAY_X0 + 8.0, ry_c - 3.0, TRAY_RIB_Z0)):
                bd.Box(TRAY_W - 16.0, 6.0, (z_t - TRAY_DECK_T) - TRAY_RIB_Z0 + 0.1, align=(bd.Align.MIN,) * 3)
        for rx_c in (114.0, BXR, TRAY_X1 - 35.0):
            with bd.Locations((rx_c - 3.0, TRAY_Y0 + 8.0, TRAY_RIB_Z0)):
                bd.Box(6.0, TRAY_D - 16.0, (z_t - TRAY_DECK_T) - TRAY_RIB_Z0 + 0.1, align=(bd.Align.MIN,) * 3)
        # OP0019 Live Wishbone mounting pads under B04 (4x wide points: (137.5, ±) & (208.5, ±))
        for tx, ty in ((137.5, -202.0), (137.5, -114.0), (208.5, -174.0), (208.5, -142.0)):
            with bd.Locations((tx, ty, TRAY_RIB_Z0)):
                bd.Cylinder(8.0, (z_t - TRAY_DECK_T) - TRAY_RIB_Z0 + 0.1, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
            with bd.Locations((tx, ty, TRAY_RIB_Z0 - 0.5)):
                bd.Cylinder(2.20, 7.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
            with bd.Locations((tx, ty, TRAY_RIB_Z0 + 2.35)):
                with bd.BuildSketch():
                    bd.RegularPolygon(radius=7.30 / math.sqrt(3.0), side_count=6, rotation=0.0)
                bd.extrude(amount=3.3, mode=bd.Mode.SUBTRACT)
            slot_dy = -12.0 if ty < -158.0 else 12.0
            with bd.Locations((tx - 3.65, min(ty, ty + slot_dy), TRAY_RIB_Z0 + 2.35)):
                bd.Box(7.30, abs(slot_dy), 3.3, align=(bd.Align.MIN,) * 3, mode=bd.Mode.SUBTRACT)
        # Built-in rear/front raised lip stops (no separate screws needed)
        for sx in (-46.0, 46.0):
            with bd.Locations((BXR + sx - 7.0, BASKET_TRAY_Y0 - 6.0, z_t)):
                bd.Box(14.0, 6.0, 12.0, align=(bd.Align.MIN,) * 3)
    tray = O.engrave(b04.part, f"B04-{O.BATCH}", (BXR, TRAY_Y0, 170.0), (1, 0, 0), (0, -1, 0), 3.4)

    tray_hw = _box(BXR - 54.0, BXR - 36.0, BASKET_TRAY_Y0, BASKET_TRAY_Y0 + 6.4, z_t + 0.05, z_t + 10.05)

    outer_parts, mid_parts, in_parts, belts_list, servo_parts = [], [], [], [], []
    fingers_local = {-1: [], 1: []}
    out_z0, out_z1 = z_t + 22.0, z_t + 30.0
    mid_z0, mid_z1 = z_t + 34.0, z_t + 42.0
    in_z0, in_z1 = z_t + 46.0, z_t + 54.0
    zc_fs = z_t + FS_ZT

    for sgn in (-1, 1):
        xa = BXR + sgn * ARM_DX_IN if sgn > 0 else BXR - ARM_DX_OUT
        xb = xa + ARM_W
        # Fixed outer 13 x 8 bar + support posts on B04 printed deck
        outer_parts.append(_box(xa, xb, ARM_MID_Y0, ARM_TIP0, out_z0, out_z1))
        outer_parts.append(_box(xa, xb, ARM_MID_Y0, ARM_MID_Y0 + 14.0, z_t, out_z0))
        outer_parts.append(_box(xa, xb, ARM_TIP0 - 12.0, ARM_TIP0, z_t, out_z0))
        outer_parts.append(_box(xa, xb, TRAY_Y0, TRAY_Y0 + 6.0, z_t, zc_fs + 8.0))
        # Stage 1 GT2 belt side loop
        bx0 = (xb + 0.5) if sgn > 0 else (xa - 4.5)
        belts_list.append(_box(bx0, bx0 + 4.0, TRAY_Y0 - 6.0, ARM_TIP0, z_t + 10.0, z_t + 28.0))

        # Middle sliding bar + PTFE pads
        mid_parts.append(_box(xa, xb, ARM_MID_Y0, ARM_TIP0, mid_z0, mid_z1))
        mid_parts.append(_box(xa, xb, ARM_MID_Y0, ARM_TIP0, out_z1, mid_z0))

        # Inner extending bar + telescoping finger rod
        in_parts.append(_box(xa, xb, ARM_MID_Y0, ARM_TIP0, in_z0, in_z1))
        in_parts.append(_box(xa, xb, ARM_MID_Y0, ARM_TIP0, mid_z1, in_z0))
        xc_arm = (xa + xb) / 2.0
        in_parts.append(_box(xa, xb, ARM_TIP0 - 10.0, ARM_TIP0 - 4.0, in_z1, zc_fs + 7.0))
        in_parts.append(_box(xa, xb, ARM_MID_Y0, ARM_MID_Y0 + 6.0, in_z1, zc_fs + 7.0))
        in_parts.append(_box(xc_arm - 4.0, xc_arm + 4.0, ARM_MID_Y0 - 6.0, ARM_TIP0 + 4.0, zc_fs - 4.0, zc_fs + 4.0))

        for fy in (ARM_TIP0 - FINGER_T, ARM_TIP0 - REAR_OFF - FINGER_T):
            fingers_local[sgn].append(_box(-4.0, 4.0, fy, fy + FINGER_T, -4.0, FINGER_L))

    # Cross shaft + Servo 1 under printed tray front + Servo 2 on left front post
    outer_parts.append(_cyl_axis(4.0, TRAY_X0 - 4.0, TRAY_X1 + 4.0, "x", TRAY_Y0 - 6.0, z_t + 18.0))
    sx0 = BXR - 36.0
    servo_parts.append(_box(sx0, sx0 + 20.0, TRAY_Y0 - 26.0, TRAY_Y0 + 14.0, TRAY_RIB_Z0 - 34.0, TRAY_RIB_Z0))
    fx = BXR - ARM_DX_IN - ARM_W / 2.0 - 21.0
    servo_parts.append(_box(fx - 6.0, fx + 17.0, TRAY_Y0 - 26.0, TRAY_Y0 + 2.0, zc_fs - 6.0, zc_fs + 6.0))
    belts_list.append(_box(fx, BXR + ARM_DX_IN + ARM_W / 2.0, TRAY_Y0 + 7.0, TRAY_Y0 + 11.0, zc_fs - 7.0, zc_fs + 7.0))

    return {
        "tray_deck": tray,
        "tray_stops": tray_hw,
        "arms_outer": bd.Compound(children=outer_parts, label="Picker_Outer_Fixed_Stage"),
        "arms_mid": bd.Compound(children=mid_parts, label="Picker_Middle_Stage"),
        "arms_in": bd.Compound(children=in_parts, label="Picker_Inner_Stage"),
        "fingers_left": bd.Compound(children=fingers_local[-1], label="Picker_Left_Fingers"),
        "fingers_right": bd.Compound(children=fingers_local[1], label="Picker_Right_Fingers"),
        "picker_belts": bd.Compound(children=belts_list, label="Picker_GT2_Belts"),
        "picker_servos": bd.Compound(children=servo_parts, label="Picker_Servos_1_2"),
    }


def build_reference_env():
    """Reference compact prototype basket (140 x 130 x 80 mm) + matched Shelf 1 prototype rack bay."""
    basket = _box(BXR - BASKET_W / 2, BXR + BASKET_W / 2, BASKET_TRAY_Y0, BASKET_TRAY_Y0 + BASKET_D,
                  TRAY_TOP_Z + 1.0, TRAY_TOP_Z + 1.0 + BASKET_H)
    rack_solids = [
        _box(BXR - RACK_HALF_OPEN - 38.0, BXR - RACK_HALF_OPEN, 0.0, 38.0, 0.0, 500.0),
        _box(BXR + RACK_HALF_OPEN, BXR + RACK_HALF_OPEN + 38.0, 0.0, 38.0, 0.0, 500.0),
        _box(BXR - RACK_HALF_OPEN - 38.0, BXR + RACK_HALF_OPEN + 38.0, 0.0, 180.0, TRAY_TOP_Z - 3.0, TRAY_TOP_Z),
    ]
    return basket, bd.Compound(children=rack_solids, label="Ref_Prototype_Shelf1_Rack_Bay")


def check_body_clearances(parts: dict):
    """Verify 0 collisions across body attachment parts AND against H1/H2 pods & rails."""
    import build_option1_dual_legs as DL

    def overlap(a, b):
        ba, bb = a.bounding_box(), b.bounding_box()
        if (ba.min.X > bb.max.X or bb.min.X > ba.max.X or ba.min.Y > bb.max.Y or bb.min.Y > ba.max.Y
                or ba.min.Z > bb.max.Z or bb.min.Z > ba.max.Z):
            return 0.0
        inter = a.intersect(b)
        return sum(s.volume for s in inter.solids()) if inter is not None and hasattr(inter, "solids") else 0.0

    bad = []
    pairs = [
        ("body_collar", "column_stub"),
        ("body_collar", "anti_crush_plug"),
        ("body_collar", "tray_boom"),
        ("body_collar", "battery_pod"),
        ("tray_boom", "load_cell"),
        ("tray_boom", "live_wishbone"),
        ("tray_boom", "tray_deck"),
        ("load_cell", "live_wishbone"),
        ("load_cell", "tray_deck"),
        ("live_wishbone", "tray_deck"),
        ("battery_pod", "battery_pack"),
        ("battery_pod", "battery_lid"),
        ("battery_lid", "battery_pack"),
        ("tray_deck", "arms_outer"),
        ("arms_outer", "arms_mid"),
        ("arms_mid", "arms_in"),
        ("tray_deck", "picker_servos"),
    ]
    for ka, kb in pairs:
        v = overlap(parts[ka], parts[kb])
        if v > 1e-3:
            bad.append((ka, kb, v))

    pod_parts = O.build_assembled_parts()
    body_solid_keys = ("body_collar", "body_fasteners", "tray_boom", "live_wishbone", "load_cell", "tray_deck", "battery_pod", "battery_lid", "picker_servos")
    for tag, zc in (("H2_lower", JZ_LO), ("H1_upper", JZ_UP)):
        loc = DL.hub_world_loc(zc, 0.0, 0.0)
        for pk in ("body_mount", "servo_body", "carriage_pod", "gantry_plate", "motor_housing"):
            ps = DL.to_hub(pod_parts[pk], loc)
            for bk in body_solid_keys:
                v = overlap(parts[bk], ps)
                if v > 1e-3:
                    bad.append((bk, f"{tag}_{pk}", v))

    rail_zone = _box(-200.0, 500.0, -60.0, -40.0, 0.0, 500.0)
    for bk in body_solid_keys + ("arms_outer",):
        v = overlap(parts[bk], rail_zone)
        if v > 1e-3:
            bad.append((bk, "rail_zone_Y_-60..-40", v))
    for bk in body_solid_keys + ("arms_outer",):
        v = overlap(parts[bk], parts["ref_rack_bay"])
        if v > 1e-3:
            bad.append((bk, "ref_rack_bay", v))

    if bad:
        for item in bad:
            print("  COLLISION:", item)
    else:
        print("RESULT: OK - 0 collisions across B01/B02/B03/B04 printed body, load cell, picker, rails, rack, and H1/H2 pods")
    return len(bad) == 0


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    os.makedirs(SIM_MESH_DIR, exist_ok=True)

    col_stub, plug = build_column_stub_and_plug()
    collar, fast_col = build_body_column_collar()
    boom, live_wish, load_cell, lc_hw = build_tray_boom_and_loadcell()
    bat_box, bat_lid, bat_pack = build_battery_pod()
    picker = build_tray_and_picker()
    basket, rack = build_reference_env()
    for lbl, part in (("B01_collar", collar), ("B02_boom", boom), ("B03_bat", bat_box), ("B03L_lid", bat_lid), ("B04_tray", picker["tray_deck"]), ("B05_wishbone", live_wish)):
        assert len(part.solids()) == 1, f"{lbl} expected 1 solid, got {len(part.solids())}"
        assert part.bounding_box().min.Z >= 126.0, f"{lbl} stray below-body geometry Zmin={part.bounding_box().min.Z}"
    fast = bd.Compound(children=[*fast_col.solids(), *lc_hw.solids()], label="Body_And_LoadCell_Fasteners")

    export_map = {
        "column_stub": col_stub,
        "anti_crush_plug": plug,
        "body_collar": collar,
        "body_fasteners": fast,
        "tray_boom": boom,
        "live_wishbone": live_wish,
        "load_cell": load_cell,
        "battery_pod": bat_box,
        "battery_lid": bat_lid,
        "battery_pack": bat_pack,
        **picker,
        "ref_basket": basket,
        "ref_rack_bay": rack,
    }
    for k, shape in export_map.items():
        bd.export_stl(shape, os.path.join(SIM_MESH_DIR, f"{k}.stl"), tolerance=0.05, angular_tolerance=0.2)

    bd.export_stl(collar, os.path.join(OUT_DIR, "opt1_body_column_collar_B01.stl"), tolerance=0.01, angular_tolerance=0.1)
    bd.export_stl(boom, os.path.join(OUT_DIR, "opt1_body_tray_boom_B02.stl"), tolerance=0.01, angular_tolerance=0.1)
    bd.export_stl(bat_box, os.path.join(OUT_DIR, "opt1_body_battery_pod_B03.stl"), tolerance=0.01, angular_tolerance=0.1)
    bd.export_stl(bat_lid, os.path.join(OUT_DIR, "opt1_body_battery_lid_B03L.stl"), tolerance=0.01, angular_tolerance=0.1)
    bd.export_stl(picker["tray_deck"], os.path.join(OUT_DIR, "opt1_body_printed_tray_B04.stl"), tolerance=0.01, angular_tolerance=0.1)
    bd.export_stl(live_wish, os.path.join(OUT_DIR, "opt1_body_live_wishbone_B05.stl"), tolerance=0.01, angular_tolerance=0.1)
    bd.export_stl(plug, os.path.join(OUT_DIR, "opt1_body_anticrush_plug_P07B.stl"), tolerance=0.01, angular_tolerance=0.1)
    print(f"Exported {len(export_map)} body viewer meshes to {SIM_MESH_DIR}")

    step_path = os.path.join(OUT_DIR, "opt1_body_assembly.step")
    bd.export_step(bd.Compound(children=list(export_map.values()), label="Option1_Body_Assembly"), step_path)
    print(f"Exported STEP assembly -> {step_path}")

    ok = check_body_clearances(export_map)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
