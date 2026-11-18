"""Option 1 Upgrade: Dual 608ZZ Ball-Bearing Swivel + M8 Bolt + 1:2 Gear Reduction (50 kg*cm).

Solves every load/strength limitation of the direct-drive plastic collar:
  1. Structural Pivot (at X=0, Y=0 on MT0134):
       - Two 608ZZ steel ball bearings (8 x 22 x 7 mm, Rs. 85 on tronic.lk) 55 mm apart (straddle mount):
         bottom one in the Carriage Pod's tower (Z = 9..16, locating), top one in a hub on the pod's new top
         arm (Z = 64..71, floating) right under the Body Mount's top plate - i.e. right under the column.
       - Bottom: M8 x 40 bolt + 12 mm spacer + M8 nyloc in a hidden Z=3.5..8.8 mm pocket (no protrusion below
         the 3.0 mm MT0134 plate).  Top: M8 x 20 bolt from below into a nut in a side slot of the Body Mount boss.
       - Carries 100% of radial shear, axial pull-out, and cantilever tilting moments.
  2. 1:2 Torque-Doubling Gear Drive (MOD=1.25, PA=20 deg):
       - 40T Driven Gear (R_pitch=25.0 mm, Z=18.0..26.0) integrated onto the Carriage Pod hub.
       - 20T Driver Pinion (R_pitch=12.5 mm, Z=18.0..26.0) bolted with 4x M3 to the Ø20 mm CNC
         Aluminum 25T Servo Horn on the stationary TD-8125MG at X = -37.5 mm.
       - Full 180-deg TD-8125MG travel produces exactly 90-deg carriage swivel with ~50 kg*cm torque
         and zero structural weight on the servo shaft.
  3. Accurate T-Slot 2020 + MT0134 + Delrin V-Wheels + Sideways 4632-370 (100 RPM) Pod.
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import build123d as bd
import build_4632_370 as M
import build_4632_370_kit as K
import build_td8125mg as S
import build_openbuilds_gantry as G
import build_t_slot_swivel_carriage as T

OUT_DIR = os.path.abspath(os.path.join(HERE, "..", "models", "components", "t_slot_option1"))
SIM_MESH_DIR = os.path.abspath(os.path.join(HERE, "..", "sim", "build", "meshes_t_slot_option1"))

# --- 608ZZ Bearing & M8 Pivot constants (mm) ---
B608_OD = 22.0
B608_ID = 8.0
B608_W = 7.0
B_BOT_Z0 = 9.0                     # bottom 608ZZ: Z = 9.0 .. 16.0 in the carriage pod's tower (LOCATING: both races held)
TOWER_TOP_Z = 27.0                 # top of the bearing tower (unchanged; the 40T gear is on it)
BOLT_TOP_Z = 33.0                  # top of stationary Body Mount spindle cap
# Top bearing (straddle mount): the second 608ZZ sits 55 mm above the bottom one, in a hub on a new ARM of the carriage pod
# that reaches over from the motor sleeve to the swivel axis just under the body mount's top plate.  Its inner race is
# clamped to a boss on the underside of that plate (the column is bolted right above it) by an M8 x 20 bolt from below
# into a nut in a side slot of the boss.  The top bearing FLOATS (outer race in a light press fit, no axial stop above),
# so print tolerances along the axis cannot make the two bearings fight.  Bearing spread 11 -> 55 mm: ~5x lower bearing
# loads under the tipping moment, and the load goes almost straight into the plate under the column.
B_TOP_Z0 = 64.0                    # top 608ZZ:    Z = 64.0 .. 71.0
ARM_Z = (62.0, 71.5)               # the carriage pod's top arm (beam) height; its hub Z 61.0 .. 71.5, r 15
ARM_HUB_R = 15.0
ARM_END = (40.0, 26.0)             # the beam runs from the axis to the -X side of the motor sleeve (its closed half)
ARM_W = 12.0
BOSS_Z0 = 72.5                     # body mount boss under the top plate: Z 72.5 .. 78.5, r 11
BOSS_R = 11.0
TOP_NUT_Z = (75.0, 81.5)           # M8 hex nut (DIN 934) in the boss's side slot (slot opens toward +X under the plate)

# --- 1:2 Swivel Gear Drive (MOD = 1.25, C = 25.0 + 12.5 = 37.5 mm) ---
SW_MOD = 1.25
SW_PA = 20.0
GEAR_40T_Z = 40
GEAR_20T_Z = 20
SERVO_PIVOT_X = -37.5              # center distance = 12.5 + 25.0 = 37.5 mm on -X side
GEAR_Z0 = 18.0
GEAR_W = 8.0

HORN_Z0 = 26.4                     # 25T aluminum horn sits above 20T gear (Z = 26.4 .. 29.9)
HORN_H = 3.5
SPLINE_TIP_Z = 27.0
SPLINE_BASE_Z = SPLINE_TIP_Z + S.SPLINE_H  # 31.5 mm
SERVO_CAP_Z = SPLINE_BASE_Z + S.BOSS_H     # 33.0 mm
SERVO_TOP_Z = SERVO_CAP_Z + S.BOT_H + S.MID_H + S.TOP_BASE_H + S.RIDGE_H  # 73.5 mm
SERVO_X_OFF = SERVO_PIVOT_X - S.SHAFT_X    # -47.5 mm so 25T spline sits at X = -37.5 mm


def involute_gear_pts(z: int, m: float = SW_MOD, pa: float = SW_PA, backlash: float = 0.08):
    """Return 2D polygon points for an involute spur gear with tooth thinning for print clearance."""
    rp = m * z / 2.0
    rb = rp * math.cos(math.radians(pa))
    ra = rp + m * 0.95
    rf = rp - 1.25 * m
    inv = lambda a: math.tan(a) - a
    a0 = math.radians(pa)
    dth = backlash / rp

    def th(r):
        ar = math.acos(min(1.0, rb / r))
        return math.pi / (2.0 * z) - dth + inv(a0) - inv(ar)

    pts = []
    n_inv, n_arc = 8, 3
    radii = [rb + (ra - rb) * (i / n_inv) ** 1.3 for i in range(n_inv + 1)]
    for k in range(z):
        ang = 2.0 * math.pi * k / z
        for i in range(n_arc):
            t = ang - math.pi / z + (math.pi / z - th(rb)) * i / n_arc
            pts.append((rf * math.cos(t), rf * math.sin(t)))
        for r in radii:
            t = ang - th(r)
            pts.append((r * math.cos(t), r * math.sin(t)))
        for r in reversed(radii):
            t = ang + th(r)
            pts.append((r * math.cos(t), r * math.sin(t)))
        for i in range(1, n_arc):
            t = ang + th(rb) + (math.pi / z - th(rb)) * i / n_arc
            pts.append((rf * math.cos(t), rf * math.sin(t)))
    return pts


def build_608zz_bearing(z0: float) -> bd.Part:
    """Standard 608ZZ deep-groove ball bearing (8 x 22 x 7 mm)."""
    with bd.BuildPart() as br:
        with bd.Locations((0, 0, z0)):
            bd.Cylinder(radius=B608_OD / 2.0, height=B608_W, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
            bd.Cylinder(radius=B608_ID / 2.0, height=B608_W + 1.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
    part = br.part
    part.label = "Bearing_608ZZ"
    part.color = bd.Color(0.75, 0.77, 0.80)
    return part


def build_m8_bolt_assembly() -> bd.Compound:
    """Bottom: M8 x 40 bolt + 12 mm spacer (Z=16..28, up to the Body Mount cap) + washer + nyloc clamping the bottom 608ZZ's
    inner race.  Top: M8 x 20 bolt from below (hex head Z 56.7..62) + 2 mm spacer + top 608ZZ inner race + 1.5 mm spacer
    -> boss, nut in the boss's side slot."""
    with bd.BuildPart() as b:
        # Bolt head on top (Z = BOLT_TOP_Z .. BOLT_TOP_Z + 5.0) + Ø8.0 mm shank down to Z = 4.0
        with bd.Locations((0, 0, BOLT_TOP_Z)):
            bd.Cylinder(radius=6.8, height=5.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
        with bd.Locations((0, 0, 4.0)):
            bd.Cylinder(radius=3.95, height=BOLT_TOP_Z - 4.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
    bolt = b.part
    bolt.color = bd.Color(0.32, 0.34, 0.38)

    # Inner race spacer sleeve from the bottom 608ZZ (16.0) up to the Body Mount cap (28.0), OD 12, ID 8.2
    with bd.BuildPart() as sp:
        with bd.Locations((0, 0, B_BOT_Z0 + B608_W)):
            bd.Cylinder(radius=6.0, height=28.0 - (B_BOT_Z0 + B608_W), align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
        with bd.Locations((0, 0, 28.5)):
            bd.Hole(radius=4.05, depth=14.0)
        # Bottom washer + M8 nut clamping lower 608ZZ inner race (Z = 4.0 .. 8.8)
        with bd.Locations((0, 0, 7.5)):
            bd.Cylinder(radius=6.2, height=1.5, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
            bd.Hole(radius=4.05, depth=3.0)
        with bd.Locations((0, 0, 4.0)):
            bd.Cylinder(radius=6.8, height=3.5, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
            bd.Hole(radius=4.05, depth=5.0)
    sleeves = sp.part
    sleeves.color = bd.Color(0.65, 0.67, 0.70)

    # Top pivot: M8 x 20 hex bolt (DIN 933) from below, spacers, M8 hex nut (DIN 934) in the boss slot
    def ring(r, z0, z1, ri=4.05):
        c = bd.Cylinder(r, z1 - z0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN)).moved(bd.Location((0, 0, z0)))
        return c - bd.Cylinder(ri, z1 - z0 + 2, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN)).moved(bd.Location((0, 0, z0 - 1)))

    def hexa(r, z0, z1):
        with bd.BuildPart() as h:
            with bd.Locations((0, 0, z0)):
                with bd.BuildSketch():
                    bd.RegularPolygon(r, 6)
                bd.extrude(amount=z1 - z0)
        return h.part
    top_bolt = hexa(7.5, B_TOP_Z0 - 2.0 - 5.3, B_TOP_Z0 - 2.0) + bd.Cylinder(3.95, 20.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN)).moved(
        bd.Location((0, 0, B_TOP_Z0 - 2.0)))
    top_sp = ring(6.0, B_TOP_Z0 - 2.0, B_TOP_Z0) + ring(6.0, B_TOP_Z0 + B608_W, BOSS_Z0)
    top_nut = hexa(7.5, *TOP_NUT_Z) - bd.Cylinder(4.0, 10.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN)).moved(bd.Location((0, 0, TOP_NUT_Z[0] - 1)))
    return bd.Compound(children=[bolt, sleeves, top_bolt, top_sp, top_nut], label="M8_Pivot_Hardware")


def build_carriage_pod_option1() -> bd.Part:
    """Reinforced MT0134 Carriage Pod with Dual 608ZZ Bearing Tower + Integrated 40T Gear + Sideways 4632-370 Pod."""
    clr = 0.8
    wall = 2.4
    tower_od = 30.0
    nut_pocket_r = 9.0             # clears M8 nut & washer (R <= 6.8)

    with bd.BuildPart() as cp:
        # 1. Thick 5.5 mm MT0134 Mounting Deck (Z = 3.0 .. 8.5, clean stepped clearance around V-wheel bolts)
        with bd.Locations((5.5, 0, G.PLATE_T)):
            bd.Box(63.0, 52.0, 5.5, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
        for sx in (-20.0, 20.0):
            for sy in (-G.WHEEL_Y, G.WHEEL_Y):
                with bd.Locations((sx, sy, G.PLATE_T - 0.2)):
                    bd.Cylinder(radius=5.2, height=3.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
        for sx in (-10.0, 10.0):
            for sy in (-G.WHEEL_Y, G.WHEEL_Y):
                with bd.Locations((sx, sy, 9.0)):
                    bd.Hole(radius=2.6, depth=7.0)

        # 2. Central Dual-608ZZ Bearing Tower (Z = 3.0 .. 27.2, OD 30.0 mm)
        with bd.Locations((0, 0, G.PLATE_T)):
            bd.Cylinder(radius=tower_od / 2.0, height=TOWER_TOP_Z - G.PLATE_T, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))

        # 3. Integrated 40T Driven Gear (MOD=1.25, Z = 18.0 .. 26.0)
        g40_pts = involute_gear_pts(GEAR_40T_Z)
        with bd.Locations((0, 0, GEAR_Z0)):
            with bd.BuildSketch():
                bd.Polygon(*g40_pts, align=None)
            bd.extrude(amount=GEAR_W)

        # Subtract internal Dual-608ZZ bearing bores + retaining lip + M8 nut cavity:
        #   - Bottom M8 nut clearance cavity (Z = 2.8 .. 9.0, Ø18.0 mm)
        with bd.Locations((0, 0, G.PLATE_T - 0.2)):
            bd.Cylinder(radius=nut_pocket_r, height=B_BOT_Z0 - G.PLATE_T + 0.3, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
        #   - Lower 608ZZ seat (Z = 9.0 .. 16.0, Ø22.2 mm)
        with bd.Locations((0, 0, B_BOT_Z0 - 0.1)):
            bd.Cylinder(radius=B608_OD / 2.0 + 0.1, height=B608_W + 0.2, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
        #   - Middle outer-race stop shoulder (Z = 16.0 .. 20.0, Ø17.0 mm bore so outer races lock against it!)
        with bd.Locations((0, 0, B_BOT_Z0 + B608_W - 0.1)):
            bd.Cylinder(radius=8.5, height=4.2, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
        #   - Above the shoulder: clearance bore for the 12 mm spacer up to the tower top (the upper 608ZZ moved to the top arm)
        with bd.Locations((0, 0, B_BOT_Z0 + B608_W + 4.0 - 0.1)):
            bd.Cylinder(radius=8.5, height=TOWER_TOP_Z - (B_BOT_Z0 + B608_W + 4.0) + 0.6, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)

        # 4. 90-deg Sideways Cuboid Pod enclosing 4632-370 (same exact fit as T-slot carriage!)
        gb_x0 = T.GB_X_CENTER - M.GB_W / 2 - clr
        gb_x1 = T.GB_X_CENTER + M.GB_W / 2 + clr
        gb_y0 = T.GB_FACE_Y - 2.6
        gb_y1 = T.GB_FACE_Y + M.GB_H + clr + wall
        gb_z0 = T.GB_Z_CENTER - M.GB_L / 2 - clr - wall
        gb_z1 = T.GB_Z_CENTER + M.GB_L / 2 + clr + wall
        with bd.Locations(((gb_x0 + gb_x1) / 2, (gb_y0 + gb_y1) / 2, gb_z0)):
            bd.Box((gb_x1 - gb_x0) + 2 * wall, gb_y1 - gb_y0, gb_z1 - gb_z0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
        with bd.Locations((34.5, (gb_y0 + gb_y1) / 2, G.PLATE_T)):
            bd.Box(10.0, gb_y1 - gb_y0, 5.5, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))

        # Subtract 4632-370 cavity & exact pockets
        with bd.Locations(((gb_x0 + gb_x1) / 2, T.GB_FACE_Y, T.GB_Z_CENTER)):
            bd.Box(gb_x1 - gb_x0, M.GB_H + clr + 1.0, M.GB_L + 2 * clr, align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.CENTER), mode=bd.Mode.SUBTRACT)
        with bd.Locations(bd.Location((T.PINION_X, T.GB_FACE_Y + 0.2, T.PINION_Z), (90, 0, 0))):
            bd.Cylinder(radius=6.5, height=2.2, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
            bd.Hole(radius=4.0, depth=6.0)
        for sx in (-1, 1):
            for sz in (-1, 1):
                bx, bz = T.GB_X_CENTER + sx * 9.0, T.GB_Z_CENTER + sz * 16.5
                with bd.Locations(bd.Location((bx, T.GB_FACE_Y + 0.2, bz), (90, 0, 0))):
                    bd.Cylinder(radius=3.6, height=2.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
                    bd.Hole(radius=1.7, depth=6.0)
        with bd.Locations(bd.Location((T.GB_X_CENTER, T.GB_FACE_Y + 0.2, T.GB_Z_CENTER + 2.5), (90, 0, 0))):
            with bd.BuildSketch():
                bd.SlotOverall(16.0, 6.0, rotation=0)
            bd.extrude(amount=1.7, mode=bd.Mode.SUBTRACT)
        for sx, sz in ((-13.5, -20.5), (13.5, -20.5), (13.5, 20.5), (-13.5, 11.0)):
            with bd.Locations(bd.Location((T.GB_X_CENTER + sx, T.GB_FACE_Y + 0.2, T.GB_Z_CENTER + sz), (90, 0, 0))):
                bd.Cylinder(radius=1.8, height=1.4, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
        with bd.Locations(bd.Location((T.PINION_X, T.GB_FACE_Y + M.GB_H - 0.2, T.PINION_Z), (-90, 0, 0))):
            bd.Cylinder(radius=6.5, height=1.8, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)

        mc_y = T.GB_FACE_Y + M.GB_H / 2
        mc_r = M.MOTOR_D / 2 + clr
        with bd.Locations((T.GB_X_CENTER, mc_y, gb_z1 - 2.0)):
            bd.Cylinder(radius=mc_r + wall, height=M.MOTOR_L, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
        with bd.Locations((T.GB_X_CENTER, mc_y, gb_z1 - 5.0)):
            bd.Cylinder(radius=mc_r, height=M.MOTOR_L + 5.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
            bd.Box(2 * (mc_r + wall + 2.0), mc_r + wall + 2.0, M.MOTOR_L + 7.0, align=(bd.Align.CENTER, bd.Align.MAX, bd.Align.MIN), mode=bd.Mode.SUBTRACT)

        # 5. Top arm (straddle mount): hub on the swivel axis with the top 608ZZ seat, beam over to the motor sleeve's -X side
        ang = math.atan2(ARM_END[1], ARM_END[0])
        ln = math.hypot(*ARM_END)
        with bd.Locations(bd.Location((ARM_END[0] / 2, ARM_END[1] / 2, ARM_Z[0]), (0, 0, math.degrees(ang)))):
            bd.Box(ln, ARM_W, ARM_Z[1] - ARM_Z[0], align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
        with bd.Locations((0, 0, B_TOP_Z0 - 3.0)):
            bd.Cylinder(radius=ARM_HUB_R, height=ARM_Z[1] - (B_TOP_Z0 - 3.0), align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
        with bd.Locations((0, 0, B_TOP_Z0)):           # top 608ZZ seat (light press fit), open to the top: the bearing floats
            bd.Cylinder(radius=B608_OD / 2.0 + 0.05, height=ARM_Z[1] - B_TOP_Z0 + 1.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
        with bd.Locations((0, 0, B_TOP_Z0 - 4.0)):     # lip under the outer race; the bolt head + spacer pass inside it
            bd.Cylinder(radius=9.0, height=4.1, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
        # keep the motor can's bore clear of the beam
        with bd.Locations((T.GB_X_CENTER, T.GB_FACE_Y + M.GB_H / 2, ARM_Z[0] - 1.0)):
            bd.Cylinder(radius=M.MOTOR_D / 2 + clr, height=ARM_Z[1] - ARM_Z[0] + 2.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)

        # Outboard pillow-block arm on -Y
        with bd.Locations((T.PINION_X, 1.0, T.PINION_Z + 13.0)):
            bd.Box(12.0, 21.0, 4.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
        with bd.Locations((T.PINION_X, -7.8, T.PINION_Z - 5.0)):
            bd.Box(12.0, 3.6, 22.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
        with bd.Locations(bd.Location((T.PINION_X, -10.5, T.PINION_Z), (-90, 0, 0))):
            bd.Hole(radius=3.2, depth=6.0)

    part = cp.part
    part.label = "Option1_Carriage_Pod_40T"
    part.color = bd.Color(0.18, 0.55, 0.88)
    return part


def build_20t_servo_pinion() -> bd.Part:
    """20T MOD=1.25 Driver Gear (Z=18.0..26.0) at X = -37.5, bolted with 4x M3 to the 25T Aluminum Horn."""
    g20_pts = involute_gear_pts(GEAR_20T_Z)
    # Rotate by half-tooth (360 / (2*20) = 9 deg) so teeth mesh cleanly between 40T teeth at C = 37.5 mm!
    with bd.BuildPart() as g20:
        with bd.Locations(bd.Location((SERVO_PIVOT_X, 0, GEAR_Z0), (0, 0, 9.0))):
            with bd.BuildSketch():
                bd.Polygon(*g20_pts, align=None)
            bd.extrude(amount=GEAR_W)
        # Center M3 spline-screw access bore + 4x M3 clearance holes matching 25T horn (Ø14 mm PCD)
        with bd.Locations((SERVO_PIVOT_X, 0, GEAR_Z0 + GEAR_W + 0.5)):
            bd.Hole(radius=2.0, depth=GEAR_W + 1.0)
        for sx in (-1, 1):
            for sy in (-1, 1):
                with bd.Locations((SERVO_PIVOT_X + sx * 5.0, sy * 5.0, GEAR_Z0 + GEAR_W + 0.5)):
                    bd.Hole(radius=1.7, depth=GEAR_W + 1.0)

    part = g20.part
    part.label = "Option1_20T_Servo_Gear"
    part.color = bd.Color(0.95, 0.58, 0.16)
    return part


def build_25t_horn_offset() -> bd.Part:
    """Ø20 mm CNC Aluminum 25T Round Horn at X = -37.5, Z = 26.4..29.9."""
    with bd.BuildPart() as hn:
        with bd.Locations((SERVO_PIVOT_X, 0, HORN_Z0)):
            bd.Cylinder(radius=10.0, height=HORN_H, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
            bd.Cylinder(radius=S.SPLINE_D / 2 + 0.05, height=HORN_H + 0.5, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
        for sx in (-1, 1):
            for sy in (-1, 1):
                with bd.Locations((SERVO_PIVOT_X + sx * 5.0, sy * 5.0, HORN_Z0 + HORN_H + 0.5)):
                    bd.Hole(radius=1.35, depth=HORN_H + 1.0)
    part = hn.part
    part.label = "Option1_25T_Alum_Horn"
    part.color = bd.Color(0.85, 0.25, 0.25)
    return part


def build_body_mount_option1() -> bd.Part:
    """Stationary Robot Body Frame:
    - Rigid M8 spindle bridge at (0, 0, Z = 28.0 .. 33.0) clamping the Dual 608ZZ bearings
    - Full 4-wall cage at X = -37.5 holding TD-8125MG (above the 20T gear & 25T horn!)
    - Top Robot Column Clamp (4x M4 holes for 28 x 24 mm column)."""
    srv_l = S.BODY_L + 0.6     # 40.6
    srv_w = S.BODY_W + 0.6     # 21.1
    mount_l = S.EAR_SPAN + 4.0 # 58.5
    mount_w = srv_w + 6.4      # 27.5
    ear_z_bot = SERVO_TOP_Z - S.BOT_H - S.MID_H - S.EAR_T  # 43.5
    ear_z_top = ear_z_bot + S.EAR_T                        # 46.0
    top_plate_z1 = SERVO_TOP_Z + 5.0                       # 78.5

    with bd.BuildPart() as bm:
        # 1. Servo Cage (Z = 46.0 .. 78.5) centered at SERVO_X_OFF (-47.5)
        with bd.Locations((SERVO_X_OFF, 0, ear_z_top)):
            bd.Box(mount_l, mount_w, top_plate_z1 - ear_z_top, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))

        # 2. Structural Bridge Beam from Servo Cage (X=-37.5) over to M8 Bearing Spindle (X=0, Y=0) at Z = 28.0..33.0
        #    Wait: keep bridge ABOVE the 40T gear (Z=26.0) & 20T gear/horn (horn top = 29.9) -> put bridge at Z = 30.8..36.0?
        #    At X = 0 (over M8 bearing): cap is at Z = 28.0 .. 33.0 (clears top 608ZZ outer race by 1.0 mm via inner spacer!).
        #    And between X = -26 .. -12 (clear of 25T horn R=10 at X=-37.5), bridge slopes/steps up to ear_z_top (46.0)!
        with bd.Locations((0, 0, 28.0)):
            bd.Cylinder(radius=12.0, height=5.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
        # Structural truss arm connecting M8 spindle cap (0, 0, 28..33) to Servo Cage (-18, 0, 46..52)
        # Split into two outer side cheeks at Y = ±15.5 mm (completely outside 25T horn R=10 and 40T gear!)
        for sy in (-1, 1):
            with bd.Locations((-14.0, sy * 14.5, 28.0)):
                bd.Box(32.0, 4.0, 18.5, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
            with bd.Locations((0, sy * 10.0, 28.0)):
                bd.Box(16.0, 11.0, 5.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))

        # Top-bearing boss under the top plate (inner race of the top 608ZZ is clamped to its face, Z = BOSS_Z0)
        with bd.Locations((0, 0, BOSS_Z0)):
            bd.Cylinder(radius=BOSS_R, height=SERVO_TOP_Z + 5.0 - BOSS_Z0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))

        # Subtract M8 bolt bore (Ø8.2 mm) at (0, 0)
        with bd.Locations((0, 0, 34.0)):
            bd.Hole(radius=4.1, depth=8.0)

        # 3. Top Robot Column Clamp Plate spanning X = -76 .. +12 mm (Z = 78.5 .. 83.0)
        with bd.Locations((-32.0, 0, top_plate_z1)):
            bd.Box(88.0, 36.0, 4.5, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
        for sx in (-1, 1):
            for sy in (-1, 1):
                with bd.Locations((-32.0 + sx * 30.0, sy * 12.0, top_plate_z1 + 5.0)):
                    bd.Hole(radius=2.25, depth=6.0)
                # head pocket under the plate for the M4 x 12 socket cap (fitted from below, before the servo goes in)
                with bd.Locations((-32.0 + sx * 30.0, sy * 12.0, BOSS_Z0 - 0.5)):     # (open below the top-bearing boss too)
                    bd.Cylinder(3.8, top_plate_z1 - BOSS_Z0 + 0.6, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)

        # Top pivot: M8 bore through the boss + side slot for the M8 hex nut (slides in from +X, under the column)
        with bd.Locations((0, 0, BOSS_Z0 - 0.5)):
            bd.Cylinder(radius=4.1, height=TOP_NUT_Z[1] - BOSS_Z0 + 2.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
        with bd.Locations((0, 0, TOP_NUT_Z[0] - 0.1)):
            with bd.BuildSketch():
                bd.RegularPolygon(7.65, 6)
            bd.extrude(amount=TOP_NUT_Z[1] - TOP_NUT_Z[0] + 0.2, mode=bd.Mode.SUBTRACT)
        with bd.Locations((8.0, 0, TOP_NUT_Z[0] - 0.1)):
            bd.Box(16.0, 13.3, TOP_NUT_Z[1] - TOP_NUT_Z[0] + 0.2, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)

        # 4. Subtract flipped TD-8125MG servo pocket cleanly + wire-boot slot
        with bd.Locations((SERVO_X_OFF, 0, SERVO_CAP_Z - 0.3)):
            bd.Box(srv_l, srv_w, (SERVO_TOP_Z - SERVO_CAP_Z) + 0.8, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
        with bd.Locations((SERVO_X_OFF - S.BODY_L / 2 - 4.0, 0, SERVO_TOP_Z - 14.0)):
            bd.Box(12.0, 9.0, 14.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
        # Clear any overlap with Ø20 mm horn + 40T gear below Z = 32.0
        with bd.Locations((SERVO_PIVOT_X, 0, 25.0)):
            bd.Cylinder(radius=12.0, height=7.5, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)

        # 5. 4x M3 servo ear holes
        for sx in (-1, 1):
            for sy in (-1, 1):
                with bd.Locations((SERVO_X_OFF + sx * S.HOLE_PITCH_X / 2, sy * S.HOLE_PITCH_Y / 2, ear_z_top + 10.0)):
                    bd.Hole(radius=1.4, depth=12.0)

    part = bm.part
    part.label = "Option1_Body_Mount"
    part.color = bd.Color(0.20, 0.72, 0.45)
    return part


# --- Fasteners (all metric, ISO/DIN, stocked by Sri Lankan fastener shops; see FASTENERS) ---
DECK_SCREWS = [(sx, sy) for sx in (-10.0, 10.0) for sy in (-G.WHEEL_Y, G.WHEEL_Y)]        # carriage pod -> MT0134
DECK_TOP_Z = G.PLATE_T + 5.5                                                               # 8.5
MOTOR_SCREWS = [(T.GB_X_CENTER + sx * 9.0, T.GB_Z_CENTER + sz * 16.5) for sx in (-1, 1) for sz in (-1, 1)]   # (X, Z)
MOTOR_WALL_Y = T.GB_FACE_Y - 2.6                                                           # outer face of the pod wall (10.2)
EAR_SCREWS = [(SERVO_X_OFF + sx * S.HOLE_PITCH_X / 2, sy * S.HOLE_PITCH_Y / 2) for sx in (-1, 1) for sy in (-1, 1)]
EAR_BOT_Z = SERVO_TOP_Z - S.BOT_H - S.MID_H - S.EAR_T                                      # 43.5: underside of the ears
HORN_SCREWS = [(SERVO_PIVOT_X + sx * 5.0, sy * 5.0) for sx in (-1, 1) for sy in (-1, 1)]
COLUMN_SCREWS = [(-32.0 + sx * 30.0, sy * 12.0) for sx in (-1, 1) for sy in (-1, 1)]
TOP_PLATE_Z0 = SERVO_TOP_Z + 5.0                                                           # 78.5

FASTENERS = [   # (part, qty per pod, where, buy as)
    ("M5 x 12 button head (ISO 7380), A2 or 10.9 black", 4, "carriage pod deck -> MT0134 plate", "+ 4 x M5 nyloc (DIN 985) under the plate"),
    ("M3 x 6 button head (ISO 7380)", 4, "4632-370 gearbox -> pod wall (head 1.65 high: 1.25 mm clear of the pinion)", "into the gearbox's tapped bosses; the 0.8 mm wall skin is clamped onto the boss"),
    ("M3 x 14 pan head thread-forming (for plastics, 'PT' / self-tapping)", 4, "TD-8125MG ears -> body mount", "or M3 x 14 machine screw + M3 heat-set insert"),
    ("M3 x 12 socket cap (DIN 912)", 4, "20T gear -> 25T aluminium horn (horn holes are tapped M3)", ""),
    ("M3 x 8 servo spline screw", 1, "horn -> servo spline", "supplied with the servo"),
    ("M4 x 12 socket cap (DIN 912)", 4, "body mount -> column bar (tapped M4); fit before the servo", "+ 4 x M4 washer (DIN 125)"),
    ("M8 x 40 hex bolt (DIN 933, 8.8) + M8 nyloc (DIN 985) + M8 washer", 1, "bottom swivel bearing (in the tower)", "+ 12 x 8.2 x 12 steel spacer"),
    ("M8 x 20 hex bolt (DIN 933, 8.8) + M8 hex nut (DIN 934)", 1, "top swivel bearing (in the arm), nut in the boss slot", "+ 12 x 8.2 spacers 2 mm + 1.5 mm"),
    ("608ZZ (8 x 22 x 7)", 2, "swivel: one in the tower, one in the top arm (55 mm apart)", "tronic.lk / bearing shops (608-2RS for dust)"),
]


def _cz(r, z0, z1, x, y, seg=None):
    c = bd.Cylinder(r, z1 - z0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
    return c.moved(bd.Location((x, y, z0)))


def _hexz(r, z0, z1, x, y):
    with bd.BuildPart() as h:
        with bd.Locations((x, y, z0)):
            with bd.BuildSketch():
                bd.RegularPolygon(r, 6)
            bd.extrude(amount=z1 - z0)
    return h.part


def build_fasteners():
    """screws of the pod in the pod frame: turning with the carriage / on the body mount / turning with the 20T gear"""
    turn, fixed, gear = [], [], []
    for x, y in DECK_SCREWS:                     # button head on the deck, nyloc under the MT0134 plate
        turn += [_cz(4.75, DECK_TOP_Z, DECK_TOP_Z + 2.75, x, y), _cz(2.5, -3.5, DECK_TOP_Z, x, y), _hexz(4.6, -5.0, 0.0, x, y)]
    for x, z in MOTOR_SCREWS:                    # button head on the pod wall's outer face, into the gearbox bosses
        rot = bd.Location((x, MOTOR_WALL_Y, z), (-90, 0, 0))
        cyl = lambda r, h0, h1: bd.Cylinder(r, h1 - h0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN)).moved(bd.Location((0, 0, h0)))
        turn += [cyl(2.85, -1.65, 0.0).moved(rot), cyl(1.5, 0.0, 6.0).moved(rot)]
    for x, y in EAR_SCREWS:                      # pan head under the ears, up into the body mount
        fixed += [_cz(2.75, EAR_BOT_Z - 2.4, EAR_BOT_Z, x, y), _cz(1.5, EAR_BOT_Z, EAR_BOT_Z + 14.0, x, y)]
    for x, y in COLUMN_SCREWS:                   # socket cap from below, up into the tapped column bar
        fixed += [_cz(3.5, TOP_PLATE_Z0 - 4.0, TOP_PLATE_Z0, x, y), _cz(2.0, TOP_PLATE_Z0, TOP_PLATE_Z0 + 12.0, x, y)]
    for x, y in HORN_SCREWS:                     # socket cap under the 20T gear, up into the horn
        gear += [_cz(2.75, GEAR_Z0 - 3.0, GEAR_Z0, x, y), _cz(1.5, GEAR_Z0, GEAR_Z0 + 12.0, x, y)]
    gear += [_cz(2.75, GEAR_Z0 - 2.0, GEAR_Z0, SERVO_PIVOT_X, 0), _cz(1.5, GEAR_Z0, SPLINE_TIP_Z + 3.0, SERVO_PIVOT_X, 0)]
    out = {}
    for k, lst in (("fasteners_turn", turn), ("fasteners_fixed", fixed), ("fasteners_gear", gear)):
        out[k] = bd.Compound(children=lst, label=k)
    return out


def build_assembled_parts():
    """Return dict of all 14 bodies in exact gantry-centered coordinates (mm)."""
    base = T.build_assembled_parts()
    # Replace carriage_pod, alum_horn, body_mount, servo_body, servo_spline with Option 1 versions
    carriage_pod = build_carriage_pod_option1()
    bearing_bot = build_608zz_bearing(B_BOT_Z0)
    bearing_top = build_608zz_bearing(B_TOP_Z0)
    m8_hardware = build_m8_bolt_assembly()
    gear_20t = build_20t_servo_pinion()
    alum_horn = build_25t_horn_offset()
    body_mount = build_body_mount_option1()
    fasteners = build_fasteners()

    servo_flip = bd.Location((SERVO_X_OFF, 0, SERVO_TOP_Z), (180, 0, 0))
    servo_solids = []
    for sub in (S.build_case_bottom(), S.build_case_mid(), S.build_case_top()):
        servo_solids.extend(sub.solids() if isinstance(sub, bd.Compound) else [sub])
    servo_body = bd.Compound(children=[s.moved(servo_flip) for s in servo_solids], label="TD8125MG_Body")
    spline_local_z = S.BOT_H + S.MID_H + S.TOP_BASE_H + S.RIDGE_H + S.BOSS_H
    servo_spline = S.build_spline().moved(bd.Location((S.SHAFT_X, 0, spline_local_z))).moved(servo_flip)

    return {
        "gantry_plate": base["gantry_plate"],
        "v_wheels": base["v_wheels"],
        "motor_housing": base["motor_housing"],
        "motor_shaft": base["motor_shaft"],
        "pinion": base["pinion"],
        "retainer": base["retainer"],
        "carriage_pod": carriage_pod,
        "bearing_bot": bearing_bot,
        "bearing_top": bearing_top,
        "m8_hardware": m8_hardware,
        "gear_20t": gear_20t,
        "alum_horn": alum_horn,
        "servo_spline": servo_spline,
        "servo_body": servo_body,
        "body_mount": body_mount,
        **fasteners,
        "t_rail": base["t_rail"],
        "t_rack": base["t_rack"],
    }


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    os.makedirs(SIM_MESH_DIR, exist_ok=True)

    parts = build_assembled_parts()
    printable = {
        "opt1_carriage_pod_40t": parts["carriage_pod"],
        "opt1_body_mount": parts["body_mount"],
        "opt1_20t_servo_gear": parts["gear_20t"],
        "opt1_25t_horn": parts["alum_horn"],
        "opt1_pinion_14t": parts["pinion"],
        "opt1_shaft_retainer": parts["retainer"],
    }
    for name, part in printable.items():
        bd.export_step(part, os.path.join(OUT_DIR, f"{name}.step"))
        bd.export_stl(part, os.path.join(OUT_DIR, f"{name}.stl"), tolerance=0.01, angular_tolerance=0.1)
        bb = part.bounding_box()
        print(f"{name:24s} -> {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm")

    asm = bd.Compound(children=list(parts.values()), label="Option1_TSlot_Dual608ZZ_Assembly")
    asm_step = os.path.join(OUT_DIR, "opt1_complete_assembly.step")
    bd.export_step(asm, asm_step)
    print(f"Exported full STEP: {asm_step}")

    # Export split STLs centered on each local pivot for Three.js animation
    pinion_origin = bd.Location((-T.PINION_X, 0, -T.PINION_Z))
    servo_origin = bd.Location((-SERVO_PIVOT_X, 0, 0))
    for k, v in parts.items():
        if k in ("motor_shaft", "pinion", "retainer"):
            m = v.moved(pinion_origin)
        elif k in ("gear_20t", "alum_horn", "servo_spline", "fasteners_gear"):
            m = v.moved(servo_origin)
        else:
            m = v
        bd.export_stl(m, os.path.join(SIM_MESH_DIR, f"{k}.stl"), tolerance=0.01, angular_tolerance=0.1)
    print(f"Exported {len(parts)} rig meshes to {SIM_MESH_DIR}")


if __name__ == "__main__":
    main()
