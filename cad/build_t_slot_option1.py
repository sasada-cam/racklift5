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
ARM_END = (31.5, 16.0)             # the beam runs from the axis to the top of the ARM POST (on the pod, beside the gearbox)
ARM_W = 12.0
ARM_POST = (29.0, 35.3, 10.2, 22.0)  # X0, X1, Y0, Y1 of the post: deck (Z 3) -> arm (Z 71.5), part of the printed pod, so the
                                     # top bearing's load never goes through the motor cap's screws
# Motor housing in two parts (the gearbox was boxed in on all sides - it could not be fitted).  The split is the plane of
# the gearbox's mounting face: the face's bosses, shaft collar and ribs sit in pockets in the front wall, so the motor can
# only go on square to that wall (along -Y), never slide along it.
#   base (printed with the pod): front wall (the gearbox screws to it, 4 x M3 from the pinion side), -X side wall + arm
#     post, deck
#   MOTOR CAP (separate print): back wall, +X wall, bottom, lid and the can's sleeve - pulls straight off along +Y (the lid
#     and sleeve are already open toward the front wall around the can), held by 4 x M3 x 12 thread-forming screws along
#     -Y into bosses on the base: two on an ear beside the front wall (+X), two on columns above it
#   Fitting: push the 4632-370 square onto the front wall, screw it from the pinion side, slide the cap on, 4 screws.
CAP_Y0 = T.GB_FACE_Y                                # 12.8: the joint plane (gearbox mounting face)
CAP_X0 = 35.5                                       # the cap starts here in X (the -X wall + arm post stay on the base)
CAP_SCREWS = [(75.0, 6.0, T.GB_FACE_Y), (75.0, 38.0, T.GB_FACE_Y), (69.0, 50.0, T.GB_FACE_Y), (32.25, 50.0, 22.0)]
# (X, Z, joint Y) of the 4 screws (axis along Y): 2 on the ear beside the front wall, 1 on a column above it (all clear of
# the motor can's X 40.3..64.7 so the cap pulls off past it), 1 into the back face of the arm post
CAP_BOSS_R = 3.75
CAP_BASE_Y0 = 6.0                                   # base bosses / ear: Y 6.0 .. 12.8 (6.8 deep, pilot 10 deep)
CAP_TAB_Y1 = 16.0                                   # cap tabs: Y 13.0 .. 16.0
# Pinion cover (same idea): front plate (retainer-screw access hole), top and side walls around the climb pinion, open at
# the bottom where it meshes the rack; pulls off along -Y (away from the front wall) after 3 x M3 screws along +Y - one into
# the arm post, two into the ear beside the front wall.  Fitting: motor on -> pinion onto the shaft -> cover on.
PC_FRONT = (-9.6, -6.6)                             # front plate Y
PC_BACK_Y = 10.0                                    # cover stops 0.2 mm short of the front wall (Y 10.2)
PC_Z = (0.3, 22.3)                                  # bottom (above the rail top -2.5) .. top
PC_X = (37.2, 68.2)                                 # side walls' outer faces (pinion tip X 40.5 .. 64.5)
# Motor can CLAMP: the sleeve's open (front) half closed by a half-shell grown out of P01's front wall; both halves run up to Z 78 (can top 75.1,
# open at the top for the wires) and are bolted through flanges on both edges (split plane Y = can centre): 2 bolts on the
# +X edge (Z 58.5, 75.1) and 1 on the -X edge above the arm post (Z 75.1; the post fills the space below).  Undo the 3
# bolts + 4 cap screws, the cap comes off along +Y.
CLAMP_TOP_Z = 78.0                                  # below the body mount's top plate (78.5) the pods sweep past
CLAMP_BOLTS = [(71.4, 58.5), (71.4, 75.1), (33.6, 75.1)]   # (X, Z), axis along Y, M3 x 10 socket cap + M3 nut
CLAMP_FL_T = 3.0                                    # each flange 3 mm thick (Y)
PC_SCREWS = [(32.0, 15.0, ARM_POST[2]), (73.0, 16.0, CAP_BASE_Y0), (73.0, 28.0, CAP_BASE_Y0)]   # (X, Z, base face Y)
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


def _build_carriage_pod_full() -> bd.Part:
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

        # Rail clearance: below the rail top the motor box's front wall hung 0.2 mm beside the rail's side face (wall Y 10.2,
        # rail Y 10.0) - thin it to 1.8 mm there (Y >= 11.0: 1.0 mm clear).  The gearbox screws sit above Z -1.5.
        with bd.Locations((30.0, 9.0, -10.0)):
            bd.Box(45.0, 2.0, -2.0 + 10.0, align=(bd.Align.MIN, bd.Align.MIN, bd.Align.MIN), mode=bd.Mode.SUBTRACT)

        # 6. Arm post (deck -> arm)
        x0, x1, y0, y1 = ARM_POST
        with bd.Locations((x0, y0, G.PLATE_T)):
            bd.Box(x1 - x0, y1 - y0, ARM_Z[1] - G.PLATE_T, align=(bd.Align.MIN, bd.Align.MIN, bd.Align.MIN))

        # 7. Motor cap fixing - on the base side of the joint plane (Y <= 12.8): ear beside the front wall (+X) for the two
        #    lower screws, columns on top of the front wall for the two upper ones
        ear_x0 = gb_x1 + wall - 0.7
        with bd.Locations((ear_x0, CAP_BASE_Y0, CAP_SCREWS[0][1] - CAP_BOSS_R)):
            bd.Box(CAP_SCREWS[0][0] + CAP_BOSS_R - ear_x0, CAP_Y0 - CAP_BASE_Y0, CAP_SCREWS[1][1] - CAP_SCREWS[0][1] + 2 * CAP_BOSS_R,
                   align=(bd.Align.MIN, bd.Align.MIN, bd.Align.MIN))
        x, z, _ = CAP_SCREWS[2]
        with bd.Locations((x - CAP_BOSS_R, CAP_BASE_Y0, gb_z1 - 4.0)):
            bd.Box(2 * CAP_BOSS_R, CAP_Y0 - CAP_BASE_Y0, z + CAP_BOSS_R - (gb_z1 - 4.0), align=(bd.Align.MIN, bd.Align.MIN, bd.Align.MIN))

    part = cp.part
    return part


# Part IDs engraved on every printed part (0.6 mm deep = 3 layers at 0.2 mm), with the print batch.  Change BATCH per print run.
BATCH = "B01"
PART_IDS = {"carriage_pod": "P01", "motor_cap": "P02", "pinion_cover": "P03", "body_mount": "P04",
            "gear_20t": "P05", "pinion": "P06"}   # (can clamp front half = part of P01; the 10 x 10 x 2.5 retainer is too small to mark)
ENGRAVE_DEPTH = 0.6


def engrave(part, text, origin, x_dir, normal, size):
    """cut `text` into the face at `origin` (outward `normal`), reading along x_dir"""
    o = tuple(origin[i] + 0.05 * normal[i] for i in range(3))
    with bd.BuildSketch(bd.Plane(origin=o, x_dir=x_dir, z_dir=normal)) as sk:
        bd.Text(text, font_size=size, font_style=bd.FontStyle.BOLD, align=(bd.Align.CENTER, bd.Align.CENTER))
    return part - bd.extrude(sk.sketch, amount=-(ENGRAVE_DEPTH + 0.05))


def mark(key):
    return f"{PART_IDS[key]}-{BATCH}"


def _cap_region(gap=0.0):
    return bd.Box(60.0, 50.0, 100.0, align=(bd.Align.MIN, bd.Align.MIN, bd.Align.MIN)).moved(bd.Location((CAP_X0 + gap, CAP_Y0 + gap, -20.0)))


def _ycyl(r, y0, y1, x, z):
    """cylinder along +Y from y0 to y1 through (x, z)"""
    return bd.Cylinder(r, y1 - y0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN)).moved(bd.Location((x, y0, z), (-90, 0, 0)))


def build_carriage_pod_option1() -> bd.Part:
    """the printed carriage pod (base): everything except the motor cap, with pilot holes for the cap screws"""
    full = _build_carriage_pod_full()
    part = full - _cap_region()
    for x, z, yj in CAP_SCREWS:                                          # M3 thread-forming pilots, 10 deep
        part = part - _ycyl(1.3, yj - 10.0, yj + 0.5, x, z)
    for x, z, yf in PC_SCREWS:                                           # pinion cover screws: pilots into the post / ear
        part = part - _ycyl(1.3, yf - 0.5, min(yf + 9.0, CAP_Y0 - 0.3) if yf == CAP_BASE_Y0 else yf + 9.0, x, z)
    part = part + _clamp_front_half()
    part = engrave(part, mark("carriage_pod"), (ARM_POST[0], (ARM_POST[2] + ARM_POST[3]) / 2, 40.0), (0, 0, 1), (-1, 0, 0), 4.0)
    part.label = "Option1_Carriage_Pod_40T"
    part.color = bd.Color(0.18, 0.55, 0.88)
    return part


def build_pinion_cover_option1() -> bd.Part:
    """removable pinion cover (pulls off along -Y): front plate + top + side walls, open at the bottom, 3 screw tabs"""
    W = 2.4
    box = lambda x0, x1, y0, y1, z0, z1: bd.Box(x1 - x0, y1 - y0, z1 - z0, align=(bd.Align.MIN,) * 3).moved(bd.Location((x0, y0, z0)))
    x0, x1 = PC_X
    yb = PC_BACK_Y
    c = box(x0, x1, PC_FRONT[0], PC_FRONT[1], PC_Z[0], PC_Z[1])                    # front plate
    c = c + box(x0, x1, PC_FRONT[0], yb, PC_Z[1] - 3.0, PC_Z[1])                    # top
    c = c + box(x0, x0 + W, PC_FRONT[0], yb, G.PLATE_T + 5.7, PC_Z[1])              # -X wall (above the deck, Z 8.7)
    c = c + box(x0, x0 + W, PC_FRONT[0], -2.0, PC_Z[0], PC_Z[1])                    #   (and beside the pinion face below it)
    c = c + box(x1 - W - 0.6, x1, PC_FRONT[0], yb, PC_Z[0], PC_Z[1])                # +X wall
    # -X tab in front of the arm post's front face (Y 10.2); +X block reaching the ear's front face (Y 6.0)
    xs, zs, ys = PC_SCREWS[0]
    c = c + box(xs - CAP_BOSS_R, x0 + W, ys - 3.4, ys - 0.2, zs - CAP_BOSS_R, zs + CAP_BOSS_R)
    zlo, zhi = PC_SCREWS[1][1] - CAP_BOSS_R, PC_SCREWS[2][1] + CAP_BOSS_R
    c = c + box(x1 - W - 0.6, PC_SCREWS[1][0] + CAP_BOSS_R, PC_FRONT[0], CAP_BASE_Y0 - 0.2, zlo, zhi)
    # retainer-screw access hole in the front plate
    c = c - _ycyl(3.2, PC_FRONT[0] - 1.0, PC_FRONT[1] + 1.0, T.PINION_X, T.PINION_Z)
    # screw holes + counterbores (3 mm of flange under each head)
    for x, z, yf in PC_SCREWS:
        c = c - _ycyl(1.7, PC_FRONT[0] - 1.0, yf + 1.0, x, z)
        c = c - _ycyl(3.1, PC_FRONT[0] - 1.0, yf - 0.2 - 3.0, x, z)
    c = engrave(c, mark("pinion_cover"), (T.PINION_X, PC_FRONT[0], 16.0), (1, 0, 0), (0, -1, 0), 3.0)
    c.label = "Option1_Pinion_Cover"
    c.color = bd.Color(0.56, 0.82, 1.00)        # sky blue
    return c


def _clamp_geom():
    mc_x, mc_y = T.GB_X_CENTER, T.GB_FACE_Y + M.GB_H / 2
    r_in, r_out = M.MOTOR_D / 2 + 0.8, M.MOTOR_D / 2 + 0.8 + 2.4
    return mc_x, mc_y, r_in, r_out


def _clamp_flanges(y0, y1):
    """flange blocks (one per bolt) between Y y0..y1, reaching from the sleeve wall out past the bolt"""
    mc_x, mc_y, r_in, r_out = _clamp_geom()
    out = []
    for x, z in CLAMP_BOLTS:
        if x > mc_x:
            x0, x1 = mc_x + r_in + 0.5, x + 3.6
        else:
            x0, x1 = x - 3.6, mc_x - r_in - 0.5
        h = 5.8 if z > 70 else 6.6
        out.append(bd.Box(x1 - x0, y1 - y0, h, align=(bd.Align.MIN,) * 3).moved(bd.Location((x0, y0, min(z - h / 2, CLAMP_TOP_Z - h)))))
    return out


def _clamp_front_half() -> bd.Part:
    """front half of the motor can's sleeve (Y < can centre), Z 43.6 .. 78, flanges + 3 bolt holes - printed as part of P01
    (it grows out of the front wall's top; the can slides into it as the gearbox is pushed onto the wall).  Print P01 with
    tree supports under the shell's lower edge (it spans the open gearbox space)."""
    mc_x, mc_y, r_in, r_out = _clamp_geom()
    z0 = T.GB_Z_CENTER + M.GB_L / 2 + 0.8 + 0.2                    # 41.8: 1 mm above the gearbox top (its cavity ends 41.6)
    ring = bd.Cylinder(r_out, CLAMP_TOP_Z - z0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN)).moved(bd.Location((mc_x, mc_y, z0))) \
        - bd.Cylinder(r_in, CLAMP_TOP_Z - z0 + 2, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN)).moved(bd.Location((mc_x, mc_y, z0 - 1)))
    half = bd.Box(2 * r_out + 2, r_out + 1, CLAMP_TOP_Z - z0 + 2, align=(bd.Align.CENTER, bd.Align.MAX, bd.Align.MIN)).moved(
        bd.Location((mc_x, mc_y - 0.1, z0 - 1)))
    c = ring & half
    # bridge down into the front wall (its top is cut to Z 39 around the can): wall Y band, Z 38.5 .. 42.4, can bore kept
    gtop = T.GB_Z_CENTER + M.GB_L / 2                              # 40.8: gearbox top = bottom of the can
    bridge = bd.Box(2 * r_out, 2.6, 42.4 - 38.5, align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.MIN)).moved(
        bd.Location((mc_x, T.GB_FACE_Y - 2.6, 38.5)))
    bridge = bridge & bd.Cylinder(r_out, 10, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN)).moved(bd.Location((mc_x, mc_y, 35)))
    bridge = bridge - bd.Cylinder(r_in, 10, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN)).moved(bd.Location((mc_x, mc_y, gtop - 0.2)))
    c = c + bridge
    for f in _clamp_flanges(mc_y - 0.1 - CLAMP_FL_T, mc_y - 0.1):
        c = c + f
    for x, z in CLAMP_BOLTS:
        c = c - _ycyl(1.7, mc_y - 10.0, mc_y + 1.0, x, z)
    # notches where the cap's two upper screw tabs / ribs (Z 46.25 .. 53.75) pass beside the can, so the clamp slides off -Y
    zt0, zt1 = CAP_SCREWS[2][1] - CAP_BOSS_R - 0.4, CAP_SCREWS[2][1] + CAP_BOSS_R + 0.4
    c = c - bd.Box(20, 40, zt1 - z0 + 1.0, align=(bd.Align.MIN,) * 3).moved(bd.Location((65.0, mc_y - 30, z0 - 1.0)))   # (+ the base's column)
    c = c - bd.Box(20, 40, zt1 - zt0, align=(bd.Align.MAX, bd.Align.MIN, bd.Align.MIN)).moved(bd.Location((mc_x - r_in + 0.2, mc_y - 30, zt0)))
    return c


def build_motor_cap_option1() -> bd.Part:
    """removable motor cap (pulls off along +Y): back / +X / bottom walls, lid, can sleeve + 4 screw tabs (0.2 mm off the base)"""
    full = _build_carriage_pod_full()
    cy0 = CAP_Y0 + 0.2
    cap = full & _cap_region(0.2)
    # lower tabs: a plate on the +X wall over the base's ear
    x0 = T.GB_X_CENTER + M.GB_W / 2 + 0.8
    cap = cap + bd.Box(CAP_SCREWS[0][0] + CAP_BOSS_R - x0, CAP_TAB_Y1 - cy0, CAP_SCREWS[1][1] - CAP_SCREWS[0][1] + 2 * CAP_BOSS_R,
                       align=(bd.Align.MIN, bd.Align.MIN, bd.Align.MIN)).moved(bd.Location((x0, cy0, CAP_SCREWS[0][1] - CAP_BOSS_R)))
    # upper right tab: disc + rib back to the sleeve's closed half (the motor can's bore, r 13, kept clear)
    mc_x, mc_y, bore = T.GB_X_CENTER, T.GB_FACE_Y + M.GB_H / 2, M.MOTOR_D / 2 + 0.8
    x, z, _ = CAP_SCREWS[2]
    cap = cap + _ycyl(CAP_BOSS_R, cy0, CAP_TAB_Y1, x, z)
    cap = cap + bd.Box(x + CAP_BOSS_R - (mc_x + bore), mc_y + 2.0 - cy0, 2 * CAP_BOSS_R, align=(bd.Align.MIN, bd.Align.MIN, bd.Align.MIN)).moved(
        bd.Location((mc_x + bore, cy0, z - CAP_BOSS_R)))
    # upper left tab: plate on the arm post's back face (Y 22), joined to the sleeve's -X side
    x, z, yj = CAP_SCREWS[3]
    px0 = ARM_POST[0] + 0.5
    cap = cap + bd.Box(mc_x - bore - px0, 3.2, 2 * CAP_BOSS_R, align=(bd.Align.MIN, bd.Align.MIN, bd.Align.MIN)).moved(
        bd.Location((px0, yj + 0.2, z - CAP_BOSS_R)))
    for x, z, yj in CAP_SCREWS:
        y0 = cy0 if yj == CAP_Y0 else yj + 0.2
        cap = cap - _ycyl(1.7, y0 - 1.0, y0 + 3.0 + 1.0 + (CAP_TAB_Y1 - cy0 - 3.0 if yj == CAP_Y0 else 0.2), x, z)
    x, z, _ = CAP_SCREWS[2]                                              # head pocket through the right rib
    cap = cap - _ycyl(3.1, CAP_TAB_Y1, CAP_TAB_Y1 + 20.0, x, z)
    # can clamp: the sleeve's back half runs on up to CLAMP_TOP_Z, flanges + bolt holes on both edges
    cmx, cmy, r_in, r_out = _clamp_geom()
    z1 = T.GB_Z_CENTER + M.GB_L / 2 + 0.8 + 2.4 - 2.0 + M.MOTOR_L            # top of the original sleeve (72.8)
    ext = bd.Cylinder(r_out, CLAMP_TOP_Z - z1 + 0.5, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN)).moved(bd.Location((cmx, cmy, z1 - 0.5))) \
        - bd.Cylinder(r_in, CLAMP_TOP_Z - z1 + 3, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN)).moved(bd.Location((cmx, cmy, z1 - 1.5)))
    ext = ext & bd.Box(2 * r_out + 2, r_out + 1, 20, align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.MIN)).moved(bd.Location((cmx, cmy, z1 - 2)))
    cap = cap + ext
    for f in _clamp_flanges(cmy, cmy + CLAMP_FL_T):
        cap = cap + f
    for x, z in CLAMP_BOLTS:
        cap = cap - _ycyl(1.7, cmy - 1.0, cmy + CLAMP_FL_T + 1.0, x, z)
    gb_y1 = T.GB_FACE_Y + M.GB_H + 0.8 + 2.4
    cap = engrave(cap, mark("motor_cap"), (T.GB_X_CENTER, gb_y1, 17.0), (-1, 0, 0), (0, 1, 0), 5.0)
    if isinstance(cap, bd.Compound) or len(cap.solids()) > 1:
        cap = max(cap.solids(), key=lambda q: q.volume) if len(cap.solids()) > 1 else cap
    cap.label = "Option1_Motor_Cap"
    cap.color = bd.Color(0.11, 0.29, 0.60)      # navy
    return cap


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
    part = engrave(part, PART_IDS["gear_20t"], (SERVO_PIVOT_X + 9.8, 0.0, GEAR_Z0), (0, 1, 0), (0, 0, -1), 1.6)
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
    part = engrave(part, mark("body_mount"), (SERVO_X_OFF, mount_w / 2, 62.0), (-1, 0, 0), (0, 1, 0), 5.0)
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
    ("M3 x 12 pan head thread-forming (for plastics, 'PT' / self-tapping)", 5, "motor cap -> carriage pod (4) + pinion cover -> arm post (1)", "or M3 x 12 + M3 heat-set insert"),
    ("M3 x 10 pan head thread-forming (for plastics, 'PT' / self-tapping)", 2, "pinion cover -> ear beside the front wall", ""),
    ("M3 x 10 socket cap (DIN 912) + M3 hex nut (DIN 934, or nyloc DIN 985)", 3, "motor can clamp: P01's front half-shell <-> the cap's half (flanges)", ""),
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
    for x, z, yf in PC_SCREWS:                   # pinion cover -> base: pan head in the cover's counterbore, along +Y
        yh = yf - 0.2 - 3.0
        turn += [_ycyl(2.75, yh - 2.4, yh, x, z), _ycyl(1.5, yh, yh + (10.0 if yf == CAP_BASE_Y0 else 12.0), x, z)]
    cmx, cmy, _, _ = _clamp_geom()
    for x, z in CLAMP_BOLTS:                     # can clamp: M3 x 10 socket cap (head on the cap's flange) + M3 nut (front)
        turn += [_ycyl(2.75, cmy + CLAMP_FL_T, cmy + CLAMP_FL_T + 3.0, x, z), _ycyl(1.5, cmy - 0.1 - CLAMP_FL_T - 2.4, cmy + CLAMP_FL_T, x, z),
                 _ycyl(3.1, cmy - 0.1 - CLAMP_FL_T - 2.4, cmy - 0.1 - CLAMP_FL_T, x, z)]
    for x, z, yj in CAP_SCREWS:                  # motor cap -> base: pan head on the cap's tab, along -Y into the base's boss
        yh = CAP_TAB_Y1 if yj == CAP_Y0 else yj + 3.4
        turn += [_ycyl(2.75, yh, yh + 2.4, x, z), _ycyl(1.5, yh - 12.0, yh, x, z)]
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
    motor_cap = build_motor_cap_option1()
    pinion_cover = build_pinion_cover_option1()
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
        "pinion": engrave(base["pinion"], PART_IDS["pinion"], (T.PINION_X, base["pinion"].bounding_box().min.Y, T.PINION_Z + 6.5),
                          (1, 0, 0), (0, -1, 0), 2.0),
        "retainer": base["retainer"],
        "carriage_pod": carriage_pod,
        "motor_cap": motor_cap,
        "pinion_cover": pinion_cover,
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
        "opt1_motor_cap": parts["motor_cap"],
        "opt1_pinion_cover": parts["pinion_cover"],
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
