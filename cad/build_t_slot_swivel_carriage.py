"""Accurate T-Slot Rail-Drive + 90-deg Swivel Carriage Assembly.

Combines ALL real bought hardware with exact dimensions:
  - OpenBuilds MT0134 Gantry Plate (65.50 x 65.50 x 3.0 mm) + 4x Delrin V-Wheels (Ø24 x 10.95)
    + 6 mm spacers + 1 mm shims + M5 hardware
  - CNC4551 2020 T-Slot Aluminium Extrusion (20 x 20 mm, R1.5, 6.2 mm slot) + In-Slot Rack
  - 4632-370 Worm Gear Motor (100 RPM) mounted 90 deg sideways in a cuboid side pod on the +X edge,
    driving the 14T Climber Pinion (5 mm face) straight inside the 6.2 mm T-slot, double-supported
    by an outboard pillow-block arm on the -Y side
  - Stationary TD-8125MG 25KG Digital Servo + Body Column Mount at (0, 0) with:
      * Reinforced 4-bolt Ø20 mm CNC Aluminum 25T Round Servo Horn (4x M3 steel bolts)
      * Unbroken Ø26.0 mm Bearing Collar completely BELOW the red servo housing (zero clash!)

Coordinate frame (Gantry-centered):
  - X = along rail when at 0 deg; Y = across rail; Z = normal to gantry plate
  - Z = 0.0 .. +3.0 mm: MT0134 Gantry Plate
  - Z = -22.475 .. -2.475 mm: 2020 T-Slot Rail centered at Z = -12.475 mm
  - Z = +5.325 mm, X = +49.0 mm, Y = 0: 14T Climber Pinion axis (along Y)
  - X = 0, Y = 0: Swivel axis (along Z) for 0..90 deg rail switching
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

OUT_DIR = os.path.abspath(os.path.join(HERE, "..", "models", "components", "t_slot_carriage"))
SIM_MESH_DIR = os.path.abspath(os.path.join(HERE, "..", "sim", "build", "meshes_t_slot_carriage"))

# --- Key assembly coordinates (mm) ---
PINION_X = 52.5                    # 19.75 mm clear past +32.75 MT0134 plate edge & +32.0 V-wheels
PINION_Z = 5.325                   # RK_PITCH_Z (-5.175) + PIN_R (10.5) = +5.325 mm
GB_FACE_Y = 12.8                   # 4632-370 gearbox shaft-side face at Y = +12.8 mm (front wall 10.4..12.8 > 10.0 rail!)
GB_X_CENTER = PINION_X             # 52.5 mm -> gearbox spans X = 36.5..68.5 mm (pod outer X = 33.3..71.7 mm)
GB_Z_CENTER = PINION_Z + M.SHAFT_OFFSET_X  # 5.325 + 12.5 = 17.825 mm -> gearbox Z = -5.175..+40.825

# Swivel center at (0, 0) on top of MT0134 plate (Z = +3.0 mm)
BASE_DECK_Z0 = G.PLATE_T           # 3.0 mm
BASE_DECK_Z1 = 7.5                 # 4.5 mm thick mounting deck on MT0134
TURN_TOP_Z = 15.5                  # top of turntable bearing skirt on Carriage Pod
HORN_Z0 = 9.0                      # 25T aluminum round horn at Z = 9.0..12.5 mm
HORN_H = 3.5

COLLAR_D = 26.0
COLLAR_Z0 = 12.8                   # 0.3 mm above horn (12.5), 2.7 mm overlap inside skirt (15.5)
COLLAR_Z1 = 17.2                   # unbroken collar BELOW servo housing!

SPLINE_TIP_Z = 11.8                # spline engages inside horn (11.8..16.3)
SPLINE_BASE_Z = SPLINE_TIP_Z + S.SPLINE_H  # 16.3 mm
SERVO_CAP_Z = SPLINE_BASE_Z + S.BOSS_H     # 17.8 mm (above collar & bridge!)
SERVO_TOP_Z = SERVO_CAP_Z + S.BOT_H + S.MID_H + S.TOP_BASE_H + S.RIDGE_H  # 58.3 mm
SERVO_X_OFF = -S.SHAFT_X           # -10.0 mm so spline sits at X = 0, Y = 0!


def build_carriage_pod_housing() -> bd.Part:
    """90-deg Sideways Cuboid Pod + MT0134 Adapter Deck + Reinforced Turntable + Outboard Pinion Support.
    Bolts directly to MT0134's existing Ø5.10 mm holes (no drilling required on the aluminum plate!)."""
    clr = 0.8
    wall = 2.4
    sk_id = COLLAR_D + 0.6         # 26.6 mm
    sk_od = sk_id + 6.4            # 33.0 mm

    with bd.BuildPart() as cp:
        # 1. MT0134 Mounting Deck (Z = 3.0 .. 7.5, spans X = -25 .. +36, Y = -25 .. +25)
        with bd.Locations((5.5, 0, BASE_DECK_Z0)):
            bd.Box(61.0, 50.0, BASE_DECK_Z1 - BASE_DECK_Z0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))

        # Clearance pockets at the 4 V-wheel M5 screw heads (±20.0, ±19.85, head Ø8.4 x 2.2)
        for sx in (-20.0, 20.0):
            for sy in (-G.WHEEL_Y, G.WHEEL_Y):
                with bd.Locations((sx, sy, BASE_DECK_Z0 - 0.2)):
                    bd.Cylinder(radius=5.2, height=3.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)

        # 4x M5 mounting bolt holes matching MT0134 existing Ø5.10 holes at (±10.0, ±19.85)
        for sx in (-10.0, 10.0):
            for sy in (-G.WHEEL_Y, G.WHEEL_Y):
                with bd.Locations((sx, sy, BASE_DECK_Z1 + 0.5)):
                    bd.Hole(radius=2.6, depth=BASE_DECK_Z1 - BASE_DECK_Z0 + 1.0)

        # 2. Turntable Flange at (0, 0, Z = 7.5 .. 15.5) with 4-bolt Ø20.4 mm horn pocket
        with bd.Locations((0, 0, BASE_DECK_Z1)):
            bd.Cylinder(radius=sk_od / 2, height=TURN_TOP_Z - BASE_DECK_Z1, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
        with bd.Locations((0, 0, HORN_Z0 - 0.1)):
            bd.Cylinder(radius=10.3, height=TURN_TOP_Z - HORN_Z0 + 1.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
        with bd.Locations((0, 0, COLLAR_Z0 - 0.1)):
            bd.Cylinder(radius=sk_id / 2, height=TURN_TOP_Z - COLLAR_Z0 + 1.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
        # Raised central boss pad (Z = 7.5 .. 8.9) under aluminum horn + 4x M3 bolts + center access
        with bd.Locations((0, 0, BASE_DECK_Z1)):
            bd.Cylinder(radius=10.2, height=HORN_Z0 - BASE_DECK_Z1, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
        with bd.Locations((0, 0, HORN_Z0 + 0.5)):
            bd.Hole(radius=2.0, depth=7.0)
        for sx in (-1, 1):
            for sy in (-1, 1):
                with bd.Locations((sx * 5.0, sy * 5.0, HORN_Z0 + 0.5)):
                    bd.Hole(radius=1.7, depth=7.0)

        # 3. 90-deg Sideways Cuboid Pod enclosing the 4632-370 gearbox (X = 33..65, Y = 8.5..31.0, Z = -5.175..40.825)
        gb_x0 = GB_X_CENTER - M.GB_W / 2 - clr
        gb_x1 = GB_X_CENTER + M.GB_W / 2 + clr
        gb_y0 = GB_FACE_Y - 2.6                 # 2.6 mm front mounting wall in front of gearbox face
        gb_y1 = GB_FACE_Y + M.GB_H + clr + wall
        gb_z0 = GB_Z_CENTER - M.GB_L / 2 - clr - wall
        gb_z1 = GB_Z_CENTER + M.GB_L / 2 + clr + wall
        with bd.Locations(((gb_x0 + gb_x1) / 2, (gb_y0 + gb_y1) / 2, gb_z0)):
            bd.Box(
                (gb_x1 - gb_x0) + 2 * wall,
                gb_y1 - gb_y0,
                gb_z1 - gb_z0,
                align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN),
            )
        # Connect Mounting Deck smoothly to Sideways Cuboid Pod (Z = 3.0..7.5 only, clears V-wheels & rail)
        with bd.Locations((34.5, (gb_y0 + gb_y1) / 2, BASE_DECK_Z0)):
            bd.Box(8.0, gb_y1 - gb_y0, BASE_DECK_Z1 - BASE_DECK_Z0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))

        # Subtract inner 4632-370 gearbox cavity + shaft/boss/bump pockets + 4x M3 gearbox mounting holes
        with bd.Locations(((gb_x0 + gb_x1) / 2, GB_FACE_Y, GB_Z_CENTER)):
            bd.Box(
                gb_x1 - gb_x0,
                M.GB_H + clr + 1.0,
                M.GB_L + 2 * clr,
                align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.CENTER),
                mode=bd.Mode.SUBTRACT,
            )
        # Front wall shaft-boss pocket (Ø13 x 2.0 mm) + Ø8.0 mm shaft pass-through
        with bd.Locations(bd.Location((PINION_X, GB_FACE_Y + 0.2, PINION_Z), (90, 0, 0))):
            bd.Cylinder(radius=6.5, height=2.2, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
            bd.Hole(radius=4.0, depth=6.0)
        # Front wall pockets for 4x M3 bosses (Ø7.2 x 1.8 mm) + Ø3.4 mm through-holes
        for sx in (-1, 1):
            for sz in (-1, 1):
                bx, bz = GB_X_CENTER + sx * 9.0, GB_Z_CENTER + sz * 16.5
                with bd.Locations(bd.Location((bx, GB_FACE_Y + 0.2, bz), (90, 0, 0))):
                    bd.Cylinder(radius=3.6, height=2.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
                    bd.Hole(radius=1.7, depth=6.0)
        # Front wall pocket for center stadium boss (16.0 x 6.0 x 1.5 mm)
        with bd.Locations(bd.Location((GB_X_CENTER, GB_FACE_Y + 0.2, GB_Z_CENTER + 2.5), (90, 0, 0))):
            with bd.BuildSketch():
                bd.SlotOverall(16.0, 6.0, rotation=0)
            bd.extrude(amount=1.7, mode=bd.Mode.SUBTRACT)
        # Front wall pockets for 4x corner cover screws (Ø3.6 x 1.2 mm)
        for sx, sz in ((-13.5, -20.5), (13.5, -20.5), (13.5, 20.5), (-13.5, 11.0)):
            with bd.Locations(bd.Location((GB_X_CENTER + sx, GB_FACE_Y + 0.2, GB_Z_CENTER + sz), (90, 0, 0))):
                bd.Cylinder(radius=1.8, height=1.4, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
        # Rear wall pocket for gearbox circular back bump (Ø13.0 x 1.6 mm at Y = GB_FACE_Y + GB_H)
        with bd.Locations(bd.Location((PINION_X, GB_FACE_Y + M.GB_H - 0.2, PINION_Z), (-90, 0, 0))):
            bd.Cylinder(radius=6.5, height=1.8, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)

        # Upper motor-can semicircle shroud (open through top wall starting at gb_z1 - 5.0)
        mc_y = GB_FACE_Y + M.GB_H / 2
        mc_r = M.MOTOR_D / 2 + clr
        with bd.Locations((GB_X_CENTER, mc_y, gb_z1 - 2.0)):
            bd.Cylinder(radius=mc_r + wall, height=M.MOTOR_L, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
        with bd.Locations((GB_X_CENTER, mc_y, gb_z1 - 5.0)):
            bd.Cylinder(radius=mc_r, height=M.MOTOR_L + 5.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
        # Open front half of motor shroud (-Y half so motor slides in cleanly!)
        with bd.Locations((GB_X_CENTER, mc_y, gb_z1 - 5.0)):
            bd.Box(2 * (mc_r + wall + 2.0), mc_r + wall + 2.0, M.MOTOR_L + 7.0,
                   align=(bd.Align.CENTER, bd.Align.MAX, bd.Align.MIN), mode=bd.Mode.SUBTRACT)

        # 4. Outboard Pinion Pillow-Block Bridge (bridges over top of pinion Z=18.5 down to Y=-9.0)
        #    Supports the far end of the Ø6 mm shaft via an M3 bolt into the shaft's axial bore!
        with bd.Locations((PINION_X, 1.0, PINION_Z + 13.0)):
            bd.Box(12.0, 21.0, 4.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
        with bd.Locations((PINION_X, -7.8, PINION_Z - 5.0)):
            bd.Box(12.0, 3.6, 22.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
        with bd.Locations(bd.Location((PINION_X, -10.5, PINION_Z), (-90, 0, 0))):
            bd.Hole(radius=3.2, depth=6.0)

    part = cp.part
    part.label = "TSlot_Carriage_Pod"
    part.color = bd.Color(0.18, 0.55, 0.88)
    return part


def build_servo_horn_25t() -> bd.Part:
    """Standard Ø20 mm CNC aluminum 25T round servo horn (bolted with 4x M3 to the turntable)."""
    with bd.BuildPart() as hn:
        with bd.Locations((0, 0, HORN_Z0)):
            bd.Cylinder(radius=10.0, height=HORN_H, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
            bd.Cylinder(radius=S.SPLINE_D / 2 + 0.05, height=HORN_H + 0.5, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
        for sx in (-1, 1):
            for sy in (-1, 1):
                with bd.Locations((sx * 5.0, sy * 5.0, HORN_Z0 + HORN_H + 0.5)):
                    bd.Hole(radius=1.35, depth=HORN_H + 1.0)
    part = hn.part
    part.label = "TSlot_25T_Alum_Horn"
    part.color = bd.Color(0.85, 0.25, 0.25)
    return part


def build_body_servo_mount() -> bd.Part:
    """Stationary Robot Body / Column Mount at (0, 0): full 4-wall cage holding TD-8125MG upside-down,
    outer bridges at Y=±19.2 mm down to unbroken Ø26.0 mm bearing collar (Z=12.8..17.2)."""
    srv_l = S.BODY_L + 0.6     # 40.6
    srv_w = S.BODY_W + 0.6     # 21.1
    mount_l = S.EAR_SPAN + 4.0 # 58.5
    mount_w = srv_w + 6.4      # 27.5
    ear_z_bot = SERVO_TOP_Z - S.BOT_H - S.MID_H - S.EAR_T  # 28.3
    ear_z_top = ear_z_bot + S.EAR_T                        # 30.8
    top_plate_z1 = SERVO_TOP_Z + 5.0                       # 63.3

    with bd.BuildPart() as bm:
        # 1. Full 4-wall servo cage (Z = ear_z_top .. top_plate_z1)
        with bd.Locations((SERVO_X_OFF, 0, ear_z_top)):
            bd.Box(mount_l, mount_w, top_plate_z1 - ear_z_top, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))

        # 2. Top Robot Column Flange (4x M4 holes for 28 x 24 mm robot column)
        with bd.Locations((SERVO_X_OFF, 0, top_plate_z1)):
            bd.Box(mount_l, 36.0, 4.5, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
        for sx in (-1, 1):
            for sy in (-1, 1):
                with bd.Locations((SERVO_X_OFF + sx * 20.0, sy * 12.0, top_plate_z1 + 5.0)):
                    bd.Hole(radius=2.25, depth=6.0)

        # 3. Unbroken Concentric Ø26.0 mm Bearing Collar Ring (Z = 12.8 .. 17.2, BELOW servo cap 17.8!)
        with bd.Locations((0, 0, COLLAR_Z0)):
            bd.Cylinder(radius=COLLAR_D / 2, height=COLLAR_Z1 - COLLAR_Z0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
            bd.Cylinder(radius=7.0, height=COLLAR_Z1 - COLLAR_Z0 + 1.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)

        # 4. Two rigid outer struts outside Ø33.0 mm turntable skirt (at Y = ±19.2 mm)
        pillar_y = 19.2
        for sy in (-1, 1):
            with bd.Locations((0, sy * pillar_y, COLLAR_Z0)):
                bd.Box(16.0, 3.6, ear_z_top - COLLAR_Z0 + 2.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
            with bd.Locations((0, sy * (pillar_y / 2 + 4.0), TURN_TOP_Z + 0.4)):
                bd.Box(16.0, pillar_y - 5.0, COLLAR_Z1 - (TURN_TOP_Z + 0.4), align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
            with bd.Locations((0, sy * 16.0, ear_z_top)):
                bd.Box(16.0, 9.5, 4.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))

        # 5. Subtract flipped TD-8125MG servo envelope + wire-boot slot
        with bd.Locations((SERVO_X_OFF, 0, SERVO_CAP_Z - 0.3)):
            bd.Box(srv_l, srv_w, (SERVO_TOP_Z - SERVO_CAP_Z) + 0.8, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
        with bd.Locations((0, 0, COLLAR_Z0 - 0.5)):
            bd.Cylinder(radius=S.BOSS_D / 2 + 0.6, height=10.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
        with bd.Locations((SERVO_X_OFF - S.BODY_L / 2 - 4.0, 0, SERVO_TOP_Z - 14.0)):
            bd.Box(12.0, 9.0, 14.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)

        # 6. 4x M3 ear holes
        for sx in (-1, 1):
            for sy in (-1, 1):
                with bd.Locations((SERVO_X_OFF + sx * S.HOLE_PITCH_X / 2, sy * S.HOLE_PITCH_Y / 2, ear_z_top + 10.0)):
                    bd.Hole(radius=1.4, depth=12.0)

    part = bm.part
    part.label = "TSlot_Body_Servo_Mount"
    part.color = bd.Color(0.20, 0.72, 0.45)
    return part


def build_in_slot_rack(length: float = 200.0) -> bd.Part:
    """In-slot toothed rack shaped to lock cleanly inside the exact 2020 blueprint T-slot (MOD=1.5, PA=20 deg)."""
    mod = 1.5
    pitch = math.pi * mod
    pa = math.radians(20.0)
    ded = 1.25 * mod
    add = 0.8 * mod
    z_rail_top = -G.RAIL_CENTER_DROP + G.RAIL_W / 2.0     # -2.475 mm
    z_floor = z_rail_top - G.SLOT_DEPTH                  # -9.475 mm (exact 7.00 mm T-slot floor)
    z_bev = z_rail_top - G.POCKET_SIDE_Z                 # -6.525 mm (4.05 mm below top)
    z_lip = z_rail_top - G.FLANGE_T                      # -3.975 mm (1.50 mm below top)
    z_root = z_rail_top - 5.5                            # -7.975 mm
    z_pitch = z_root + ded                               # -6.100 mm
    z_tip = z_pitch + add                                # -4.900 mm
    s = pitch / 2.0 - 0.30
    hw = lambda z: s / 2.0 + (z_pitch - z) * math.tan(pa)

    n_teeth = int(length / pitch)
    x0 = -n_teeth * pitch / 2.0
    pts = [(x0 - 1.0, z_root - 0.3)]
    for i in range(n_teeth):
        xc = x0 + (i + 0.5) * pitch
        pts += [
            (xc - hw(z_root), z_root),
            (xc - hw(z_tip), z_tip),
            (xc + hw(z_tip), z_tip),
            (xc + hw(z_root), z_root),
        ]
    pts.append((x0 + n_teeth * pitch + 1.0, z_root - 0.3))

    with bd.BuildPart() as rk:
        # Tooth band: 5.2 mm wide -> fits cleanly inside the 6.00 mm slot neck (0.40 mm clearance each side)
        with bd.BuildSketch(bd.Plane.XZ):
            bd.Polygon(*pts, align=None)
        bd.extrude(amount=2.6, both=True)
        # Trapezoidal T-slot base locked inside the 11.90 mm T-pocket & 1.59 mm X-web bevels (Z = -9.275 .. -4.275).
        # Only the two WINGS (|Y| 2.9..5.2, under the slot lips -> the rack cannot pull out through the 6 mm neck) rise to the
        # lips; between them a 5.8 mm channel stays open down to the tooth roots, so the teeth stand free and the 5 mm pinion
        # runs in the channel (0.4 mm each side).  (Base up to the lips across the full width buried the teeth: tips -4.9.)
        z_xbev = z_rail_top - (G.RAIL_W / 2.0 - (G.SLOT_INNER / 2.0 + 1.59 / (2.0 ** 0.5)))  # -5.401 mm
        ch = 2.9                                          # half width of the tooth channel
        with bd.BuildSketch(bd.Plane.YZ):
            bd.Polygon(
                (-1.7, z_floor + 0.2),
                (1.7, z_floor + 0.2),
                (5.2, z_xbev - 0.3),
                (5.2, z_lip - 0.3),
                (ch, z_lip - 0.3),
                (ch, z_root),
                (-ch, z_root),
                (-ch, z_lip - 0.3),
                (-5.2, z_lip - 0.3),
                (-5.2, z_xbev - 0.3),
                align=None,
            )
        bd.extrude(amount=(n_teeth * pitch) / 2.0 + 1.0, both=True)
    part = rk.part
    part.label = "T_Slot_In_Rack"
    part.color = bd.Color(0.95, 0.70, 0.25)
    return part


def build_assembled_parts():
    """Return dict of all bodies in exact gantry-centered coordinates (mm)."""
    # 1. MT0134 Gantry Plate (stock Ø5.10 center hole) + 4 Delrin V-Wheel stacks
    gantry_plate = G.build_mt0134_plate(5.10)
    wheel_solids = []
    for sx in (-20.0, 20.0):
        for sy, is_ecc in ((-G.WHEEL_Y, True), (G.WHEEL_Y, False)):
            w_cmp = G.build_wheel_stack(is_eccentric=is_ecc)
            loc = bd.Location((sx, sy, 0))
            wheel_solids.extend([s.moved(loc) for s in w_cmp.solids()])
    v_wheels = bd.Compound(children=wheel_solids, label="Delrin_V_Wheels_x4")

    # 2. 4632-370 Worm Gear Motor mounted 90-deg sideways.
    #    In build_4632_370, gearbox occupies Z_local = 0..22.5 and shaft points +Z_local.
    #    Origin at Y = GB_FACE_Y + M.GB_H (35.3 mm) with z_dir = (0, -1, 0) puts:
    #      back of gearbox (Z_local=0)    at Y = +35.3 mm
    #      shaft face      (Z_local=22.5) at Y = +12.8 mm (GB_FACE_Y)
    #      shaft tip       (Z_local=38.3) at Y = -3.0 mm
    motor_plane = bd.Plane(origin=(GB_X_CENTER, GB_FACE_Y + M.GB_H, GB_Z_CENTER), x_dir=(0, 0, -1), z_dir=(0, -1, 0))
    motor_loc = motor_plane.location

    housing_solids = []
    for sub in (M.build_gearbox(), M.build_cover_screws(), M.build_motor_can(), M.build_terminals()):
        housing_solids.extend(sub.solids() if isinstance(sub, bd.Compound) else [sub])
    motor_housing = bd.Compound(children=[s.moved(motor_loc) for s in housing_solids], label="4632-370_Housing")
    motor_shaft = M.build_output_shaft().moved(motor_loc)

    # 3. 14T Climber Pinion + Retainer on the -Y shaft at (PINION_X=52.5, Y=0, Z=5.325)
    #    Teeth span Y = -2.5 .. +2.5 (inside 6.2 mm T-slot), 6.0 mm hub spans Y = +2.5 .. +8.5 (clears front wall Y=10.2)
    with bd.BuildPart() as pin_bp:
        with bd.BuildSketch():
            bd.Polygon(*K.involute_points(), align=None)
            bd.Polygon(*K.d_bore_points(), align=None, mode=bd.Mode.SUBTRACT)
        bd.extrude(amount=5.0)
        with bd.Locations((0, 0, 5.0)):
            with bd.BuildSketch():
                bd.Circle(radius=K.HUB_D / 2)
                bd.Polygon(*K.d_bore_points(), align=None, mode=bd.Mode.SUBTRACT)
            bd.extrude(amount=4.8)
        with bd.Locations(bd.Location((0, K.HUB_D / 2 + 1.0, 7.4), (90, 0, 0))):
            bd.Hole(radius=1.25, depth=K.HUB_D / 2)
    pin_rot = bd.Location((PINION_X, -2.5, PINION_Z)) * bd.Rotation(-90, 0, 0) * bd.Rotation(0, 0, -90)
    pinion = pin_bp.part.moved(pin_rot)
    pinion.label = "TSlot_Pinion_14T"
    pinion.color = bd.Color(0.92, 0.48, 0.14)

    retainer = K.build_shaft_retainer().moved(bd.Location((PINION_X, -3.1, PINION_Z), (90, 0, 0)))

    carriage_pod = build_carriage_pod_housing()
    alum_horn = build_servo_horn_25t()

    # 4. Flipped TD-8125MG Servo at (SERVO_X_OFF=-10, 0, SERVO_TOP_Z=58.3)
    servo_flip = bd.Location((SERVO_X_OFF, 0, SERVO_TOP_Z), (180, 0, 0))
    servo_solids = []
    for sub in (S.build_case_bottom(), S.build_case_mid(), S.build_case_top()):
        servo_solids.extend(sub.solids() if isinstance(sub, bd.Compound) else [sub])
    servo_body = bd.Compound(children=[s.moved(servo_flip) for s in servo_solids], label="TD8125MG_Body")
    spline_local_z = S.BOT_H + S.MID_H + S.TOP_BASE_H + S.RIDGE_H + S.BOSS_H
    servo_spline = S.build_spline().moved(bd.Location((S.SHAFT_X, 0, spline_local_z))).moved(servo_flip)

    body_mount = build_body_servo_mount()

    # 5. Reference 2020 T-Slot Rail + In-Slot Rack (centered at Z = -12.475 mm)
    t_rail = G.build_t_slot_2020(160.0).moved(bd.Location((20.0, 0, -G.RAIL_CENTER_DROP)))
    t_rack = build_in_slot_rack(240.0).moved(bd.Location((20.0, 0, 0)))

    return {
        "gantry_plate": gantry_plate,
        "v_wheels": v_wheels,
        "motor_housing": motor_housing,
        "motor_shaft": motor_shaft,
        "pinion": pinion,
        "retainer": retainer,
        "carriage_pod": carriage_pod,
        "alum_horn": alum_horn,
        "servo_spline": servo_spline,
        "servo_body": servo_body,
        "body_mount": body_mount,
        "t_rail": t_rail,
        "t_rack": t_rack,
    }


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    os.makedirs(SIM_MESH_DIR, exist_ok=True)

    parts = build_assembled_parts()
    printable = {
        "tslot_carriage_pod": parts["carriage_pod"],
        "tslot_body_mount": parts["body_mount"],
        "tslot_25t_horn": parts["alum_horn"],
        "tslot_pinion_14t": parts["pinion"],
        "tslot_shaft_retainer": parts["retainer"],
    }
    for name, part in printable.items():
        bd.export_step(part, os.path.join(OUT_DIR, f"{name}.step"))
        bd.export_stl(part, os.path.join(OUT_DIR, f"{name}.stl"), tolerance=0.01, angular_tolerance=0.1)
        bb = part.bounding_box()
        print(f"{name:24s} -> {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm")

    asm = bd.Compound(children=list(parts.values()), label="TSlot_Swivel_Carriage_Assembly")
    asm_step = os.path.join(OUT_DIR, "tslot_swivel_carriage_complete.step")
    bd.export_step(asm, asm_step)
    print(f"Exported full STEP: {asm_step}")

    # Export rig meshes (note: pinion, shaft, retainer are shifted to origin for spinning around Y!)
    pinion_origin = bd.Location((-PINION_X, 0, -PINION_Z))
    for k, v in parts.items():
        m = v.moved(pinion_origin) if k in ("motor_shaft", "pinion", "retainer") else v
        bd.export_stl(m, os.path.join(SIM_MESH_DIR, f"{k}.stl"), tolerance=0.01, angular_tolerance=0.1)
    print(f"Exported {len(parts)} rig meshes to {SIM_MESH_DIR}")


if __name__ == "__main__":
    main()
