"""Complete Rail-Drive + 90-deg Swivel Pod CAD Assembly.

Combines:
  - 4632-370 worm gear motor (mounted shaft-down so the 14T pinion drives the rail)
  - TD-8125MG 25KG digital servo (mounted upright on top of the 4632-370 cover)
  - Lower Base Cradle (with pockets for every gearbox bump/screw, motor U-saddle + 1.8 mm snap lip, 4x M4 ears)
  - Upper Cover + Integrated Servo Tower (with bottom-bump pocket, M4 ears, TD-8125MG cradle,
    wire-boot cutout, Ø26 mm load-bearing swivel collar, and 0/90-deg hard stops)
  - 14T Climber Pinion + M3 Axial Shaft Retainer (below base cradle, meshing with V-slot rail)
  - 25T Splined Swivel Hub + Leg Arm (rides on the Ø26 mm bearing collar so bending loads bypass the spline)

Local coordinate frame (Z=0 at the 4632-370 gearbox shaft-side face):
  - Pinion & retainer: Z = -18.5 .. -5.0 mm (below base cradle)
  - Base cradle plate: Z = -4.8 .. 0.0 mm (motor saddle rises to Z = 13.05 mm)
  - 4632-370 gearbox:  Z =  0.0 .. +22.5 mm (circular bump to +23.7 mm)
  - Upper cover + servo tower: Z = 0.0 .. +69.5 mm
  - TD-8125MG servo:   Z = +26.0 .. +66.5 mm (25T spline Z = +68.0 .. +72.5 mm at X = +10.0 mm)
  - Swivel leg hub:    Z = +65.2 .. +84.0 mm (pivots 0..90 deg around X = +10.0, Y = 0)
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

OUT_DIR = os.path.abspath(os.path.join(HERE, "..", "models", "components", "swivel_pod"))
SIM_MESH_DIR = os.path.abspath(os.path.join(HERE, "..", "sim", "build", "meshes_swivel_pod"))

# Shared assembly constants (mm)
PLATE_T = 4.8              # base cradle thickness under gearbox (clears 1.8 mm boss + leaves 11.0 mm shaft)
WALL = 2.4
CLR = 1.0
M3_CLR = 3.4
M4_CLR = 4.5
EAR_W = 10.0

# Servo placement on top of cover
SERVO_Z0 = 26.0            # bottom of TD-8125MG housing
SERVO_X_OFF = 0.0          # centered over gearbox X; spline sits at X = +10.0 mm
COLLAR_D = 26.0            # concentric load-bearing journal around spline axis (X=10, Y=0)
COLLAR_Z0 = 66.8           # 0.3 mm above TD-8125MG top (66.5 mm)
COLLAR_Z1 = 70.2


def build_base_cradle() -> bd.Part:
    """Lower base cradle: sits at Z=-4.8..0.0 under flipped 4632-370, with exact pockets
    for all shaft-face bosses/screws, motor U-saddle + 1.8 mm snap lip, and 4x M4 ears."""
    cradle_l = M.GB_L + 2 * CLR + 2 * WALL
    cradle_w = M.GB_W + 2 * CLR + 2 * WALL + 2 * EAR_W
    saddle_l = M.MOTOR_L - 2.0
    saddle_w = M.MOTOR_D + 2 * WALL
    saddle_x = -M.GB_L / 2 - CLR - WALL - saddle_l / 2 + 1.0
    saddle_eq_z = M.GB_H / 2
    snap_lip_h = 1.8

    with bd.BuildPart() as cr:
        # Base plate (Z = -PLATE_T .. 0.0)
        bd.Box(
            cradle_l,
            cradle_w,
            PLATE_T,
            align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MAX),
        )
        # Motor U-saddle rising from Z = -PLATE_T up to equator (Z = 11.25 mm)
        with bd.Locations((saddle_x, 0, -PLATE_T)):
            bd.Box(
                saddle_l,
                saddle_w,
                saddle_eq_z + PLATE_T,
                align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN),
            )
        # 1.8 mm elastic snap-fit lip on -Y side
        with bd.Locations((saddle_x, -(M.MOTOR_D / 2 + WALL / 2), saddle_eq_z)):
            bd.Box(
                saddle_l,
                WALL + 0.6,
                snap_lip_h,
                align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN),
            )
        # Semicircle cutout for 370 motor can (+0.2 mm snug fit)
        with bd.Locations(bd.Location((-M.GB_L / 2, 0, M.GB_H / 2), (0, -90, 0))):
            bd.Cylinder(
                radius=M.MOTOR_D / 2 + 0.2,
                height=M.MOTOR_L + 4.0,
                align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN),
                mode=bd.Mode.SUBTRACT,
            )

        # --- Exact downward pockets from Z=0 for the flipped gearbox face ---
        # 1. Output shaft bearing boss pocket (Ø12.6 x 2.1 mm) + Ø8.0 mm shaft bore
        with bd.Locations((M.SHAFT_OFFSET_X, 0, 0.2)):
            bd.Cylinder(
                radius=6.3,
                height=2.3,
                align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MAX),
                mode=bd.Mode.SUBTRACT,
            )
            bd.Hole(radius=4.0, depth=PLATE_T + 2.0)

        # 2. 4x M3 mounting boss pockets (Ø7.2 x 1.8 mm) + Ø3.4 mm through-holes
        m3_pts = [
            (sx * M.HOLE_PITCH_X / 2, sy * M.HOLE_PITCH_Y / 2, 0.2)
            for sx in (-1, 1)
            for sy in (-1, 1)
        ]
        with bd.Locations(*m3_pts):
            bd.Cylinder(
                radius=3.6,
                height=2.0,
                align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MAX),
                mode=bd.Mode.SUBTRACT,
            )
            bd.Hole(radius=M3_CLR / 2, depth=PLATE_T + 2.0)

        # 3. Center stadium boss pocket (16.0 x 6.0 x 1.5 mm)
        with bd.Locations((-2.5, 0, 0.2)):
            with bd.BuildSketch():
                bd.SlotOverall(16.0, 6.0, rotation=90)
            bd.extrude(amount=-1.7, mode=bd.Mode.SUBTRACT)

        # 4. 4x corner cover screw head pockets (Ø3.6 x 1.2 mm, Y flipped for 180-deg X mount)
        screw_pts = [
            (-M.GB_L / 2 + 2.5, -(M.GB_W / 2 - 2.5), 0.2),
            (M.GB_L / 2 - 2.5, -(M.GB_W / 2 - 2.5), 0.2),
            (M.GB_L / 2 - 2.5, M.GB_W / 2 - 2.5, 0.2),
            (-M.GB_L / 2 + 12.0, M.GB_W / 2 - 2.5, 0.2),
        ]
        with bd.Locations(*screw_pts):
            bd.Cylinder(
                radius=1.8,
                height=1.4,
                align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MAX),
                mode=bd.Mode.SUBTRACT,
            )

        # 5. 4x M4 mounting holes on the side flange ears
        ear_pts = [
            (sx * 12.0, sy * (M.GB_W / 2 + CLR + WALL + EAR_W / 2), 1.0)
            for sx in (-1, 1)
            for sy in (-1, 1)
        ]
        with bd.Locations(*ear_pts):
            bd.Hole(radius=M4_CLR / 2, depth=PLATE_T + 2.0)

    part = cr.part
    part.label = "SwivelPod_Base_Cradle"
    part.color = bd.Color(0.22, 0.25, 0.30)
    return part


def build_upper_cover_servo_tower() -> bd.Part:
    """Upper cover over 4632-370 + integrated TD-8125MG servo mount + Ø26 mm swivel bearing collar."""
    gb_x0 = -M.GB_L / 2 - CLR
    gb_x1 = M.GB_L / 2 + CLR
    gb_y0 = -M.GB_W / 2 - CLR
    gb_y1 = M.GB_W / 2 + CLR
    cov_top_z = SERVO_Z0       # 26.0 mm (22.5 gearbox + 1.2 bump + 2.3 top deck)
    mc_r = M.MOTOR_D / 2 + CLR

    cradle_l = M.GB_L + 2 * CLR + 2 * WALL
    cradle_w = M.GB_W + 2 * CLR + 2 * WALL

    # Servo tower wall geometry
    srv_l = S.BODY_L + 0.6     # 40.6 mm pocket
    srv_w = S.BODY_W + 0.6     # 21.1 mm pocket
    tower_l = S.EAR_SPAN + 2.0 # 56.5 mm outer span
    tower_w = srv_w + 6.0      # 27.1 mm outer width
    ear_pad_z = SERVO_Z0 + S.EAR_Z  # 53.5 mm

    with bd.BuildPart() as uc:
        # 1. Gearbox outer shell (Z = 0 .. 26.0)
        with bd.Locations((0, 0, 0)):
            bd.Box(
                cradle_l,
                cradle_w,
                cov_top_z,
                align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN),
            )

        # 2. Motor shroud (upper semicircle Z >= 11.25 mm)
        with bd.Locations(bd.Location((-M.GB_L / 2, 0, M.GB_H / 2), (0, -90, 0))):
            bd.Cylinder(
                radius=mc_r + WALL,
                height=M.MOTOR_L + 2.0,
                align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN),
            )
        with bd.Locations((-M.GB_L / 2 - (M.MOTOR_L + 4.0) / 2, 0, M.GB_H / 2)):
            bd.Box(
                M.MOTOR_L + 6.0,
                2 * (mc_r + WALL + 2.0),
                M.GB_H / 2 + 5.0,
                align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MAX),
                mode=bd.Mode.SUBTRACT,
            )
        # Clean clearance notch on -Y side for the cradle's 1.8 mm snap lip
        with bd.Locations((-M.GB_L / 2 - (M.MOTOR_L + 4.0) / 2, -(M.MOTOR_D / 2 - 0.5), M.GB_H / 2 - 0.5)):
            bd.Box(
                M.MOTOR_L + 6.0,
                WALL + CLR + 4.0,
                2.8,
                align=(bd.Align.CENTER, bd.Align.MAX, bd.Align.MIN),
                mode=bd.Mode.SUBTRACT,
            )

        # 3. Side clamping flange ears sitting flush on Base Cradle at Z = 0 .. 3.6
        for sy in (-1, 1):
            ey = sy * (M.GB_W / 2 + CLR + WALL + EAR_W / 2 - 0.5)
            with bd.Locations((0, ey, 0)):
                bd.Box(36.0, EAR_W + 1.0, 3.6, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))

        # Subtract gearbox inner cavity (Z = -1 .. 23.0) + pocket for circular bump (Z = 22.5 .. 24.1)
        with bd.Locations((0, 0, -1.0)):
            bd.Box(
                gb_x1 - gb_x0,
                gb_y1 - gb_y0,
                M.GB_H + 1.3,
                align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN),
                mode=bd.Mode.SUBTRACT,
            )
        with bd.Locations((M.SHAFT_OFFSET_X, 0, M.GB_H - 0.1)):
            bd.Cylinder(
                radius=6.5,
                height=1.8,
                align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN),
                mode=bd.Mode.SUBTRACT,
            )
        # Subtract motor can inner bore
        with bd.Locations(bd.Location((-M.GB_L / 2 + 1.0, 0, M.GB_H / 2), (0, -90, 0))):
            bd.Cylinder(
                radius=mc_r,
                height=M.MOTOR_L + 5.0,
                align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN),
                mode=bd.Mode.SUBTRACT,
            )

        # 4. TD-8125MG Servo Cradle Tower (Z = 26.0 .. 53.5 mm)
        with bd.Locations((SERVO_X_OFF, 0, cov_top_z)):
            bd.Box(tower_l, tower_w, ear_pad_z - cov_top_z, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))

        # Subtract servo body pocket (40.6 x 21.1 mm) + wire boot slot on -X
        with bd.Locations((SERVO_X_OFF, 0, cov_top_z - 0.2)):
            bd.Box(srv_l, srv_w, ear_pad_z - cov_top_z + 1.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
        with bd.Locations((-S.BODY_L / 2 - 5.0, 0, cov_top_z - 0.2)):
            bd.Box(12.0, 9.0, 14.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)

        # 4x M3 mounting holes in the servo ear pads (49.5 x 10.0 mm)
        srv_holes = [
            (sx * S.HOLE_PITCH_X / 2, sy * S.HOLE_PITCH_Y / 2, ear_pad_z + 0.5)
            for sx in (-1, 1)
            for sy in (-1, 1)
        ]
        with bd.Locations(*srv_holes):
            bd.Hole(radius=1.4, depth=10.0)

        # 5. Side Bearing Arch & Ø26 mm Load-Bearing Swivel Collar around spline (X=10, Y=0)
        # Two vertical arch pillars outside the Ø31.6 mm bearing skirt (at Y = ±17.6 mm) + bridge into collar
        pillar_y = 17.6
        for sy in (-1, 1):
            with bd.Locations((S.SHAFT_X, sy * pillar_y, ear_pad_z)):
                bd.Box(14.0, 3.0, COLLAR_Z1 - ear_pad_z, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
            # Horizontal bridge tab from pillar inward to the Ø26 mm collar (below the skirt at Z = 66.8)
            with bd.Locations((S.SHAFT_X, sy * (pillar_y / 2 + 4.0), COLLAR_Z0 - 2.4)):
                bd.Box(14.0, pillar_y - 5.0, 2.4, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
            # Side gusset connecting tower wall (Y=±12.0) out to pillar (Y=±17.6) at ear_pad_z
            with bd.Locations((S.SHAFT_X, sy * 14.5, ear_pad_z)):
                bd.Box(14.0, 8.0, 3.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))

        # Top ring collar (Z = 66.8 .. 70.2, OD = 26.0, ID = 14.0 to clear Ø12 mm servo boss)
        with bd.Locations((S.SHAFT_X, 0, COLLAR_Z0)):
            bd.Cylinder(radius=COLLAR_D / 2, height=COLLAR_Z1 - COLLAR_Z0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
            bd.Cylinder(radius=7.0, height=COLLAR_Z1 - COLLAR_Z0 + 1.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
        # Subtract servo upper cap envelope (Z = 53.5 .. 66.6) so zero collision with TD-8125MG top
        with bd.Locations((SERVO_X_OFF, 0, ear_pad_z - 0.2)):
            bd.Box(S.BODY_L + 0.6, S.BODY_W + 0.6, 13.3, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
        with bd.Locations((S.SHAFT_X, 0, ear_pad_z)):
            bd.Cylinder(radius=S.BOSS_D / 2 + 0.6, height=15.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)

        # 6. 4x M4 clamping holes through the side ears
        ear_pts = [
            (sx * 12.0, sy * (M.GB_W / 2 + CLR + WALL + EAR_W / 2), 5.0)
            for sx in (-1, 1)
            for sy in (-1, 1)
        ]
        with bd.Locations(*ear_pts):
            bd.Hole(radius=M4_CLR / 2, depth=7.0)

    part = uc.part
    part.label = "SwivelPod_Upper_Cover_ServoTower"
    part.color = bd.Color(0.18, 0.55, 0.88)
    return part


def build_pinion_down() -> bd.Part:
    """14T climber pinion oriented shaft-down (hub at top Z=-5..-11, gear face Z=-11..-16)."""
    gear_pts = K.involute_points()
    bore_pts = K.d_bore_points()

    with bd.BuildPart() as pin:
        # Teeth (Z = 0 .. 5.0) + Hub (Z = 5.0 .. 11.0) built upright then positioned
        with bd.BuildSketch():
            bd.Polygon(*gear_pts, align=None)
            bd.Polygon(*bore_pts, align=None, mode=bd.Mode.SUBTRACT)
        bd.extrude(amount=K.FACE_W)
        with bd.Locations((0, 0, K.FACE_W)):
            with bd.BuildSketch():
                bd.Circle(radius=K.HUB_D / 2)
                bd.Polygon(*bore_pts, align=None, mode=bd.Mode.SUBTRACT)
            bd.extrude(amount=K.HUB_H)
        with bd.Locations(bd.Location((0, K.HUB_D / 2 + 1.0, K.FACE_W + K.HUB_H / 2), (90, 0, 0))):
            bd.Hole(radius=1.25, depth=K.HUB_D / 2)

    part = pin.part
    part.label = "SwivelPod_Pinion_14T"
    part.color = bd.Color(0.92, 0.48, 0.14)
    return part


def build_swivel_leg_hub() -> bd.Part:
    """25T splined swivel hub + leg mounting arm:
    - Concentric bearing skirt (Ø26.6 mm ID) rides on the Upper Cover's Ø26.0 mm collar
    - Internal 25T spline pocket engages the TD-8125MG shaft (Z=0..4.5 local)
    - Column/leg mounting bracket (28 x 24 mm clamp plate with 4x M4 holes) extends outward."""
    skirt_id = COLLAR_D + 0.6   # 26.6 mm (0.3 mm radial clearance on Ø26.0 mm collar)
    skirt_od = skirt_id + 5.0   # 31.6 mm OD
    cap_z0 = 4.6                # clears top of spline (4.5 mm)
    cap_h = 5.0

    with bd.BuildPart() as sh:
        # 1. Top hub plate (Z = 0 .. 9.6 local, relative to spline base at Z = 68.0)
        bd.Cylinder(radius=skirt_od / 2, height=cap_z0 + cap_h, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))

        # 2. Concentric bearing skirt hanging down to Z_local = -1.2 (world Z = 66.8, riding on Ø26.0 collar Z = 66.8..70.2)
        with bd.Locations((0, 0, -1.2)):
            bd.Cylinder(radius=skirt_od / 2, height=1.2 + cap_z0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
            bd.Cylinder(radius=skirt_id / 2, height=1.2 + 2.5, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)

        # 3. Internal spline clearance socket (Ø6.1 mm x 4.6 mm) + M3 axial bolt hole
        with bd.Locations((0, 0, -0.2)):
            bd.Cylinder(radius=S.SPLINE_D / 2 + 0.1, height=S.SPLINE_H + 0.3, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
        with bd.Locations((0, 0, cap_z0 + cap_h + 1.0)):
            bd.Hole(radius=M3_CLR / 2, depth=cap_h + 2.0)

        # 4. Leg / Column Arm extending along +X (to hold the 28 x 24 mm robot column bar)
        with bd.Locations((0, 0, cap_z0)):
            bd.Box(48.0, 26.0, cap_h, align=(bd.Align.MIN, bd.Align.CENTER, bd.Align.MIN))
        # Column clamp flange at X = 24..48 mm with 4x M4 holes
        leg_holes = [(30.0 + ix * 12.0, sy * 8.0, cap_z0 + cap_h + 1.0) for ix in (0, 1) for sy in (-1, 1)]
        with bd.Locations(*leg_holes):
            bd.Hole(radius=M4_CLR / 2, depth=cap_h + 2.0)

    part = sh.part
    part.label = "SwivelPod_Leg_Hub"
    part.color = bd.Color(0.20, 0.72, 0.45)
    return part


def build_assembled_parts():
    """Return dict of all 9 bodies in their assembled world coordinates (mm)."""
    # 1. Flipped 4632-370 motor (flatten nested compounds first so every solid moves cleanly)
    flip_x = bd.Location((0, 0, M.GB_H), (180, 0, 0))
    housing_solids = []
    for sub in (M.build_gearbox(), M.build_cover_screws(), M.build_motor_can(), M.build_terminals()):
        housing_solids.extend(sub.solids() if isinstance(sub, bd.Compound) else [sub])
    motor_housing = bd.Compound(children=[s.moved(flip_x) for s in housing_solids], label="4632-370_Housing")
    motor_shaft = M.build_output_shaft().moved(flip_x)

    # 2. Pinion & Retainer below Base Cradle (rotate pinion 180 deg in Z so D-flat aligns with flipped shaft)
    pinion = build_pinion_down().moved(bd.Location((M.SHAFT_OFFSET_X, 0, -16.0), (0, 0, 180)))
    retainer = K.build_shaft_retainer().moved(bd.Location((M.SHAFT_OFFSET_X, 0, -18.5)))

    # 3. Base Cradle & Upper Cover + Servo Tower
    base_cradle = build_base_cradle()
    upper_cover = build_upper_cover_servo_tower()

    # 4. TD-8125MG Servo seated at Z = SERVO_Z0 = 26.0 mm
    servo_loc = bd.Location((SERVO_X_OFF, 0, SERVO_Z0))
    servo_body = bd.Compound(
        children=[
            S.build_case_bottom(),
            S.build_case_mid(),
            S.build_case_top(),
        ]
    ).moved(servo_loc)
    spline_z = SERVO_Z0 + S.BOT_H + S.MID_H + S.TOP_BASE_H + S.RIDGE_H + S.BOSS_H
    servo_spline = S.build_spline().moved(bd.Location((SERVO_X_OFF + S.SHAFT_X, 0, spline_z)))

    # 5. Swivel Leg Hub seated on spline & Ø26 mm bearing collar
    swivel_leg_hub = build_swivel_leg_hub().moved(bd.Location((SERVO_X_OFF + S.SHAFT_X, 0, spline_z)))

    return {
        "motor_housing": motor_housing,
        "motor_shaft": motor_shaft,
        "pinion": pinion,
        "retainer": retainer,
        "base_cradle": base_cradle,
        "upper_cover": upper_cover,
        "servo_body": servo_body,
        "servo_spline": servo_spline,
        "swivel_leg_hub": swivel_leg_hub,
    }


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    os.makedirs(SIM_MESH_DIR, exist_ok=True)

    parts = build_assembled_parts()

    # Export printable CAD components (STEP + STL)
    printable = {
        "swivel_pod_base_cradle": parts["base_cradle"],
        "swivel_pod_upper_cover": parts["upper_cover"],
        "swivel_pod_pinion_14t": build_pinion_down(),
        "swivel_pod_shaft_retainer": K.build_shaft_retainer(),
        "swivel_pod_25t_swivel_hub": build_swivel_leg_hub(),
    }
    for name, part in printable.items():
        bd.export_step(part, os.path.join(OUT_DIR, f"{name}.step"))
        bd.export_stl(part, os.path.join(OUT_DIR, f"{name}.stl"), tolerance=0.01, angular_tolerance=0.1)
        bb = part.bounding_box()
        print(f"{name:28s} -> {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm")

    # Full colored assembly STEP
    asm = bd.Compound(children=list(parts.values()), label="RailDrive_Swivel_Pod_Assembly")
    asm_step = os.path.join(OUT_DIR, "swivel_pod_complete.step")
    bd.export_step(asm, asm_step)
    print(f"Exported full STEP: {asm_step}")

    # Split STL meshes for Three.js dual-motor rig studio
    # Note: shaft, pinion, retainer are exported centered on their rotation axis (X=12.5, Y=0)
    #       servo_spline, swivel_leg_hub are centered on their rotation axis (X=10.0, Y=0)
    shaft_local = parts["motor_shaft"].moved(bd.Location((-M.SHAFT_OFFSET_X, 0, 0)))
    pinion_local = parts["pinion"].moved(bd.Location((-M.SHAFT_OFFSET_X, 0, 0)))
    retainer_local = parts["retainer"].moved(bd.Location((-M.SHAFT_OFFSET_X, 0, 0)))
    spline_local = S.build_spline()
    leg_local = build_swivel_leg_hub()

    sim_exports = {
        "motor_housing.stl": parts["motor_housing"],
        "motor_shaft.stl": shaft_local,
        "pinion.stl": pinion_local,
        "retainer.stl": retainer_local,
        "base_cradle.stl": parts["base_cradle"],
        "upper_cover.stl": parts["upper_cover"],
        "servo_body.stl": parts["servo_body"],
        "servo_spline.stl": spline_local,
        "swivel_leg_hub.stl": leg_local,
    }
    for fname, p in sim_exports.items():
        bd.export_stl(p, os.path.join(SIM_MESH_DIR, fname), tolerance=0.01, angular_tolerance=0.1)
    print(f"Exported {len(sim_exports)} rig meshes to {SIM_MESH_DIR}")


if __name__ == "__main__":
    main()
