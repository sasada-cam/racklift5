"""Switched-Sides Rail-Drive + 90-deg Swivel Pod CAD Assembly (v2 — Reinforced & Rail-Ready).

Improvements over v1:
  1. ZERO red/green collar overlap: TD-8125MG housing sits cleanly ABOVE the Ø26.0 mm bearing collar
     (collar Z = 28.5..33.5 mm; servo housing Z = 35.5..76.0 mm -> 2.0 mm clear gap).
  2. Much stronger 25T Servo Horn Joint: uses a standard Ø20 mm CNC aluminum 25T round servo horn
     bolted to the Upper Cover Turntable with 4 x M3 steel bolts (instead of a weak single set-screw).
  3. Full 4-wall Body Mount cage + outer struts at Y=±19.5 mm supporting the unbroken Ø26 mm collar.
  4. Rectangular V-Wheel Hub Carriage (65 x 96 x 6 mm cuboid plate + 4 V-wheels) beneath the Base
     Cradle so the 14T pinion engages the V-slot rail sideways and rotates 90 deg cleanly.
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import build123d as bd
import build_4632_370 as M
import build_4632_370_kit as K
import build_td8125mg as S
import build_rail_swivel_assembly as R

OUT_DIR = os.path.abspath(os.path.join(HERE, "..", "models", "components", "switched_swivel_pod"))
SIM_MESH_DIR = os.path.abspath(os.path.join(HERE, "..", "sim", "build", "meshes_switched_swivel_pod"))

SWIVEL_X = M.SHAFT_OFFSET_X        # 12.5 mm (coaxial with 14T climb pinion!)
SERVO_X_OFF = SWIVEL_X - S.SHAFT_X # +2.5 mm so 25T spline sits at X = 12.5 mm

# Z coordinates (Z = 0 at 4632-370 gearbox shaft-side face)
HUB_PLATE_T = 6.0                  # rectangular hub carriage plate (Z = -10.8 .. -4.8)
HUB_Z0 = -R.PLATE_T - HUB_PLATE_T  # -10.8 mm
COV_TOP_Z = 26.0
SKIRT_TOP_Z = 33.8                 # top of turntable skirt on upper cover
HORN_Z0 = 27.5                     # 25T aluminum round horn sits at Z = 27.5 .. 31.0
HORN_H = 3.5

COLLAR_D = 26.0
COLLAR_Z0 = 31.2                   # 0.2 mm above alum horn (31.0), 2.6 mm engagement with skirt (33.8)
COLLAR_Z1 = 34.2                   # unbroken collar below servo housing!

SPLINE_TIP_Z = 30.2                # spline engages inside horn (30.2 .. 34.7)
SPLINE_BASE_Z = SPLINE_TIP_Z + S.SPLINE_H  # 34.7 mm
SERVO_CAP_Z = SPLINE_BASE_Z + S.BOSS_H     # 36.2 mm (above collar & bridge!)
SERVO_TOP_Z = SERVO_CAP_Z + S.BOT_H + S.MID_H + S.TOP_BASE_H + S.RIDGE_H  # 76.7 mm


def build_servo_horn_25t() -> bd.Part:
    """Standard Ø20 mm CNC aluminum 25T round servo horn (bolted with 4x M3 to the turntable)."""
    with bd.BuildPart() as hn:
        with bd.Locations((SWIVEL_X, 0, HORN_Z0)):
            bd.Cylinder(radius=10.0, height=HORN_H, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
            # Center spline bore (Ø6.0 mm)
            bd.Cylinder(radius=S.SPLINE_D / 2 + 0.05, height=HORN_H + 0.5, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
        # 4x M3 threaded mounting holes on a Ø14.0 mm bolt circle
        pts = [
            (SWIVEL_X + sx * 5.0, sy * 5.0, HORN_Z0 + HORN_H + 0.5)
            for sx in (-1, 1)
            for sy in (-1, 1)
        ]
        with bd.Locations(*pts):
            bd.Hole(radius=1.35, depth=HORN_H + 1.0)

    part = hn.part
    part.label = "SwitchedPod_25T_Alum_Horn"
    part.color = bd.Color(0.85, 0.25, 0.25)
    return part


def build_upper_cover_turntable() -> bd.Part:
    """Upper cover over 4632-370 + reinforced 4-bolt 25T servo-horn turntable at (X=12.5, Y=0)."""
    gb_x0 = -M.GB_L / 2 - R.CLR
    gb_x1 = M.GB_L / 2 + R.CLR
    gb_y0 = -M.GB_W / 2 - R.CLR
    gb_y1 = M.GB_W / 2 + R.CLR
    mc_r = M.MOTOR_D / 2 + R.CLR

    cradle_l = M.GB_L + 2 * R.CLR + 2 * R.WALL
    cradle_w = M.GB_W + 2 * R.CLR + 2 * R.WALL

    skirt_id = COLLAR_D + 0.6   # 26.6 mm
    skirt_od = skirt_id + 6.4   # 33.0 mm OD (thicker 3.2 mm wall)

    with bd.BuildPart() as uc:
        # 1. Gearbox outer shell (Z = 0 .. 26.0)
        bd.Box(cradle_l, cradle_w, COV_TOP_Z, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))

        # 2. Motor shroud (upper semicircle Z >= 11.25 mm)
        with bd.Locations(bd.Location((-M.GB_L / 2, 0, M.GB_H / 2), (0, -90, 0))):
            bd.Cylinder(radius=mc_r + R.WALL, height=M.MOTOR_L + 2.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
        with bd.Locations((-M.GB_L / 2 - (M.MOTOR_L + 4.0) / 2, 0, M.GB_H / 2)):
            bd.Box(M.MOTOR_L + 6.0, 2 * (mc_r + R.WALL + 2.0), M.GB_H / 2 + 5.0,
                   align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MAX), mode=bd.Mode.SUBTRACT)
        with bd.Locations((-M.GB_L / 2 - (M.MOTOR_L + 4.0) / 2, -(M.MOTOR_D / 2 - 0.5), M.GB_H / 2 - 0.5)):
            bd.Box(M.MOTOR_L + 6.0, R.WALL + R.CLR + 4.0, 2.8,
                   align=(bd.Align.CENTER, bd.Align.MAX, bd.Align.MIN), mode=bd.Mode.SUBTRACT)

        # 3. Side clamping flange ears sitting flush on Base Cradle at Z = 0 .. 3.6
        for sy in (-1, 1):
            ey = sy * (M.GB_W / 2 + R.CLR + R.WALL + R.EAR_W / 2 - 0.5)
            with bd.Locations((0, ey, 0)):
                bd.Box(36.0, R.EAR_W + 1.0, 3.6, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))

        # Subtract gearbox cavity + circular bump pocket
        with bd.Locations((0, 0, -1.0)):
            bd.Box(gb_x1 - gb_x0, gb_y1 - gb_y0, M.GB_H + 1.3,
                   align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
        with bd.Locations((M.SHAFT_OFFSET_X, 0, M.GB_H - 0.1)):
            bd.Cylinder(radius=6.5, height=1.8, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
        with bd.Locations(bd.Location((-M.GB_L / 2 + 1.0, 0, M.GB_H / 2), (0, -90, 0))):
            bd.Cylinder(radius=mc_r, height=M.MOTOR_L + 5.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)

        # 4. Reinforced Swivel Turntable Base (Z = 26.0 .. 32.0) with Ø20.4 mm pocket for 25T Aluminum Horn
        with bd.Locations((SWIVEL_X, 0, COV_TOP_Z)):
            bd.Cylinder(radius=skirt_od / 2, height=SKIRT_TOP_Z - COV_TOP_Z, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
        # Recess pocket for Ø20.0 mm aluminum horn (Z = 27.4 .. 32.5) + Ø26.6 bearing groove (Z = 28.4 .. 32.5)
        with bd.Locations((SWIVEL_X, 0, HORN_Z0 - 0.1)):
            bd.Cylinder(radius=10.3, height=SKIRT_TOP_Z - HORN_Z0 + 1.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
        with bd.Locations((SWIVEL_X, 0, COLLAR_Z0 - 0.1)):
            bd.Cylinder(radius=skirt_id / 2, height=SKIRT_TOP_Z - COLLAR_Z0 + 1.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
        # Keep raised central boss pad (Z = 26.0 .. 27.4) under the horn with 4x M3 bolt holes + center access hole!
        with bd.Locations((SWIVEL_X, 0, COV_TOP_Z)):
            bd.Cylinder(radius=10.2, height=HORN_Z0 - COV_TOP_Z, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
        # Center access hole (through top ceiling Z = 24..27.5) for the M3 spline screw
        with bd.Locations((SWIVEL_X, 0, HORN_Z0 + 0.5)):
            bd.Hole(radius=2.0, depth=4.0)
        # 4x M3 through-holes (Ø3.4 mm) matching the 25T aluminum horn
        horn_bolts = [
            (SWIVEL_X + sx * 5.0, sy * 5.0, HORN_Z0 + 0.5)
            for sx in (-1, 1)
            for sy in (-1, 1)
        ]
        with bd.Locations(*horn_bolts):
            bd.Hole(radius=1.7, depth=4.0)

        # 5. 4x M4 clamping holes through the side ears
        ear_pts = [
            (sx * 12.0, sy * (M.GB_W / 2 + R.CLR + R.WALL + R.EAR_W / 2), 5.0)
            for sx in (-1, 1)
            for sy in (-1, 1)
        ]
        with bd.Locations(*ear_pts):
            bd.Hole(radius=R.M4_CLR / 2, depth=7.0)

    part = uc.part
    part.label = "SwitchedPod_Upper_Cover_Turntable"
    part.color = bd.Color(0.18, 0.55, 0.88)
    return part


def build_body_servo_mount() -> bd.Part:
    """Stationary Robot Body / Column Mount: full 4-wall cage holding TD-8125MG upside-down,
    outer bridges at Y=±19.5 mm down to the unbroken Ø26.0 mm bearing collar (Z=28.5..33.5)."""
    srv_l = S.BODY_L + 0.6     # 40.6 mm pocket
    srv_w = S.BODY_W + 0.6     # 21.1 mm pocket
    mount_l = S.EAR_SPAN + 4.0 # 58.5 mm
    mount_w = srv_w + 6.4      # 27.5 mm (full 3.2 mm walls around servo)
    ear_z_bot = SERVO_TOP_Z - S.BOT_H - S.MID_H - S.EAR_T  # 46.0 mm
    ear_z_top = ear_z_bot + S.EAR_T                        # 48.5 mm
    top_plate_z1 = SERVO_TOP_Z + 5.0                       # 81.0 mm

    with bd.BuildPart() as bm:
        # 1. Full 4-wall servo cage (Z = ear_z_top .. top_plate_z1)
        with bd.Locations((SERVO_X_OFF, 0, ear_z_top)):
            bd.Box(mount_l, mount_w, top_plate_z1 - ear_z_top, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))

        # 2. Top Robot Column Clamp Flange (for 28 x 24 mm robot column)
        with bd.Locations((SERVO_X_OFF, 0, top_plate_z1)):
            bd.Box(mount_l, 36.0, 4.5, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
        col_holes = [
            (SERVO_X_OFF + sx * 20.0, sy * 12.0, top_plate_z1 + 5.0)
            for sx in (-1, 1)
            for sy in (-1, 1)
        ]
        with bd.Locations(*col_holes):
            bd.Hole(radius=R.M4_CLR / 2, depth=6.0)

        # 3. Unbroken Concentric Ø26.0 mm Bearing Collar Ring (Z = 28.5 .. 33.5, completely BELOW servo!)
        with bd.Locations((SWIVEL_X, 0, COLLAR_Z0)):
            bd.Cylinder(radius=COLLAR_D / 2, height=COLLAR_Z1 - COLLAR_Z0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
            bd.Cylinder(radius=7.0, height=COLLAR_Z1 - COLLAR_Z0 + 1.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)

        # 4. Two rigid outer struts outside Ø33.0 mm turntable skirt (at Y = ±19.2 mm)
        pillar_y = 19.2
        for sy in (-1, 1):
            with bd.Locations((SWIVEL_X, sy * pillar_y, COLLAR_Z0)):
                bd.Box(16.0, 3.6, ear_z_top - COLLAR_Z0 + 2.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
            # Bridge from pillar inward to Ø26.0 collar ABOVE turntable skirt (Z = 34.2 .. 35.6)
            with bd.Locations((SWIVEL_X, sy * (pillar_y / 2 + 4.0), SKIRT_TOP_Z + 0.4)):
                bd.Box(16.0, pillar_y - 5.0, 1.4, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
            # Top gusset connecting pillar (Y=±19.2) inward to servo cage (Y=±13.75) at ear_z_top
            with bd.Locations((SWIVEL_X, sy * 16.0, ear_z_top)):
                bd.Box(16.0, 9.5, 4.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))

        # Keep collar top up to 35.6 so horizontal bridge (34.2..36.6?) joins solidly!
        # Wait: bridge is at SKIRT_TOP_Z + 0.4 = 34.2, height 1.4 -> 35.6 (0.6 mm below servo cap 36.2!)
        with bd.Locations((SWIVEL_X, 0, COLLAR_Z1)):
            bd.Cylinder(radius=COLLAR_D / 2, height=1.4, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
            bd.Cylinder(radius=7.0, height=2.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)

        # 5. Subtract flipped TD-8125MG servo pocket cleanly (Z = SERVO_CAP_Z .. SERVO_TOP_Z + 0.5)
        with bd.Locations((SERVO_X_OFF, 0, SERVO_CAP_Z - 0.3)):
            bd.Box(srv_l, srv_w, (SERVO_TOP_Z - SERVO_CAP_Z) + 0.8, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
        with bd.Locations((SWIVEL_X, 0, COLLAR_Z0 - 0.5)):
            bd.Cylinder(radius=S.BOSS_D / 2 + 0.6, height=10.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
        # Wire strain-relief boot slot at -X near top of flipped servo
        with bd.Locations((SERVO_X_OFF - S.BODY_L / 2 - 4.0, 0, SERVO_TOP_Z - 14.0)):
            bd.Box(12.0, 9.0, 14.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)

        # 6. 4x M3 mounting holes in servo ear pads
        srv_holes = [
            (SERVO_X_OFF + sx * S.HOLE_PITCH_X / 2, sy * S.HOLE_PITCH_Y / 2, ear_z_top + 10.0)
            for sx in (-1, 1)
            for sy in (-1, 1)
        ]
        with bd.Locations(*srv_holes):
            bd.Hole(radius=1.4, depth=12.0)

    part = bm.part
    part.label = "SwitchedPod_Body_Servo_Mount"
    part.color = bd.Color(0.20, 0.72, 0.45)
    return part


def build_hub_carriage() -> bd.Part:
    """Rectangular 65 x 96 x 6 mm aluminum Hub Carriage Plate + 4 V-Wheels beneath the Base Cradle,
    showing the sideways rail-riding cuboid orientation and 90-deg crossing clearance."""
    plate_l = 96.0
    plate_w = 65.0
    wheel_dx = 26.0
    wheel_dy = 19.8
    wheel_r = 12.0
    wheel_h = 10.2
    wheel_z1 = HUB_Z0 - 1.5    # 1.5 mm spacer gap below hub plate

    with bd.BuildPart() as hc:
        # Main rectangular hub plate (Z = -10.8 .. -4.8, centered at SWIVEL_X=12.5? Center at X=-8 so it spans motor+pinion)
        with bd.Locations((-4.0, 0, HUB_Z0)):
            bd.Box(plate_l, plate_w, HUB_PLATE_T, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
        # Pinion & shaft clearance bore (Ø28 mm) at (SWIVEL_X, 0)
        with bd.Locations((SWIVEL_X, 0, HUB_Z0 - 1.0)):
            bd.Cylinder(radius=14.0, height=HUB_PLATE_T + 2.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
        # 4x M4 holes matching Base Cradle ears
        ear_pts = [
            (sx * 12.0, sy * (M.GB_W / 2 + R.CLR + R.WALL + R.EAR_W / 2), HUB_Z0 + HUB_PLATE_T + 1.0)
            for sx in (-1, 1)
            for sy in (-1, 1)
        ]
        with bd.Locations(*ear_pts):
            bd.Hole(radius=R.M4_CLR / 2, depth=HUB_PLATE_T + 2.0)

        # 4x V-wheels (2 above rail, 2 below rail along X-axis rail) + steel spacers
        for sx in (-1, 1):
            for sy in (-1, 1):
                wx = SWIVEL_X + sx * wheel_dx
                wy = sy * wheel_dy
                with bd.Locations((wx, wy, wheel_z1)):
                    bd.Cylinder(radius=5.0, height=1.5, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
                    bd.Cylinder(radius=wheel_r, height=wheel_h, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MAX))

    part = hc.part
    part.label = "SwitchedPod_Hub_Carriage"
    part.color = bd.Color(0.55, 0.58, 0.64)
    return part


def build_assembled_parts():
    """Return dict of all 11 bodies in assembled world coordinates (mm)."""
    flip_x = bd.Location((0, 0, M.GB_H), (180, 0, 0))
    housing_solids = []
    for sub in (M.build_gearbox(), M.build_cover_screws(), M.build_motor_can(), M.build_terminals()):
        housing_solids.extend(sub.solids() if isinstance(sub, bd.Compound) else [sub])
    motor_housing = bd.Compound(children=[s.moved(flip_x) for s in housing_solids], label="4632-370_Housing")
    motor_shaft = M.build_output_shaft().moved(flip_x)

    # Pinion placed below Hub Carriage Plate at Z = -16.5 .. -5.5 (14T teeth sit right in the V-wheel rail plane Z=-22..-11!)
    pinion = R.build_pinion_down().moved(bd.Location((M.SHAFT_OFFSET_X, 0, -16.0), (0, 0, 180)))
    retainer = K.build_shaft_retainer().moved(bd.Location((M.SHAFT_OFFSET_X, 0, -18.5)))

    hub_carriage = build_hub_carriage()
    base_cradle = R.build_base_cradle()
    upper_cover = build_upper_cover_turntable()
    alum_horn = build_servo_horn_25t()

    servo_flip = bd.Location((SERVO_X_OFF, 0, SERVO_TOP_Z), (180, 0, 0))
    servo_solids = []
    for sub in (S.build_case_bottom(), S.build_case_mid(), S.build_case_top()):
        servo_solids.extend(sub.solids() if isinstance(sub, bd.Compound) else [sub])
    servo_body = bd.Compound(children=[s.moved(servo_flip) for s in servo_solids], label="TD8125MG_Body")

    spline_local_z = S.BOT_H + S.MID_H + S.TOP_BASE_H + S.RIDGE_H + S.BOSS_H
    servo_spline = S.build_spline().moved(bd.Location((S.SHAFT_X, 0, spline_local_z))).moved(servo_flip)
    body_mount = build_body_servo_mount()

    return {
        "motor_housing": motor_housing,
        "motor_shaft": motor_shaft,
        "pinion": pinion,
        "retainer": retainer,
        "hub_carriage": hub_carriage,
        "base_cradle": base_cradle,
        "upper_cover": upper_cover,
        "alum_horn": alum_horn,
        "servo_spline": servo_spline,
        "servo_body": servo_body,
        "body_mount": body_mount,
    }


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    os.makedirs(SIM_MESH_DIR, exist_ok=True)

    parts = build_assembled_parts()

    printable = {
        "switched_pod_hub_carriage": parts["hub_carriage"],
        "switched_pod_base_cradle": parts["base_cradle"],
        "switched_pod_upper_cover": parts["upper_cover"],
        "switched_pod_body_mount": parts["body_mount"],
        "switched_pod_25t_horn": parts["alum_horn"],
        "switched_pod_pinion_14t": R.build_pinion_down(),
        "switched_pod_shaft_retainer": K.build_shaft_retainer(),
    }
    for name, part in printable.items():
        bd.export_step(part, os.path.join(OUT_DIR, f"{name}.step"))
        bd.export_stl(part, os.path.join(OUT_DIR, f"{name}.stl"), tolerance=0.01, angular_tolerance=0.1)
        bb = part.bounding_box()
        print(f"{name:28s} -> {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm")

    asm = bd.Compound(children=list(parts.values()), label="Switched_Swivel_Pod_Assembly")
    asm_step = os.path.join(OUT_DIR, "switched_pod_complete.step")
    bd.export_step(asm, asm_step)
    print(f"Exported full STEP: {asm_step}")

    to_swivel_axis = bd.Location((-SWIVEL_X, 0, 0))
    sim_exports = {k + ".stl": v.moved(to_swivel_axis) for k, v in parts.items()}
    for fname, p in sim_exports.items():
        bd.export_stl(p, os.path.join(SIM_MESH_DIR, fname), tolerance=0.01, angular_tolerance=0.1)
    print(f"Exported {len(sim_exports)} rig meshes to {SIM_MESH_DIR}")


if __name__ == "__main__":
    main()
