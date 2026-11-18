"""3D-printable rail-drive kit for the 4632-370 (JGY-370) worm gear motor:
  1. Plastic protective cover + mounting flange (PETG, 2.4 mm walls)
  2. Rotating head: 14T involute climber pinion (MOD=1.5, PA=20 deg, 5 mm face + 6 mm hub)
     matching RackLift5's in-slot rack, with D-bore + radial M3 set-screw hole
  3. Axial M3 shaft retainer cap (locks pinion onto the 4632-370 shaft tip)

Outputs:
  models/components/4632-370_cover.step / .stl
  models/components/4632-370_climber_pinion.step / .stl
  models/components/4632-370_shaft_retainer.step / .stl
  sim/build/meshes_4632_370/{cover,pinion,retainer}.stl
"""

import math
import os
import build123d as bd

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.abspath(os.path.join(HERE, "..", "models", "components"))
SIM_MESH_DIR = os.path.abspath(os.path.join(HERE, "..", "sim", "build", "meshes_4632_370"))

# Match build_4632_370.py
GB_L = 46.0
GB_W = 32.0
GB_H = 22.5
HOLE_PITCH_X = 33.0
HOLE_PITCH_Y = 18.0
SHAFT_OFFSET_X = 12.5
SHAFT_D = 6.0
SHAFT_FLAT = 5.5
MOTOR_D = 24.4
MOTOR_L = 30.8

# Cover parameters (mm)
WALL = 2.4
CLR = 1.0                  # clearance around gearbox & motor can
M3_CLR = 3.4               # M3 clearance hole
M4_CLR = 4.5               # M4 robot frame mounting hole
EAR_W = 10.0               # side flange ear width

# Pinion parameters (matching RackLift5 generator/params.py & parts_v2.py)
MOD = 1.5
PIN_T = 14
PA = 20.0
FACE_W = 5.0               # runs inside 6.2 mm V-slot opening
HUB_H = 6.0
HUB_D = 14.0
BORE_CLR = 0.24            # print clearance on 6.0 mm D-shaft


def involute_points(z=PIN_T, m=MOD, pa=PA, n_inv=10, n_arc=4):
    """Return 2D (x, y) polygon points for a standard involute spur gear."""
    rp = m * z / 2.0
    rb = rp * math.cos(math.radians(pa))
    ra = rp + m
    rf = rp - 1.25 * m
    inv = lambda a: math.tan(a) - a
    a0 = math.radians(pa)

    def th(r):
        ar = math.acos(min(1.0, rb / r))
        return math.pi / (2.0 * z) + inv(a0) - inv(ar)

    pts = []
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


def d_bore_points(d=SHAFT_D + BORE_CLR, flat=SHAFT_FLAT + BORE_CLR, n=40):
    """Return 2D (x, y) points for a D-shaped shaft bore."""
    r = d / 2.0
    y_flat = flat - d / 2.0
    pts = []
    for i in range(n):
        a = 2.0 * math.pi * i / n
        pts.append((r * math.cos(a), min(r * math.sin(a), y_flat)))
    return pts


def build_cover() -> bd.Part:
    """Build the 2.4 mm PETG protective cover + robot mounting flanges with exact internal pockets."""
    # Inner cavity around gearbox + motor can
    gb_x0 = -GB_L / 2 - CLR
    gb_x1 = GB_L / 2 + CLR
    gb_y0 = -GB_W / 2 - CLR
    gb_y1 = GB_W / 2 + CLR
    gb_z0 = 0.0                # sits flush on top of the bottom cradle
    gb_z1 = GB_H + 0.3         # 0.3 mm top clearance; bosses/bumps sit in dedicated pockets

    mc_r = MOTOR_D / 2 + CLR

    with bd.BuildPart() as cov:
        # Outer gearbox shell
        with bd.Locations((0, 0, gb_z0)):
            bd.Box(
                (gb_x1 - gb_x0) + 2 * WALL,
                (gb_y1 - gb_y0) + 2 * WALL,
                (gb_z1 - gb_z0) + WALL,
                align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN),
            )

        # Outer motor shroud cylinder (upper semicircle only so cover slides straight down onto motor)
        with bd.Locations(bd.Location((-GB_L / 2, 0, GB_H / 2), (0, -90, 0))):
            bd.Cylinder(
                radius=mc_r + WALL,
                height=MOTOR_L + 2.0,
                align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN),
            )
        # Cut away lower half of the motor shroud below the equator (Z < GB_H / 2)
        with bd.Locations((-GB_L / 2 - (MOTOR_L + 4.0) / 2, 0, GB_H / 2)):
            bd.Box(
                MOTOR_L + 6.0,
                2 * (mc_r + WALL + 2.0),
                GB_H / 2 + 5.0,
                align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MAX),
                mode=bd.Mode.SUBTRACT,
            )
        # Clearance notch on -Y side for the bottom cradle's 1.8 mm elastic snap lip (cut cleanly past outer wall)
        with bd.Locations((-GB_L / 2 - (MOTOR_L + 4.0) / 2, -(MOTOR_D / 2 - 0.5), GB_H / 2 - 0.5)):
            bd.Box(
                MOTOR_L + 6.0,
                WALL + CLR + 4.0,
                2.8,
                align=(bd.Align.CENTER, bd.Align.MAX, bd.Align.MIN),
                mode=bd.Mode.SUBTRACT,
            )

        # Side mounting flange ears (at bottom z = gb_z0) for bolting to robot plate
        ear_len = 36.0
        for sy in (-1, 1):
            ey = sy * (GB_W / 2 + CLR + WALL + EAR_W / 2 - 0.5)
            with bd.Locations((0, ey, gb_z0)):
                bd.Box(
                    ear_len,
                    EAR_W + 1.0,
                    3.6,
                    align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN),
                )

        # Subtract gearbox inner cavity (open at bottom for slide-over assembly)
        with bd.Locations((0, 0, gb_z0 - 1.0)):
            bd.Box(
                gb_x1 - gb_x0,
                gb_y1 - gb_y0,
                (gb_z1 - gb_z0) + 1.0,
                align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN),
                mode=bd.Mode.SUBTRACT,
            )

        # Subtract motor can inner bore (open at rear for ventilation & wires)
        with bd.Locations(bd.Location((-GB_L / 2 + 1.0, 0, GB_H / 2), (0, -90, 0))):
            bd.Cylinder(
                radius=mc_r,
                height=MOTOR_L + 5.0,
                align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN),
                mode=bd.Mode.SUBTRACT,
            )

        # Internal ceiling pockets for the 4x M3 mounting bosses (Ø7.2 x 1.6 mm)
        m3_pts = [
            (sx * HOLE_PITCH_X / 2, sy * HOLE_PITCH_Y / 2, gb_z1 - 0.1)
            for sx in (-1, 1)
            for sy in (-1, 1)
        ]
        with bd.Locations(*m3_pts):
            bd.Cylinder(
                radius=3.6,
                height=1.7,
                align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN),
                mode=bd.Mode.SUBTRACT,
            )

        # Internal ceiling pocket for the center stadium boss (16.0 x 6.0 x 1.5 mm)
        with bd.Locations((-2.5, 0, gb_z1 - 0.1)):
            with bd.BuildSketch() as sk_slot:
                bd.SlotOverall(16.0, 6.0, rotation=90)
            bd.extrude(amount=1.6, mode=bd.Mode.SUBTRACT)

        # Internal ceiling pockets for the 4x corner cover screw heads (Ø3.6 x 1.2 mm)
        screw_pts = [
            (-GB_L / 2 + 2.5, GB_W / 2 - 2.5, gb_z1 - 0.1),
            (GB_L / 2 - 2.5, GB_W / 2 - 2.5, gb_z1 - 0.1),
            (GB_L / 2 - 2.5, -GB_W / 2 + 2.5, gb_z1 - 0.1),
            (-GB_L / 2 + 12.0, -GB_W / 2 + 2.5, gb_z1 - 0.1),
        ]
        with bd.Locations(*screw_pts):
            bd.Cylinder(
                radius=1.8,
                height=1.3,
                align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN),
                mode=bd.Mode.SUBTRACT,
            )

        # Top shaft boss clearance hole (Ø14 mm)
        with bd.Locations((SHAFT_OFFSET_X, 0, gb_z1 + WALL + 1.0)):
            bd.Hole(radius=7.0, depth=WALL + 3.0)

        # 4x M3 clearance holes matching gearbox mounting pattern (33 x 18 mm)
        m3_clr_pts = [
            (sx * HOLE_PITCH_X / 2, sy * HOLE_PITCH_Y / 2, gb_z1 + WALL + 1.0)
            for sx in (-1, 1)
            for sy in (-1, 1)
        ]
        with bd.Locations(*m3_clr_pts):
            bd.Hole(radius=M3_CLR / 2, depth=WALL + 3.0)

        # 4x M4 mounting holes on the side flange ears
        ear_pts = [
            (sx * 12.0, sy * (GB_W / 2 + CLR + WALL + EAR_W / 2), gb_z0 + 5.0)
            for sx in (-1, 1)
            for sy in (-1, 1)
        ]
        with bd.Locations(*ear_pts):
            bd.Hole(radius=M4_CLR / 2, depth=6.0)

    part = cov.part
    part.label = "4632-370_Cover"
    part.color = bd.Color(0.18, 0.55, 0.88)
    return part


def build_climber_pinion() -> bd.Part:
    """Build the 14T involute climber pinion (5 mm face + 6 mm hub, D-bore + M3 set screw)."""
    gear_pts = involute_points()
    bore_pts = d_bore_points()

    with bd.BuildPart() as pin:
        # Extrude involute gear teeth (Z = 0 .. FACE_W)
        with bd.BuildSketch() as sk_gear:
            bd.Polygon(*gear_pts, align=None)
            bd.Polygon(*bore_pts, align=None, mode=bd.Mode.SUBTRACT)
        bd.extrude(amount=FACE_W)

        # Add 6 mm cylindrical hub (Z = FACE_W .. FACE_W + HUB_H)
        with bd.Locations((0, 0, FACE_W)):
            with bd.BuildSketch() as sk_hub:
                bd.Circle(radius=HUB_D / 2)
                bd.Polygon(*bore_pts, align=None, mode=bd.Mode.SUBTRACT)
            bd.extrude(amount=HUB_H)

        # Radial M3 set-screw pilot hole into the D-flat (+Y side of hub)
        with bd.Locations(bd.Location((0, HUB_D / 2 + 1.0, FACE_W + HUB_H / 2), (90, 0, 0))):
            bd.Hole(radius=1.25, depth=HUB_D / 2)

    part = pin.part
    part.label = "4632-370_Climber_Pinion_14T"
    part.color = bd.Color(0.92, 0.48, 0.14)
    return part


def build_bottom_cradle() -> bd.Part:
    """Build the 3.0 mm bottom mounting cradle with a Ø13 mm circular pocket for the gearbox bottom bump
    plus a semicircle U-saddle under the 370 motor can (with a 1.8 mm elastic snap-fit lip on one side)."""
    cradle_l = GB_L + 2 * CLR + 2 * WALL
    cradle_w = GB_W + 2 * CLR + 2 * WALL + 2 * EAR_W
    cradle_t = 3.0
    saddle_l = MOTOR_L - 2.0
    saddle_w = MOTOR_D + 2 * WALL
    saddle_x = -GB_L / 2 - CLR - WALL - saddle_l / 2 + 1.0
    saddle_top_z = GB_H / 2        # exact semicircle equator (11.25 mm)
    snap_lip_h = 1.8               # elastic snap-fit extension on -Y side

    with bd.BuildPart() as cr:
        # Main gearbox baseplate (z = -3.0 .. 0.0)
        bd.Box(
            cradle_l,
            cradle_w,
            cradle_t,
            align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MAX),
        )
        # Solid motor-can support block extending up to the semicircle equator
        with bd.Locations((saddle_x, 0, -cradle_t)):
            bd.Box(
                saddle_l,
                saddle_w,
                saddle_top_z + cradle_t,
                align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN),
            )
        # Elastic snap-fit lip extending 1.8 mm past the equator on the -Y side
        with bd.Locations((saddle_x, -(MOTOR_D / 2 + WALL / 2), saddle_top_z)):
            bd.Box(
                saddle_l,
                WALL + 0.6,
                snap_lip_h,
                align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN),
            )
        # Curved semicircle cutout matching the 370 motor can (+0.2 mm snug elastic fit)
        with bd.Locations(bd.Location((-GB_L / 2, 0, GB_H / 2), (0, -90, 0))):
            bd.Cylinder(
                radius=MOTOR_D / 2 + 0.2,
                height=MOTOR_L + 4.0,
                align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN),
                mode=bd.Mode.SUBTRACT,
            )

        # Circular clearance pocket/hole for the Ø11 mm x 1.2 mm bottom bearing bump
        with bd.Locations((SHAFT_OFFSET_X, 0, 1.0)):
            bd.Hole(radius=6.5, depth=cradle_t + 2.0)

        # 4x M4 mounting holes aligned with the cover's side flange ears
        ear_pts = [
            (sx * 12.0, sy * (GB_W / 2 + CLR + WALL + EAR_W / 2), 1.0)
            for sx in (-1, 1)
            for sy in (-1, 1)
        ]
        with bd.Locations(*ear_pts):
            bd.Hole(radius=M4_CLR / 2, depth=cradle_t + 2.0)

    part = cr.part
    part.label = "4632-370_Bottom_Cradle"
    part.color = bd.Color(0.22, 0.25, 0.30)
    return part


def build_shaft_retainer() -> bd.Part:
    """Build the axial M3 retainer cap that bolts into the 4632-370 shaft tip."""
    with bd.BuildPart() as ret:
        bd.Cylinder(radius=5.0, height=2.5, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
        with bd.Locations((0, 0, 2.5)):
            bd.Hole(radius=M3_CLR / 2, depth=3.0)
    part = ret.part
    part.label = "4632-370_Shaft_Retainer"
    part.color = bd.Color(0.25, 0.27, 0.30)
    return part


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    os.makedirs(SIM_MESH_DIR, exist_ok=True)

    cover = build_cover()
    cradle = build_bottom_cradle()
    pinion = build_climber_pinion()
    retainer = build_shaft_retainer()

    items = [
        ("4632-370_cover", cover, "cover.stl"),
        ("4632-370_bottom_cradle", cradle, "cradle.stl"),
        ("4632-370_climber_pinion", pinion, "pinion.stl"),
        ("4632-370_shaft_retainer", retainer, "retainer.stl"),
    ]
    for stem, part, sim_name in items:
        step_p = os.path.join(OUT_DIR, f"{stem}.step")
        stl_p = os.path.join(OUT_DIR, f"{stem}.stl")
        sim_p = os.path.join(SIM_MESH_DIR, sim_name)
        bd.export_step(part, step_p)
        bd.export_stl(part, stl_p, tolerance=0.01, angular_tolerance=0.1)
        bd.export_stl(part, sim_p, tolerance=0.01, angular_tolerance=0.1)
        bb = part.bounding_box()
        print(f"{stem:28s} -> {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm")


if __name__ == "__main__":
    main()
