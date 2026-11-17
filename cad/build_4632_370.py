"""Exact B-rep CAD model of the 4632-370 (JGY-370) 12V 100RPM worm gear motor.

Dimensions based on the standard 4632-370 / GW4632-370 / JGY-370 datasheet
and the YRK-370SH-15250 motor photos in scans/worm-gear-motors/:
  - Gearbox housing: 46.0 mm (L) x 32.0 mm (W) x 22.5 mm (H)
  - Mounting holes:  4 x M3 on a 33.0 mm x 18.0 mm pattern (raised Ø6.5 mm bosses)
  - Output shaft:    Ø6.0 mm D-shaft (5.5 mm flat), 14.0 mm protrusion, M3 axial bore
  - Motor can (370): Ø24.4 mm x 30.8 mm long + rear bearing boss & solder terminals

Outputs:
  models/components/4632-370_worm_gear_motor.step
  models/components/4632-370_worm_gear_motor.stl
"""

import os
import build123d as bd

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.abspath(os.path.join(HERE, "..", "models", "components"))
SIM_MESH_DIR = os.path.abspath(os.path.join(HERE, "..", "sim", "build", "meshes_4632_370"))

# --- Dimensional parameters (mm) ---
GB_L = 46.0
GB_W = 32.0
GB_H = 22.5

HOLE_PITCH_X = 33.0
HOLE_PITCH_Y = 18.0
BOSS_D = 6.5
BOSS_H = 1.5
M3_PILOT_D = 2.5
M3_DEPTH = 4.5

SHAFT_OFFSET_X = 12.5      # from gearbox center toward far end (+X)
SHAFT_BOSS_D = 12.0
SHAFT_BOSS_H = 1.8
SHAFT_D = 6.0
SHAFT_FLAT = 5.5           # D-flat thickness
SHAFT_L = 14.0             # protrusion above gearbox top face
SHAFT_FLAT_L = 11.0
SHAFT_AXIAL_D = 2.5        # M3 tap pilot
SHAFT_AXIAL_DEPTH = 6.0

MOTOR_D = 24.4
MOTOR_L = 30.8
ENDBELL_BOSS_D = 10.0
ENDBELL_BOSS_H = 2.2


def build_gearbox() -> bd.Part:
    """Build the cast zinc-alloy 46x32x22.5 mm worm gearbox housing."""
    with bd.BuildPart() as gb:
        # Main rectangular body centered in X/Y, Z from 0 to GB_H
        bd.Box(GB_L, GB_W, GB_H, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
        # Subtle corner fillets on vertical edges
        bd.fillet(gb.edges().filter_by(bd.Axis.Z), radius=1.2)
        # Chamfer on bottom perimeter (visible on the back face in photos)
        bd.chamfer(gb.edges().filter_by_position(bd.Axis.Z, 0, 0.1), length=0.8)

        # Bottom circular bearing boss opposite the output shaft
        with bd.Locations((SHAFT_OFFSET_X, 0, 0)):
            bd.Cylinder(radius=5.5, height=1.2, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MAX))

        # 4x M3 mounting bosses on top face (Z = GB_H)
        boss_pts = [
            (sx * HOLE_PITCH_X / 2, sy * HOLE_PITCH_Y / 2, GB_H)
            for sx in (-1, 1)
            for sy in (-1, 1)
        ]
        with bd.Locations(*boss_pts):
            bd.Cylinder(radius=BOSS_D / 2, height=BOSS_H, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
            bd.Hole(radius=M3_PILOT_D / 2, depth=M3_DEPTH + BOSS_H)

        # Output shaft bearing boss on top face (bored for Ø6.1 mm shaft clearance)
        with bd.Locations((SHAFT_OFFSET_X, 0, GB_H)):
            bd.Cylinder(
                radius=SHAFT_BOSS_D / 2,
                height=SHAFT_BOSS_H,
                align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN),
            )
        with bd.Locations((SHAFT_OFFSET_X, 0, GB_H + SHAFT_BOSS_H + 0.5)):
            bd.Hole(radius=SHAFT_D / 2 + 0.05, depth=SHAFT_BOSS_H + 1.0)

        # Center stadium-shaped raised boss on top face
        with bd.Locations((-2.5, 0, GB_H)):
            with bd.BuildSketch() as sk:
                bd.SlotOverall(15.0, 5.0, rotation=90)
            bd.extrude(amount=1.2)

    part = gb.part
    part.label = "Gearbox_4632"
    part.color = bd.Color(0.70, 0.72, 0.75)
    return part


def build_cover_screws() -> bd.Compound:
    """Build the 4 small corner cover screws on the gearbox top plate."""
    screws = []
    pts = [
        (-GB_L / 2 + 2.5, GB_W / 2 - 2.5),
        (GB_L / 2 - 2.5, GB_W / 2 - 2.5),
        (GB_L / 2 - 2.5, -GB_W / 2 + 2.5),
        (-GB_L / 2 + 12.0, -GB_W / 2 + 2.5),
    ]
    for x, y in pts:
        with bd.BuildPart() as sc:
            with bd.Locations((x, y, GB_H)):
                bd.Cylinder(radius=1.5, height=0.8, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
        s = sc.part
        s.color = bd.Color(0.82, 0.78, 0.68)
        screws.append(s)
    return bd.Compound(children=screws, label="Cover_Screws")


def build_output_shaft() -> bd.Part:
    """Build the Ø6 mm D-shaped output shaft with axial M3 bore."""
    z0 = GB_H + SHAFT_BOSS_H
    with bd.BuildPart() as sh:
        with bd.Locations((SHAFT_OFFSET_X, 0, GB_H)):
            bd.Cylinder(
                radius=SHAFT_D / 2,
                height=SHAFT_L + SHAFT_BOSS_H,
                align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN),
            )
        # Cut D-flat along the upper SHAFT_FLAT_L of the shaft
        flat_cut_y = SHAFT_FLAT - SHAFT_D / 2
        with bd.Locations((SHAFT_OFFSET_X, flat_cut_y + 2.0, z0 + SHAFT_L - SHAFT_FLAT_L)):
            bd.Box(
                SHAFT_D + 1.0,
                4.0,
                SHAFT_FLAT_L + 0.5,
                align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN),
                mode=bd.Mode.SUBTRACT,
            )
        # Axial M3 tap pilot hole at shaft tip
        with bd.Locations((SHAFT_OFFSET_X, 0, GB_H + SHAFT_L + SHAFT_BOSS_H)):
            bd.Hole(radius=SHAFT_AXIAL_D / 2, depth=SHAFT_AXIAL_DEPTH)

    part = sh.part
    part.label = "Output_Shaft_6mm_D"
    part.color = bd.Color(0.82, 0.84, 0.86)
    return part


def build_motor_can() -> bd.Part:
    """Build the 370 brushed DC motor cylinder and rear bearing boss."""
    cx = -GB_L / 2
    cz = GB_H / 2
    with bd.BuildPart() as mc:
        with bd.Locations(bd.Location((cx, 0, cz), (0, -90, 0))):
            bd.Cylinder(
                radius=MOTOR_D / 2,
                height=MOTOR_L,
                align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN),
            )
        # Rear bearing boss
        with bd.Locations(bd.Location((cx - MOTOR_L, 0, cz), (0, -90, 0))):
            bd.Cylinder(
                radius=ENDBELL_BOSS_D / 2,
                height=ENDBELL_BOSS_H,
                align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN),
            )

    part = mc.part
    part.label = "Motor_370_Can"
    part.color = bd.Color(0.76, 0.78, 0.80)
    return part


def build_motor_cover() -> bd.Part:
    """Translucent PETG shell fitted over the gearbox body."""
    WALL = 1.8
    OW = GB_L + 2 * WALL
    OD = GB_W + 2 * WALL
    OZ = GB_H + WALL
    with bd.BuildPart() as cv:
        bd.Box(OW, OD, OZ, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
        bd.Box(GB_L, GB_W, OZ - WALL,
               align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN),
               mode=bd.Mode.SUBTRACT)
        with bd.Locations((SHAFT_OFFSET_X, 0, OZ)):
            bd.Hole(radius=SHAFT_BOSS_D / 2 + 2.0, depth=WALL + 0.5)
    part = cv.part
    part.label = "Motor_Cover_PETG"
    part.color = bd.Color(0.18, 0.55, 0.88)
    return part


def build_bottom_cradle() -> bd.Part:
    """PETG base plate below gearbox (z from -3 to 0 in housing frame)."""
    H = 3.0
    with bd.BuildPart() as cr:
        bd.Box(GB_L, GB_W, H,
               align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MAX))
        boss_pts = [
            (sx * HOLE_PITCH_X / 2, sy * HOLE_PITCH_Y / 2, 0)
            for sx in (-1, 1)
            for sy in (-1, 1)
        ]
        with bd.Locations(*boss_pts):
            bd.Hole(radius=M3_PILOT_D / 2, depth=H + 0.5)
    part = cr.part
    part.label = "Bottom_Cradle"
    part.color = bd.Color(0.22, 0.25, 0.30)
    return part


def build_climber_pinion() -> bd.Part:
    """14T module-1 climbing pinion (D-bore 6mm), centered at shaft-body origin."""
    PITCH_R = 7.0        # module 1, 14T → pitch radius = 7mm
    OD_R = PITCH_R + 1.0
    FACE = 8.0
    with bd.BuildPart() as pin:
        bd.Cylinder(radius=OD_R, height=FACE,
                    align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.CENTER))
        bd.Cylinder(radius=SHAFT_D / 2, height=FACE + 1.0,
                    align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.CENTER),
                    mode=bd.Mode.SUBTRACT)
    part = pin.part
    part.label = "Climber_Pinion_14T"
    part.color = bd.Color(0.92, 0.48, 0.14)
    return part


def build_shaft_retainer() -> bd.Part:
    """M3 axial cap retainer at shaft tip, centered at shaft-body origin."""
    R, H = 4.5, 2.5
    with bd.BuildPart() as ret:
        bd.Cylinder(radius=R, height=H,
                    align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.CENTER))
        bd.Cylinder(radius=M3_PILOT_D / 2, height=H + 0.5,
                    align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.CENTER),
                    mode=bd.Mode.SUBTRACT)
    part = ret.part
    part.label = "Shaft_Retainer"
    part.color = bd.Color(0.25, 0.27, 0.30)
    return part


def build_terminals() -> bd.Compound:
    """Build the 2 rear copper solder terminals."""
    tx = -GB_L / 2 - MOTOR_L
    cz = GB_H / 2
    tabs = []
    for sy in (-1, 1):
        with bd.BuildPart() as tb:
            with bd.Locations((tx, sy * 6.5, cz - sy * 4.5)):
                bd.Box(3.5, 2.2, 0.5, align=(bd.Align.MAX, bd.Align.CENTER, bd.Align.CENTER))
        t = tb.part
        t.color = bd.Color(0.85, 0.55, 0.25)
        tabs.append(t)
    return bd.Compound(children=tabs, label="Terminals")


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    os.makedirs(SIM_MESH_DIR, exist_ok=True)

    gearbox = build_gearbox()
    screws = build_cover_screws()
    shaft = build_output_shaft()
    motor_can = build_motor_can()
    terminals = build_terminals()

    parts = [gearbox, screws, shaft, motor_can, terminals]
    asm = bd.Compound(children=parts, label="4632-370_Worm_Gear_Motor")

    step_path = os.path.join(OUT_DIR, "4632-370_worm_gear_motor.step")
    stl_path = os.path.join(OUT_DIR, "4632-370_worm_gear_motor.stl")

    bd.export_step(asm, step_path)
    bd.export_stl(asm, stl_path, tolerance=0.01, angular_tolerance=0.1)
    bb = asm.bounding_box()

    # Rigged split meshes for MuJoCo / Three.js
    housing_asm = bd.Compound(children=[gearbox, screws, motor_can, terminals], label="Housing")
    shaft_local = shaft.moved(bd.Location((-SHAFT_OFFSET_X, 0, -GB_H)))

    housing_stl = os.path.join(SIM_MESH_DIR, "housing.stl")
    shaft_stl = os.path.join(SIM_MESH_DIR, "shaft.stl")

    bd.export_stl(housing_asm, housing_stl, tolerance=0.01, angular_tolerance=0.1)
    bd.export_stl(shaft_local, shaft_stl, tolerance=0.01, angular_tolerance=0.1)

    print(f"Exported STEP: {step_path}")
    print(f"Exported STL:  {stl_path}")
    print(f"Exported rig:  {housing_stl}, {shaft_stl}")
    print(
        f"Overall bounding box: {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm"
    )


if __name__ == "__main__":
    main()
