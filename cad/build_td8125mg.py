"""Exact B-rep CAD model of the TD-8125MG 25KG Digital Servo (180 deg, PWM 500-2500 us).

Dimensions from the TD-8125MG datasheet and the photos in scans/25-kg-servo/:
  - Body:          40.0 mm (L) x 20.5 mm (W) x 40.5 mm (H, base to top of gearhead)
  - Mounting ears: 54.5 mm total span, 4 x Ø4.2 mm slotted holes at 49.5 x 10.0 mm
  - Output spline: 25T brass spline (Ø5.9 mm x 4.5 mm), centered 10.0 mm from end, M3 axial bore
  - Construction:  black bottom cap (7.5 mm) + red aluminum mid-case (20.0 mm) + black top cap (13.0 mm)

Outputs:
  models/components/TD-8125MG_servo.step
  models/components/TD-8125MG_servo.stl
  sim/build/meshes_td8125mg/{case_bottom,case_mid,case_top,spline}.stl
"""

import math
import os
import build123d as bd

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.abspath(os.path.join(HERE, "..", "models", "components"))
SIM_MESH_DIR = os.path.abspath(os.path.join(HERE, "..", "sim", "build", "meshes_td8125mg"))

# --- Dimensional parameters (mm) ---
BODY_L = 40.0
BODY_W = 20.5
BOT_H = 7.5
MID_H = 20.0
TOP_BASE_H = 10.5          # total housing = 7.5 + 20.0 + 10.5 + 2.5 (ridge) = 40.5 mm
RIDGE_H = 2.5
BOSS_D = 12.0
BOSS_H = 1.5

EAR_SPAN = 54.5
EAR_W = 20.0
EAR_T = 2.5
EAR_Z = BOT_H + MID_H      # 27.5 mm (flush with top of red middle case)
HOLE_PITCH_X = 49.5
HOLE_PITCH_Y = 10.0
HOLE_D = 4.2

SHAFT_X = 10.0             # 10.0 mm from +X end (40/2 - 10 = 10.0)
SPLINE_D = 5.9
SPLINE_H = 4.5
SPLINE_TEETH = 25


def build_case_bottom() -> bd.Part:
    """Build the black bottom cap + 4 corner screw heads + rear wire strain-relief boot."""
    with bd.BuildPart() as cb:
        bd.Box(BODY_L, BODY_W, BOT_H, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
        bd.fillet(cb.edges().filter_by(bd.Axis.Z), radius=1.2)
        bd.chamfer(cb.edges().filter_by_position(bd.Axis.Z, 0, 0.1), length=0.6)

        # Wire strain-relief boot on the -X end
        with bd.Locations((-BODY_L / 2, 0, 3.0)):
            bd.Box(4.5, 6.2, 6.5, align=(bd.Align.MAX, bd.Align.CENTER, bd.Align.MIN))

        # 4 corner screw recesses on bottom face
        pts = [(sx * (BODY_L / 2 - 2.4), sy * (BODY_W / 2 - 2.4), 0) for sx in (-1, 1) for sy in (-1, 1)]
        with bd.Locations(*pts):
            bd.Hole(radius=1.5, depth=1.2)

    part = cb.part
    part.label = "TD8125MG_Bottom_Cap"
    part.color = bd.Color(0.14, 0.15, 0.17)
    return part


def build_case_mid() -> bd.Part:
    """Build the red anodized aluminum heatsink middle case."""
    with bd.BuildPart() as cm:
        with bd.Locations((0, 0, BOT_H)):
            bd.Box(BODY_L, BODY_W, MID_H, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
        bd.fillet(cm.edges().filter_by(bd.Axis.Z), radius=1.2)

    part = cm.part
    part.label = "TD8125MG_Red_Mid_Case"
    part.color = bd.Color(0.76, 0.11, 0.14)
    return part


def build_case_top() -> bd.Part:
    """Build the black top gearhead cap, mounting ears with slotted holes, and bearing boss."""
    z_top = BOT_H + MID_H
    with bd.BuildPart() as ct:
        # Main upper cap block
        with bd.Locations((0, 0, z_top)):
            bd.Box(BODY_L, BODY_W, TOP_BASE_H, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
        bd.fillet(ct.edges().filter_by(bd.Axis.Z), radius=1.2)

        # Raised gearhead ridge on top
        with bd.Locations((1.5, 0, z_top + TOP_BASE_H)):
            bd.Box(35.0, 17.5, RIDGE_H, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))

        # Circular bearing boss around spline
        with bd.Locations((SHAFT_X, 0, z_top + TOP_BASE_H + RIDGE_H)):
            bd.Cylinder(radius=BOSS_D / 2, height=BOSS_H, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))

        # Mounting ears at Z = EAR_Z
        with bd.Locations((0, 0, EAR_Z)):
            bd.Box(EAR_SPAN, EAR_W, EAR_T, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))

        # 4x slotted mounting holes on the ears (open at outer X ends as in photos)
        for sx in (-1, 1):
            for sy in (-1, 1):
                hx = sx * HOLE_PITCH_X / 2
                hy = sy * HOLE_PITCH_Y / 2
                with bd.Locations((hx, hy, EAR_Z + EAR_T + 1.0)):
                    bd.Hole(radius=HOLE_D / 2, depth=EAR_T + 2.0)
                # Open slot channel to the outer edge
                slot_x = hx + sx * 2.5
                with bd.Locations((slot_x, hy, EAR_Z - 0.5)):
                    bd.Box(5.0, 2.8, EAR_T + 1.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)

        # Center rib on each mounting ear
        for sx in (-1, 1):
            rx = sx * (BODY_L / 2 + 2.0)
            with bd.Locations((rx, 0, EAR_Z + EAR_T)):
                bd.Box(4.0, 2.0, 3.5, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))

    part = ct.part
    part.label = "TD8125MG_Top_Cap"
    part.color = bd.Color(0.14, 0.15, 0.17)
    return part


def build_spline() -> bd.Part:
    """Build the 25T brass output spline (centered at local origin Z=0..SPLINE_H) with M3 bore."""
    r_out = SPLINE_D / 2
    r_in = r_out - 0.35
    pts = []
    for i in range(SPLINE_TEETH * 2):
        a = 2.0 * math.pi * i / (SPLINE_TEETH * 2)
        r = r_out if (i % 2 == 0) else r_in
        pts.append((r * math.cos(a), r * math.sin(a)))

    with bd.BuildPart() as sp:
        with bd.BuildSketch():
            bd.Polygon(*pts, align=None)
        bd.extrude(amount=SPLINE_H)
        with bd.Locations((0, 0, SPLINE_H + 0.5)):
            bd.Hole(radius=1.25, depth=SPLINE_H + 1.0)

    part = sp.part
    part.label = "TD8125MG_25T_Spline"
    part.color = bd.Color(0.83, 0.68, 0.32)
    return part


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    os.makedirs(SIM_MESH_DIR, exist_ok=True)

    c_bot = build_case_bottom()
    c_mid = build_case_mid()
    c_top = build_case_top()
    spline_local = build_spline()

    spline_z = BOT_H + MID_H + TOP_BASE_H + RIDGE_H + BOSS_H
    spline_placed = spline_local.moved(bd.Location((SHAFT_X, 0, spline_z)))

    asm = bd.Compound(children=[c_bot, c_mid, c_top, spline_placed], label="TD-8125MG_Servo")
    step_path = os.path.join(OUT_DIR, "TD-8125MG_servo.step")
    stl_path = os.path.join(OUT_DIR, "TD-8125MG_servo.stl")

    bd.export_step(asm, step_path)
    bd.export_stl(asm, stl_path, tolerance=0.01, angular_tolerance=0.1)
    bb = asm.bounding_box()

    # Export split meshes for interactive 3D rigging
    for fname, part in [
        ("case_bottom.stl", c_bot),
        ("case_mid.stl", c_mid),
        ("case_top.stl", c_top),
        ("spline.stl", spline_local),
    ]:
        bd.export_stl(part, os.path.join(SIM_MESH_DIR, fname), tolerance=0.01, angular_tolerance=0.1)

    print(f"Exported STEP: {step_path}")
    print(f"Exported STL:  {stl_path}")
    print(f"Overall bounding box: {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm")


if __name__ == "__main__":
    main()
