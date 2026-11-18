"""Exact B-rep CAD models for the bought OpenBuilds MT0134 Gantry Plate Set,
Delrin V-Wheels + 6 mm Spacers + 1 mm Shims, and CNC4551 2020 T-Slot Extrusion Rail.

All dimensions taken directly from the product drawings in the user's specs:
  1. OpenBuilds (Type B) V Gantry Plate (MT0134):
       65.50 x 65.50 x 3.0 mm, R3.0 corners, 3x Ø7.20 eccentric holes, 12x Ø5.10 holes,
       2x 20x5 mm center slots, 6x 6.5x3 mm outer slots.
  2. Delrin V-Wheel (A1):
       Ø24.0 mm OD x 10.95 mm width, Ø5.0 mm bore, 6.0 mm spacer + 1.0 mm shim (7.0 mm standoff),
       M5 x 25 low-profile screw + M5 nyloc nut.
  3. 2020 T-Slot Aluminium Extrusion (CNC4551):
       20.0 x 20.0 mm, R1.5 corners, Ø5.1 mm center bore, 6.2 mm slot opening, 11.0 mm inner T-slot.
"""

import os
import build123d as bd

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.abspath(os.path.join(HERE, "..", "models", "components", "openbuilds_t_slot"))

# --- MT0134 Gantry Plate (mm) ---
PLATE_SIZE = 65.50
PLATE_T = 3.0
PLATE_R = 3.0
WHEEL_Y = 19.85            # 32.75 - 12.90 = 19.85 mm across rail
WHEEL_X = 20.00            # 32.75 - 12.75 = 20.00 mm along rail

# --- Delrin V-Wheel + Spacer + Shim (mm) ---
WHEEL_OD = 24.0
WHEEL_W = 10.95
SPACER_H = 6.0
SHIM_H = 1.0
STANDOFF_H = SPACER_H + SHIM_H  # 7.0 mm from plate bottom to wheel face
RAIL_CENTER_DROP = STANDOFF_H + WHEEL_W / 2.0  # 12.475 mm below plate bottom

# --- 2020 T-Slot Extrusion (mm, exact blueprint dimensions) ---
RAIL_W = 20.0
RAIL_R = 1.50
MOUTH_W = 7.00             # outer step mouth width
MOUTH_D = 0.50             # outer step depth
SLOT_OPEN = 6.00           # inner neck width (6.00 ± 0.30)
FLANGE_T = 1.50            # total lip thickness (0.50 mouth + 1.00 neck)
SLOT_INNER = 11.90         # inner T-pocket width
POCKET_SIDE_Z = 4.05       # outer face to start of 45-deg bevel
SLOT_DEPTH = 7.00          # total depth from outer face (4.05 + 2.95)
CORE_D = 5.48              # center bore diameter


def build_mt0134_plate(center_shaft_bore_d: float = 5.10) -> bd.Part:
    """Build the exact 65.50 x 65.50 x 3.0 mm OpenBuilds Type-B Gantry Plate (MT0134)."""
    with bd.BuildPart() as gp:
        with bd.BuildSketch():
            bd.RectangleRounded(PLATE_SIZE, PLATE_SIZE, PLATE_R)
        bd.extrude(amount=PLATE_T)

        # 3x Ø7.20 eccentric spacer holes on -Y side (y = -19.85 in rail-along-X frame)
        # Note: in Image 2, X_bp=12.90 is the eccentric column; we align rail along X,
        # so wheel-across-rail is Y = ±19.85 and wheel-along-rail is X = ±20.00!
        ecc_pts = [(x, -WHEEL_Y, PLATE_T + 1.0) for x in (-20.0, 0.0, 20.0)]
        with bd.Locations(*ecc_pts):
            bd.Hole(radius=7.20 / 2.0, depth=PLATE_T + 2.0)

        # Ø5.10 holes: 2 on -Y side (±10, -19.85), 5 on +Y side (all -20..+20, +19.85),
        # and 4 cross holes around center (±10, 0) & (0, ±10)
        holes_51 = (
            [(x, -WHEEL_Y, PLATE_T + 1.0) for x in (-10.0, 10.0)]
            + [(x, WHEEL_Y, PLATE_T + 1.0) for x in (-20.0, -10.0, 0.0, 10.0, 20.0)]
            + [(-10.0, 0.0, PLATE_T + 1.0), (10.0, 0.0, PLATE_T + 1.0),
               (0.0, -10.0, PLATE_T + 1.0), (0.0, 10.0, PLATE_T + 1.0)]
        )
        with bd.Locations(*holes_51):
            bd.Hole(radius=5.10 / 2.0, depth=PLATE_T + 2.0)

        # Center hole (5.10 mm stock, or enlarged e.g. 8.0 mm for 4632-370 Ø6 mm shaft pass-through)
        with bd.Locations((0.0, 0.0, PLATE_T + 1.0)):
            bd.Hole(radius=center_shaft_bore_d / 2.0, depth=PLATE_T + 2.0)

        # 2x long inner slots (20.0 x 5.0 mm, along Y at X = ±20.0)
        for sx in (-20.0, 20.0):
            with bd.Locations((sx, 0.0, -0.5)):
                with bd.BuildSketch():
                    bd.SlotOverall(20.0, 5.0, rotation=90)
                bd.extrude(amount=PLATE_T + 1.0, mode=bd.Mode.SUBTRACT)

        # 6x outer edge slots (6.5 x 3.0 mm, along Y at X = ±27.70)
        for sx in (-27.70, 27.70):
            for sy in (-WHEEL_Y, 0.0, WHEEL_Y):
                with bd.Locations((sx, sy, -0.5)):
                    with bd.BuildSketch():
                        bd.SlotOverall(6.5, 3.0, rotation=90)
                    bd.extrude(amount=PLATE_T + 1.0, mode=bd.Mode.SUBTRACT)

    part = gp.part
    part.label = "OpenBuilds_MT0134_Plate"
    part.color = bd.Color(0.14, 0.15, 0.17)
    return part


def build_delrin_v_wheel() -> bd.Part:
    """Build the Ø24.0 x 10.95 mm Delrin V-Wheel (centered Z = 0 .. 10.95)."""
    r_out = WHEEL_OD / 2.0        # 12.0
    r_edge = 9.45                  # beveled outer edge
    w_half = WHEEL_W / 2.0         # 5.475
    flat_half = 2.6                # center cylindrical land half-width
    r_bore = 2.5                   # Ø5.0 mm bore

    with bd.BuildPart() as vw:
        with bd.BuildSketch(bd.Plane.XZ):
            with bd.BuildLine():
                l1 = bd.Polyline(
                    (r_bore, -w_half),
                    (r_edge, -w_half),
                    (r_out, -flat_half),
                    (r_out, flat_half),
                    (r_edge, w_half),
                    (r_bore, w_half),
                    (r_bore, -w_half),
                )
            bd.make_face()
        bd.revolve(axis=bd.Axis.Z)
    part = vw.part.moved(bd.Location((0, 0, w_half)))
    part.label = "Delrin_V_Wheel_24x10.95"
    part.color = bd.Color(0.16, 0.17, 0.19)
    return part


def build_wheel_stack(is_eccentric: bool = False) -> bd.Compound:
    """Build one complete V-wheel stack (6 mm spacer + 1 mm shim + Delrin wheel + M5 bolt/nut).
    Local origin Z=0 is the bottom of the 3 mm gantry plate; wheel extends downward (-Z)."""
    spacer_r = 5.0
    with bd.BuildPart() as sp:
        # 6.0 mm spacer (Z = -6.0 .. 0.0)
        bd.Cylinder(radius=spacer_r, height=SPACER_H, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MAX))
        if is_eccentric:
            # Eccentric boss extends +2.5 mm up into the Ø7.20 plate hole
            bd.Cylinder(radius=3.55, height=2.5, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
        bd.Hole(radius=2.55, depth=SPACER_H + 3.0)
    spacer = sp.part
    spacer.color = bd.Color(0.76, 0.78, 0.80)

    with bd.BuildPart() as sh:
        with bd.Locations((0, 0, -SPACER_H)):
            bd.Cylinder(radius=5.0, height=SHIM_H, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MAX))
            bd.Hole(radius=2.55, depth=SHIM_H + 1.0)
    shim = sh.part
    shim.color = bd.Color(0.82, 0.65, 0.25)

    wheel = build_delrin_v_wheel().moved(bd.Location((0, 0, -STANDOFF_H - WHEEL_W)))

    # M5 x 25 screw head above plate (Z = +3.0 .. +5.5) + nyloc nut below wheel
    with bd.BuildPart() as hw:
        with bd.Locations((0, 0, PLATE_T)):
            bd.Cylinder(radius=4.2, height=2.2, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
        with bd.Locations((0, 0, -STANDOFF_H - WHEEL_W)):
            bd.Cylinder(radius=4.5, height=4.5, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MAX))
    hardware = hw.part
    hardware.color = bd.Color(0.35, 0.37, 0.40)

    return bd.Compound(children=[spacer, shim, wheel, hardware], label="V_Wheel_Stack")


def build_t_slot_2020(length: float = 200.0) -> bd.Part:
    """Build exact 2020 T-Slot Aluminium Extrusion with 4 symmetric T-slots and 1.59 mm diagonal X-webs:
    20.00 x 20.00 mm, R1.50 corners, 7.00x0.50 mouth, 6.00 neck, 1.50 lip,
    11.90 pocket, 1.59 mm diagonal X-webs, 7.00 total slot depth, Ø5.48 center bore."""
    h = RAIL_W / 2.0               # 10.00 mm
    web_t = 1.59                   # diagonal X-web thickness
    diag_off = web_t / (2.0 ** 0.5)  # 1.1243 mm offset from y = ±x

    y_mouth = h - MOUTH_D          # 9.50
    y_lip = h - FLANGE_T           # 8.50
    x_mouth = MOUTH_W / 2.0        # 3.50
    x_neck = SLOT_OPEN / 2.0       # 3.00
    x_pkt = SLOT_INNER / 2.0       # 5.95
    y_bev = x_pkt + diag_off       # 7.074 mm (leaves exact 1.59 mm diagonal X-webs!)
    y_bot = h - SLOT_DEPTH         # 3.00
    x_bot = y_bot - diag_off       # 1.876 mm

    slot_poly = [
        (-x_mouth, h + 0.5),
        (x_mouth, h + 0.5),
        (x_mouth, y_mouth),
        (x_neck, y_mouth),
        (x_neck, y_lip),
        (x_pkt, y_lip),
        (x_pkt, y_bev),
        (x_bot, y_bot),
        (-x_bot, y_bot),
        (-x_pkt, y_bev),
        (-x_pkt, y_lip),
        (-x_neck, y_lip),
        (-x_neck, y_mouth),
        (-x_mouth, y_mouth),
    ]

    with bd.BuildPart() as rail:
        with bd.BuildSketch(bd.Plane.YZ):
            bd.RectangleRounded(RAIL_W, RAIL_W, RAIL_R)
            # Rotate 2D coordinates explicitly for all 4 slots (0, 90, 180, 270 deg)
            for rot_pts in (
                slot_poly,
                [(-y, x) for x, y in slot_poly],
                [(-x, -y) for x, y in slot_poly],
                [(y, -x) for x, y in slot_poly],
            ):
                bd.Polygon(*rot_pts, align=None, mode=bd.Mode.SUBTRACT)
            bd.Circle(radius=CORE_D / 2.0, mode=bd.Mode.SUBTRACT)
        bd.extrude(amount=length / 2.0, both=True)

    part = rail.part
    part.label = "T_Slot_2020_Blueprint"
    part.color = bd.Color(0.80, 0.82, 0.85)
    return part


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    parts = {
        "MT0134_gantry_plate": build_mt0134_plate(5.10),
        "Delrin_V_wheel": build_delrin_v_wheel(),
        "V_wheel_stack": build_wheel_stack(False),
        "T_slot_2020_rail_200mm": build_t_slot_2020(200.0),
    }
    for name, p in parts.items():
        bd.export_step(p, os.path.join(OUT_DIR, f"{name}.step"))
        bd.export_stl(p, os.path.join(OUT_DIR, f"{name}.stl"), tolerance=0.01, angular_tolerance=0.1)
        bb = p.bounding_box()
        print(f"{name:26s} -> {bb.size.X:.2f} x {bb.size.Y:.2f} x {bb.size.Z:.2f} mm")


if __name__ == "__main__":
    main()
