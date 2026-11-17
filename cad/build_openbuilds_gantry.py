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

# --- 2020 T-Slot Extrusion (mm) ---
RAIL_W = 20.0
RAIL_R = 1.5
SLOT_OPEN = 6.2
SLOT_INNER = 11.0
CORE_D = 5.1


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
    """Build exact 2020 T-Slot Aluminium Extrusion (CNC4551) centered at (Y=0, Z=0), running along X."""
    h = RAIL_W / 2.0               # 10.0 mm
    lip = 1.8                      # outer T-slot lip thickness
    pocket_d = 5.8                 # slot depth from outer face (leaving 8.4 mm core)

    with bd.BuildPart() as rail:
        with bd.BuildSketch(bd.Plane.YZ):
            bd.RectangleRounded(RAIL_W, RAIL_W, RAIL_R)
            # Subtract 4 T-slots (one on each face ±Y, ±Z)
            for rot in (0, 90, 180, 270):
                with bd.Locations(bd.Location((0, 0), rot)):
                    # Outer 6.2 mm neck (from y = h - lip to h + 0.5)
                    with bd.Locations((0, h - lip / 2.0 + 0.25)):
                        bd.Rectangle(SLOT_OPEN, lip + 0.6, mode=bd.Mode.SUBTRACT)
                    # Inner 11.0 mm T-pocket (from y = h - pocket_d to h - lip)
                    with bd.Locations((0, h - (pocket_d + lip) / 2.0)):
                        bd.Rectangle(SLOT_INNER, pocket_d - lip, mode=bd.Mode.SUBTRACT)
            # Center Ø5.1 mm bore
            bd.Circle(radius=CORE_D / 2.0, mode=bd.Mode.SUBTRACT)
        bd.extrude(amount=length, both=True)

    part = rail.part
    part.label = "T_Slot_2020_CNC4551"
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
