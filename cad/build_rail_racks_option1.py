"""Compact 2-Bay 2020 V-Slot Rail Cut Pieces & Printable MOD=1.5 In-Slot Rack Strips.

Generates all individual printable in-slot gear rack STLs (split at exact tooth-valley
centers so every strip fits <= 190 mm on a 220x220 / 256x256 3D printer bed with continuous
4.7124 mm pitch across joints) plus B-rep specs for the matching 2020 V-slot aluminium rail cuts
used by `cad/build_option1_full.py` and `cad/build_forklift_robot_full.py`.

Usage:
  .pixi/envs/default/bin/python cad/build_rail_racks_option1.py
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
GEN_DIR = os.path.abspath(os.path.join(HERE, "..", "generator"))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
if GEN_DIR not in sys.path:
    sys.path.insert(0, GEN_DIR)

import build123d as bd                  # noqa: E402
import build_option1_crossing as XC     # noqa: E402
import build_option1_dual_legs as DL    # noqa: E402
import build_option1_full as OF         # noqa: E402

OUT_DIR = os.path.abspath(os.path.join(HERE, "..", "models", "components", "option1_rails"))


def _flat_rack(x0: float, x1: float) -> bd.Shape:
    """Build in-slot rack comb segment centered in X/Y on Z = 0..5 mm (teeth +Z)."""
    sh = XC.comb(x0, x1)
    bb = sh.bounding_box()
    cx = 0.5 * (bb.min.X + bb.max.X)
    cy = 0.5 * (bb.min.Y + bb.max.Y)
    return sh.moved(bd.Location((-cx, -cy, -bb.min.Z)))


def _centered_rail(length_mm: float) -> bd.Shape:
    """Build centered 2020 V-slot extrusion bar (X along length, Y/Z = 20x20 mm)."""
    sh = XC.rail(0.0, length_mm)
    bb = sh.bounding_box()
    cx = 0.5 * (bb.min.X + bb.max.X)
    cy = 0.5 * (bb.min.Y + bb.max.Y)
    cz = 0.5 * (bb.min.Z + bb.max.Z)
    return sh.moved(bd.Location((-cx, -cy, -cz)))


def get_rail_rack_specs() -> list[dict]:
    """Return catalog + B-rep shapes for printable in-slot racks and 2020 cut rails (2-Bay Forklift frame)."""
    P = DL.P
    G_HALF = DL.G_HALF
    vmb_len = (DL.JZ_UP - G_HALF) - (DL.JZ_LO + G_HALF)          # 69.69 mm
    vup_len = DL.RAIL_TOP - (DL.JZ_UP + G_HALF)                  # 664.35 mm
    hl_len = (-G_HALF) - DL.LEFT_END_X                           # 94.68 mm
    hb_len = (OF.BAY_P_FULL - G_HALF) - G_HALF                   # 187.50 mm (358.14 mm bay pitch)
    hr_len = OF.RIGHT_END_X_FULL - (OF.BAY_P_FULL + G_HALF)      # 264.68 mm

    vup_z0 = (DL.JZ_UP + G_HALF) - DL.JZ_LO                      # 325.655 mm
    vup_z1 = DL.RAIL_TOP - DL.JZ_LO                              # 990.0 mm
    hr_x0 = OF.BAY_P_FULL + G_HALF                               # 443.465 mm
    hr_x1 = OF.RIGHT_END_X_FULL                                  # 708.142 mm

    rack_specs = [
        {"id": "RK-VMB", "mark": "RK-VMB", "short_title": "Vert Mid Bridge Rack",
         "name": "In-Slot Rack Strip: Vertical Middle Bridge (Z=175.3..245.0)",
         "process": "3D PRINT (FDM)", "qty": 2, "color": "#F59E0B",
         "material": "PETG (0.12-0.15mm, 100% infill, flat teeth +Z)",
         "notes": "Slides between lower (Z=90) and upper (Z=330.3) crossings; 1 drop CA glue.",
         "rel_path": "models/components/option1_rails/RK_VMB_vert_mid_rack.stl",
         "shape": _flat_rack(G_HALF, DL.JZ_UP - DL.JZ_LO - G_HALF),
         "key_dims": [f"Length: {vmb_len:.2f} mm", "Section: 10.4 x 5.0 mm (MOD=1.5)", "Fits 69.69 mm vertical 2020 bridge"]},

        {"id": "RK-VUP-1", "mark": "RK-VUP-1", "short_title": "Vert Climb Rack (Bottom)",
         "name": "In-Slot Rack Strip: Vertical Climb Start (Above H1 Crossing)",
         "process": "3D PRINT (FDM)", "qty": 2, "color": "#F97316",
         "material": "PETG (0.12-0.15mm, 100% infill, flat teeth +Z)",
         "notes": "Bottom strip starting 1.0 mm above upper crossing up to valley 105*P.",
         "rel_path": "models/components/option1_rails/RK_VUP_1_climb_bot.stl",
         "shape": _flat_rack(vup_z0, 105 * P - 0.1),
         "key_dims": [f"Length: {(105 * P - 0.1) - vup_z0:.2f} mm", "Joins cleanly into RK-VUP-2", "1x per vertical bay climb rail"]},

        {"id": "RK-VUP-2", "mark": "RK-VUP-2", "short_title": "Vert Climb Rack (Mid x2)",
         "name": "In-Slot Rack Strip: Vertical Climb 36-Pitch Section (Print 2x/rail)",
         "process": "3D PRINT (FDM)", "qty": 4, "color": "#EA580C",
         "material": "PETG (0.12-0.15mm, 100% infill, flat teeth +Z)",
         "notes": "Symmetric 36-pitch middle section: install 2x end-to-end per vertical rail (4 total).",
         "rel_path": "models/components/option1_rails/RK_VUP_2_climb_mid_36p.stl",
         "shape": _flat_rack(105 * P + 0.1, 141 * P - 0.1),
         "key_dims": [f"Length: {36 * P - 0.2:.2f} mm (36 pitches)", "Print 4x total (2 per column)", "Zero phase shift across valley seam"]},

        {"id": "RK-VUP-3", "mark": "RK-VUP-3", "short_title": "Vert Climb Rack (Top)",
         "name": "In-Slot Rack Strip: Vertical Climb Top Section (To Z=1080 mm)",
         "process": "3D PRINT (FDM)", "qty": 2, "color": "#C2410C",
         "material": "PETG (0.12-0.15mm, 100% infill, flat teeth +Z)",
         "notes": "Top strip above 2x RK-VUP-2 up to RAIL_TOP = 1080.0 mm.",
         "rel_path": "models/components/option1_rails/RK_VUP_3_climb_top.stl",
         "shape": _flat_rack(177 * P + 0.1, vup_z1),
         "key_dims": [f"Length: {vup_z1 - (177 * P + 0.1):.2f} mm", "Completes 664.35 mm climb stack", "1x per vertical bay climb rail"]},

        {"id": "RK-HL", "mark": "RK-HL", "short_title": "Horiz Left Tail Rack",
         "name": "In-Slot Rack Strip: Horizontal Left Approach Tail",
         "process": "3D PRINT (FDM)", "qty": 2, "color": "#FBBF24",
         "material": "PETG (0.12-0.15mm, 100% infill, flat teeth +Z)",
         "notes": "Covers X = -180.0..-85.32 mm on both lower & upper horizontal rails.",
         "rel_path": "models/components/option1_rails/RK_HL_horiz_left.stl",
         "shape": _flat_rack(DL.LEFT_END_X, -G_HALF),
         "key_dims": [f"Length: {hl_len:.2f} mm", "Section: 10.4 x 5.0 mm (MOD=1.5)", "Qty 2 (H1 & H2 left approach)"]},

        {"id": "RK-HB", "mark": "RK-HB", "short_title": "Horiz Bay Span Rack",
         "name": "In-Slot Rack Strip: Horizontal Bay-to-Bay Span (358.14 Pitch)",
         "process": "3D PRINT (FDM)", "qty": 2, "color": "#D97706",
         "material": "PETG (0.12-0.15mm, 100% infill, flat teeth +Z)",
         "notes": "Spans X = 85.32..272.82 mm between Bay 0 & Bay 1 crossings (fits 220x220 bed).",
         "rel_path": "models/components/option1_rails/RK_HB_horiz_bay_span.stl",
         "shape": _flat_rack(G_HALF, OF.BAY_P_FULL - G_HALF),
         "key_dims": [f"Length: {hb_len:.2f} mm (1 piece)", "Bay pitch: 358.14 mm (Forklift)", "Note: wider dual-leg layout uses 267.61 mm"]},

        {"id": "RK-HR-1", "mark": "RK-HR-1", "short_title": "Horiz Right Overhang (1/2)",
         "name": "In-Slot Rack Strip: Horizontal Right Overhang Inner Half",
         "process": "3D PRINT (FDM)", "qty": 2, "color": "#EAB308",
         "material": "PETG (0.12-0.15mm, 100% infill, flat teeth +Z)",
         "notes": "Inner half of X = 443.46..708.14 mm overhang; splits cleanly at valley 122*P.",
         "rel_path": "models/components/option1_rails/RK_HR_1_horiz_right_a.stl",
         "shape": _flat_rack(hr_x0, 122 * P - 0.1),
         "key_dims": [f"Length: {(122 * P - 0.1) - hr_x0:.2f} mm", "Bed-friendly half #1 of 264.68 mm", "Mates directly with RK-HR-2"]},

        {"id": "RK-HR-2", "mark": "RK-HR-2", "short_title": "Horiz Right Overhang (2/2)",
         "name": "In-Slot Rack Strip: Horizontal Right Overhang Outer Half",
         "process": "3D PRINT (FDM)", "qty": 2, "color": "#CA8A04",
         "material": "PETG (0.12-0.15mm, 100% infill, flat teeth +Z)",
         "notes": "Outer half ending at X = 708.14 mm (stops flush with 264.68 mm 2020 overhang rail).",
         "rel_path": "models/components/option1_rails/RK_HR_2_horiz_right_b.stl",
         "shape": _flat_rack(122 * P + 0.1, hr_x1),
         "key_dims": [f"Length: {hr_x1 - (122 * P + 0.1):.2f} mm", "Bed-friendly half #2 of 264.68 mm", "Stops flush at right rail cap"]},
    ]

    xp_rail = bd.import_step(os.path.join(os.path.dirname(HERE), "models/components/t_slot_option1_crossing/xp_rail.step"))
    rail_cut_specs = [
        {"id": "XP-RAIL", "mark": "Crossing 2020", "short_title": "Crossing 2020 V-Rail Piece",
         "name": "2020 V-Slot Rail Segment on Turntable Crossing",
         "process": "SHOP CUT / DRILL", "qty": "1/crossing", "color": "#94A3B8",
         "material": "2020 Anodized Aluminium V-Slot",
         "notes": "Cut square to 168.65 mm; bolts onto XP-ROT disc with 2x M5 x 10 + T-nuts.",
         "rel_path": "models/components/t_slot_option1_crossing/xp_rail.stl", "shape": xp_rail,
         "key_dims": ["Section: 20.0 x 20.0 mm V-slot", f"Cut length: {2 * DL.SEG:.2f} mm (17.5P)", "1.0 mm clearance gap each end"]},

        {"id": "M-VMB", "mark": "Cut 2020 (VMB)", "short_title": "2020 Vert Middle Bridge",
         "name": "2020 V-Slot Aluminium Rail: Vertical Middle Bridge",
         "process": "SHOP CUT / DRILL", "qty": 2, "color": "#94A3B8",
         "material": "2020 Anodized Aluminium V-Slot",
         "notes": "Cut square 69.69 mm; fits vertically between H2 & H1 crossings (holes w/ RK-VMB).",
         "rel_path": "cad/build_option1_full.py#L51", "shape": _centered_rail(vmb_len),
         "key_dims": ["Section: 20.0 x 20.0 mm V-slot", f"Cut length: {vmb_len:.2f} mm", "Qty 2 (Bay 0 & Bay 1 middle)"]},

        {"id": "M-VUP", "mark": "Cut 2020 (VUP)", "short_title": "2020 Vert Upper Climb",
         "name": "2020 V-Slot Aluminium Rail: Vertical Upper Climb Mast",
         "process": "SHOP CUT / DRILL", "qty": 2, "color": "#94A3B8",
         "material": "2020 Anodized Aluminium V-Slot",
         "notes": "Cut square 664.35 mm; holds RK-VUP-1 + 2x RK-VUP-2 + RK-VUP-3.",
         "rel_path": "cad/build_option1_full.py#L51", "shape": _centered_rail(vup_len),
         "key_dims": ["Section: 20.0 x 20.0 mm V-slot", f"Cut length: {vup_len:.2f} mm", "Qty 2 (Bay 0 & Bay 1 climb)"]},

        {"id": "M-HL", "mark": "Cut 2020 (HL)", "short_title": "2020 Horiz Left Tail",
         "name": "2020 V-Slot Aluminium Rail: Horizontal Left Approach",
         "process": "SHOP CUT / DRILL", "qty": 2, "color": "#94A3B8",
         "material": "2020 Anodized Aluminium V-Slot",
         "notes": "Cut square 94.68 mm; holds RK-HL rack strip.",
         "rel_path": "cad/build_option1_full.py#L66", "shape": _centered_rail(hl_len),
         "key_dims": ["Section: 20.0 x 20.0 mm V-slot", f"Cut length: {hl_len:.2f} mm", "Qty 2 (Lower & Upper left)"]},

        {"id": "M-HB", "mark": "Cut 2020 (HB)", "short_title": "2020 Horiz Bay Span",
         "name": "2020 V-Slot Aluminium Rail: Horizontal Bay-to-Bay Span",
         "process": "SHOP CUT / DRILL", "qty": 2, "color": "#94A3B8",
         "material": "2020 Anodized Aluminium V-Slot",
         "notes": "Cut square 187.50 mm for 358.14 mm bay pitch; holds RK-HB rack strip.",
         "rel_path": "cad/build_option1_full.py#L67", "shape": _centered_rail(hb_len),
         "key_dims": ["Section: 20.0 x 20.0 mm V-slot", f"Cut length: {hb_len:.2f} mm", "Qty 2 (Lower & Upper span)"]},

        {"id": "M-HR", "mark": "Cut 2020 (HR)", "short_title": "2020 Horiz Right Overhang",
         "name": "2020 V-Slot Aluminium Rail: Horizontal Right Overhang",
         "process": "SHOP CUT / DRILL", "qty": 2, "color": "#94A3B8",
         "material": "2020 Anodized Aluminium V-Slot",
         "notes": "Cut square 264.68 mm; holds RK-HR-1 + RK-HR-2.",
         "rel_path": "cad/build_option1_full.py#L68", "shape": _centered_rail(hr_len),
         "key_dims": ["Section: 20.0 x 20.0 mm V-slot", f"Cut length: {hr_len:.2f} mm", "Qty 2 (Lower & Upper right)"]},
    ]
    return rack_specs + rail_cut_specs


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    specs = get_rail_rack_specs()
    for sp in specs:
        rel = sp["rel_path"]
        if rel.startswith("models/components/option1_rails/") and rel.endswith(".stl"):
            sh = sp["shape"]
            assert len(sh.solids()) == 1, f"{sp['id']} expected 1 solid, got {len(sh.solids())}"
            out_path = os.path.join(os.path.dirname(HERE), rel)
            bd.export_stl(sh, out_path, tolerance=0.01, angular_tolerance=0.1)
            bb = sh.bounding_box()
            print(f"  exported {sp['id']:9s} (qty {sp['qty']}) -> {rel} ({bb.size.X:.2f} x {bb.size.Y:.2f} x {bb.size.Z:.2f} mm)")


if __name__ == "__main__":
    main()
