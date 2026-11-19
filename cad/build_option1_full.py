"""Unified Option 1 Full Robot + Complete 2-Bay 2020 Railing CAD Builder.

Combines the finalized Dual-Leg climber + crossings + 2-bay 2020 rail/rack frame
(`cad/build_option1_dual_legs.py`) with the finalized Body & Picker payload assembly
(`cad/build_option1_body.py`):
  - Full 40 x 25 x 2.0 mm hollow aluminium column (8 leg holes + 4 B01 body holes)
  - 3-zone internal PETG anti-crush plugs (H2 82..162, B01_upper 162.5..226, H1 258..338)
  - Dual Option 1 swivel pods (H1 @ Z=330.33, H2 mirrored @ Z=90.0) + P08/P09 Hall sensors
  - B01 closed-box body collar + 8x M4 double-shear wing bolts
  - B02 truss boom + sturdy sub-base + 5 kg TAL220 load cell + B05 wishbone + B04 180x175 tray
  - Left & right 2-stage telescopic picker arms + flip fingers + reference Daraz basket
  - B03 counterweight enclosure + triangular knee gusset + flush B03L lid (Z=226.0)
  - Complete 2-bay v3 2020 railing set, 4x 17.5P crossing turntables, and 5-shelf rack frame

Usage:
  pixi run python cad/build_option1_full.py
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
GEN_DIR = os.path.abspath(os.path.join(HERE, "..", "generator"))
sys.path.insert(0, HERE)
sys.path.insert(0, GEN_DIR)

import build123d as bd                      # noqa: E402
import build_t_slot_option1 as O            # noqa: E402
import build_option1_crossing as XC         # noqa: E402
import build_option1_dual_legs as DL        # noqa: E402
import build_option1_body as OB             # noqa: E402
import params_v3 as PV                      # noqa: E402

OUT_DIR = os.path.abspath(os.path.join(HERE, "..", "models", "components", "option1_full"))
SIM_MESH_DIR = os.path.abspath(os.path.join(HERE, "..", "sim", "build", "meshes_option1_full"))

# Compact prototype rack dimensions matched to trimmed B04 (180x175) & 140x130 Daraz basket:
BAY_P_FULL = 76 * DL.P                      # 358.14 mm (76 rack pitches: Bay 0 center X=180 fits 184 mm opening)
BAY_XS_FULL = (0.0, BAY_P_FULL)
RIGHT_END_X_FULL = BAY_P_FULL + 350.0       # 708.14 mm (clears Bay 1 right post at 668.14)
COMPACT_SHELF_D = 180.0                     # Y = 0 .. 180 mm compact prototype shelf depth


def build_compact_railing_and_magnets():
    """2-bay 2020 railing set + crossing magnets + compact 184 mm opening x 180 mm deep prototype racks."""
    v_rails, v_racks, h_rails, h_racks, fx_cross, cross_mags = [], [], [], [], [], []
    turning_tpl, fixed_tpl = XC.crossing_parts(DL.SEG)

    for bx in BAY_XS_FULL:
        for z0, z1 in ((DL.JZ_LO + DL.G_HALF, DL.JZ_UP - DL.G_HALF), (DL.JZ_UP + DL.G_HALF, DL.RAIL_TOP)):
            r, k = DL.build_vert_rail_and_rack(z0, z1, bx)
            v_rails.append(r)
            v_racks.append(k)
        for zc in (DL.JZ_LO, DL.JZ_UP):
            loc = DL.hub_world_loc(zc, 0.0, bx)
            for fk in ("fx_pivot_housing", "fx_housing_screws", "fx_bracket"):
                fx_cross.extend(s.moved(loc * DL.LOC_POD_TO_HUB) for s in (
                    fixed_tpl[fk].solids() if isinstance(fixed_tpl[fk], bd.Compound) else [fixed_tpl[fk]]
                ))
            for mx, my in ((-42.0, 42.0), (-42.0, -42.0)):
                m = bd.Cylinder(2.5, 1.2, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN)).moved(bd.Location((mx, my, -30.475)))
                cross_mags.append(DL.to_hub(m, loc))

    h_spans = (
        (DL.LEFT_END_X, -DL.G_HALF),
        (DL.G_HALF, BAY_P_FULL - DL.G_HALF),
        (BAY_P_FULL + DL.G_HALF, RIGHT_END_X_FULL),
    )
    for zc in (DL.JZ_LO, DL.JZ_UP):
        for x0, x1 in h_spans:
            r, k = DL.build_horiz_rail_and_rack(x0, x1, zc)
            h_rails.append(r)
            h_racks.append(k)

    # Compact slotted-angle 5-shelf bays matching B04 (184 mm clear opening, 38x38x2.5 L-angle posts, 180 mm depth)
    # Plus symmetric far-left outrigger 38x38x2.5 post (X = -88..-50) tied to Bay 0 behind xp_m8_stack (Y >= 12)
    def _l_post(px, py, z_top=1000.0, outer_side="-x"):
        """38 x 38 x 2.5 mm slotted-angle post (open L-profile so M6 nuts sit cleanly inside behind Y=2.5!)."""
        t = 2.5
        front_leg = OB._box(px, px + 38.0, py, py + t, 0.0, z_top) if py == 0.0 else OB._box(px, px + 38.0, py + 38.0 - t, py + 38.0, 0.0, z_top)
        side_x0 = px if outer_side == "-x" else (px + 38.0 - t)
        side_leg = OB._box(side_x0, side_x0 + t, py, py + 38.0, 0.0, z_top)
        return front_leg + side_leg

    rack_solids = [
        _l_post(-88.0, 0.0, 1000.0, "-x"),
        bd.Box(176.0, 26.0, 38.0, align=(bd.Align.MIN,) * 3).moved(bd.Location((-88.0, 12.0, 390.0))),
    ]
    shelf_w = 2 * (OB.RACK_HALF_OPEN + 38.0)   # 260 mm total exterior shelf width (was 382 mm)
    for bx in BAY_XS_FULL:
        bc = bx + OB.BXR                       # 180.0 and 538.14
        for px, side_dir in ((bc - OB.RACK_HALF_OPEN - 38.0, "+x"), (bc + OB.RACK_HALF_OPEN, "-x")):
            for py in (0.0, COMPACT_SHELF_D - 38.0):
                rack_solids.append(_l_post(px, py, 1000.0, side_dir))
        for z_lvl in PV.LEVELS:
            rack_solids.append(bd.Box(shelf_w, COMPACT_SHELF_D, 3.0, align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.MAX)).moved(
                bd.Location((bc, 0.0, z_lvl))))

    # Rigid steel/aluminium Rack Mounting Brackets tying fx_bracket + V/H 2020 rails to rack front posts (Y = 0)
    box = OB._box
    mounts, mount_hw = [], []
    holes_cross, holes_post = [], []

    def _hex_y(af_mm, cx, cz, y0, y1):
        with bd.BuildPart() as hp:
            with bd.Locations(bd.Location((cx, y0, cz), (-90, 0, 0))):
                with bd.BuildSketch():
                    bd.RegularPolygon(radius=af_mm / math.sqrt(3.0), side_count=6, rotation=30.0)
                bd.extrude(amount=y1 - y0)
        return hp.part

    def add_m6_hex_bolt_nut_y(cx, cz, y_head_back, y_nut_front):
        """Real DIN 933 M6 hex bolt + steel washers + DIN 934 M6 hex nut along +Y (0 interference in holes)."""
        head = _hex_y(10.0, cx, cz, y_head_back - 4.0, y_head_back)
        w1 = OB._cyl_axis(6.0, y_head_back, y_head_back + 1.2, "y", cx, cz)
        shank = OB._cyl_axis(3.0, y_head_back + 1.2, y_nut_front + 6.5, "y", cx, cz)
        w2 = OB._cyl_axis(6.0, y_nut_front, y_nut_front + 1.2, "y", cx, cz)
        nut = _hex_y(10.0, cx, cz, y_nut_front + 1.2, y_nut_front + 6.2) - OB._cyl_axis(2.5, y_nut_front + 0.5, y_nut_front + 7.0, "y", cx, cz)
        mount_hw.append(head + w1 + shank + w2 + nut)

    def add_m5_tslot_bolt_y(cx, cz, y_cap_front, tnut_loc):
        """Real DIN 912 M5 socket cap bolt + washer tightening forward (-Y) into a real 2020 drop-in M5 T-nut."""
        cap = OB._cyl_axis(4.25, y_cap_front, y_cap_front + 5.0, "y", cx, cz) - _hex_y(4.0, cx, cz, y_cap_front + 2.5, y_cap_front + 5.2)
        wash = OB._cyl_axis(5.2, y_cap_front - 1.0, y_cap_front, "y", cx, cz)
        shank = OB._cyl_axis(2.5, -42.6, y_cap_front - 1.0, "y", cx, cz)
        tn = XC.tnut(0.0, XC.Z_RB).moved(tnut_loc)
        mount_hw.append(cap + wash + shank + tn)

    # 1. Four Crossing Plate Z-Standoff Brackets (Y = -22..0, drilled Ø6.5 with real M6 hex bolts & nuts)
    for bx in BAY_XS_FULL:
        for zc in (DL.JZ_LO, DL.JZ_UP):
            # L-profile flange (-21.9..-15.9, avoiding fx_housing/stop-pin) + corner post flange (-6..-0.1) + return gusset
            br_l = (
                box(bx - 68.0, bx - 30.0, -21.9, -15.9, zc - 44.0, zc + 44.0)
                + box(bx - 52.0, bx - 48.0, -16.0, -0.1, zc - 44.0, zc + 44.0)
                + box(bx - 68.0, bx - 48.0, -6.0, -0.1, zc - 44.0, zc + 44.0)
            )
            br_r = (
                box(bx + 26.0, bx + 68.0, -21.9, -15.9, zc + 18.0, zc + 44.0)
                + box(bx + 48.0, bx + 52.0, -16.0, -0.1, zc + 18.0, zc + 44.0)
                + box(bx + 48.0, bx + 68.0, -6.0, -0.1, zc + 18.0, zc + 44.0)
            )
            for cx, cz in ((bx - 38.0, zc + 36.0), (bx - 38.0, zc - 36.0)):
                br_l = br_l - OB._cyl_axis(3.25, -24.0, -14.0, "y", cx, cz)
                holes_cross.append(OB._cyl_axis(3.25, -34.0, -20.0, "y", cx, cz))
                add_m6_hex_bolt_nut_y(cx, cz, -33.2, -15.9)
            br_r = br_r - OB._cyl_axis(3.25, -24.0, -14.0, "y", bx + 38.0, zc + 36.0)
            holes_cross.append(OB._cyl_axis(3.25, -34.0, -20.0, "y", bx + 38.0, zc + 36.0))
            add_m6_hex_bolt_nut_y(bx + 38.0, zc + 36.0, -33.2, -15.9)
            for cx, cz in ((bx - 59.0, zc + 36.0), (bx - 59.0, zc - 36.0)):
                br_l = br_l - OB._cyl_axis(3.25, -8.0, 4.0, "y", cx, cz)
                holes_post.append(OB._cyl_axis(3.25, -1.0, 4.0, "y", cx, cz))
                add_m6_hex_bolt_nut_y(cx, cz, -7.2, 2.5)
            br_r = br_r - OB._cyl_axis(3.25, -8.0, 4.0, "y", bx + 59.0, zc + 36.0)
            holes_post.append(OB._cyl_axis(3.25, -1.0, 4.0, "y", bx + 59.0, zc + 36.0))
            add_m6_hex_bolt_nut_y(bx + 59.0, zc + 36.0, -7.2, 2.5)
            mounts.extend([br_l, br_r])

    # 2. Vertical 2020 Middle Bridge (Z=210) & Upper Climb Rail Tie-Arms (Z=505, 680, 855, 960 - clear of shelf ledges)
    for bx in BAY_XS_FULL:
        for zt in (210.0, 505.0, 680.0, 855.0, 960.0):
            v_br = (
                box(bx - 6.0, bx + 6.0, -39.9, -29.9, zt - 16.0, zt + 16.0)
                + box(bx - 52.0, bx + 52.0, -30.0, -22.0, zt - 16.0, zt + 16.0)
                + box(bx - 52.0, bx - 48.0, -22.1, -0.1, zt - 16.0, zt + 16.0)
                + box(bx + 48.0, bx + 52.0, -22.1, -0.1, zt - 16.0, zt + 16.0)
                + box(bx - 68.0, bx - 48.0, -6.0, -0.1, zt - 16.0, zt + 16.0)
                + box(bx + 48.0, bx + 68.0, -6.0, -0.1, zt - 16.0, zt + 16.0)
            )
            for tz in (zt - 10.0, zt + 10.0):
                v_br = v_br - OB._cyl_axis(2.75, -42.0, -20.0, "y", bx, tz)
                tn_loc = DL.hub_world_loc(tz, 0.0, bx) * DL.LOC_POD_TO_HUB
                add_m5_tslot_bolt_y(bx, tz, -21.0, tn_loc)
            for cx in (bx - 59.0, bx + 59.0):
                v_br = v_br - OB._cyl_axis(3.25, -8.0, 4.0, "y", cx, zt)
                holes_post.append(OB._cyl_axis(3.25, -1.0, 4.0, "y", cx, zt))
                add_m6_hex_bolt_nut_y(cx, zt, -7.2, 2.5)
            mounts.append(v_br)

    # 3. Horizontal 2020 Rail Standoffs (dual M5 T-nut flange ears + rear M6 post bolts)
    for zc in (DL.JZ_LO, DL.JZ_UP):
        m_horiz = bd.Plane(origin=(0.0, -62.475, zc), x_dir=(1, 0, 0), z_dir=(0, -1, 0)).location
        for cx_clamp, arm_x0, arm_x1, foot_x0, foot_x1, stem_side_x, fx_c in (
            (105.0, 58.0, 109.8, 58.0, 88.0, 84.0, 71.0),
            (253.0, 248.2, 275.0, 271.0, 290.0, 271.0, 282.5),
            (463.0, 442.0, 467.8, 412.0, 446.0, 442.0, 427.0),
            (649.0, 636.0, 653.8, 636.0, 666.0, 636.0, 653.0),
        ):
            h_br = (
                box(cx_clamp - 17.0, cx_clamp + 17.0, -39.9, -33.9, zc - 8.0, zc + 8.0)
                + box(cx_clamp - 4.8, cx_clamp + 4.8, -34.0, -21.9, zc - 8.0, zc + 8.0)
                + box(arm_x0, arm_x1, -22.0, -15.9, zc - 8.0, zc + 8.0)
                + box(stem_side_x, stem_side_x + 4.0, -16.0, -0.1, zc - 8.0, zc + 8.0)
                + box(foot_x0, foot_x1, -6.0, -0.1, zc - 8.0, zc + 8.0)
            )
            for tx in (cx_clamp - 11.0, cx_clamp + 11.0):
                h_br = h_br - OB._cyl_axis(2.75, -42.0, -32.0, "y", tx, zc)
                tn_loc = bd.Location((tx, 0.0, 0.0)) * m_horiz
                add_m5_tslot_bolt_y(tx, zc, -32.9, tn_loc)
            h_br = h_br - OB._cyl_axis(3.25, -8.0, 4.0, "y", fx_c, zc)
            holes_post.append(OB._cyl_axis(3.25, -1.0, 4.0, "y", fx_c, zc))
            add_m6_hex_bolt_nut_y(fx_c, zc, -7.2, 2.5)
            mounts.append(h_br)

    # Drill matching M6 thru-holes in fx_bracket plates and rack front L-angle posts so hardware has 0 interference
    hole_cross_cmp = bd.Compound(children=holes_cross)
    fx_cross = [s - hole_cross_cmp if s.bounding_box().max.Y <= -21.0 and s.bounding_box().min.Y >= -33.0 else s for s in fx_cross]
    hole_post_cmp = bd.Compound(children=holes_post)
    rack_solids = [s - hole_post_cmp if s.bounding_box().min.Y >= -0.1 and s.bounding_box().max.Y <= 38.5 and s.bounding_box().max.Z >= 950.0 else s for s in rack_solids]

    env = {
        "rails_vert": bd.Compound(children=v_rails, label="Vertical_2020_Rails"),
        "racks_vert": bd.Compound(children=v_racks, label="Vertical_InSlot_Racks"),
        "rails_horiz": bd.Compound(children=h_rails, label="Horizontal_2020_Rails"),
        "racks_horiz": bd.Compound(children=h_racks, label="Horizontal_InSlot_Racks"),
        "crossings_fixed": bd.Compound(children=fx_cross, label="Crossings_Fixed_Brackets_Housings"),
        "rack_bays": bd.Compound(children=rack_solids, label="Compact_2Bay_Rack_Frames"),
        "rack_mounts": bd.Compound(children=mounts, label="Rack_Rail_And_Crossing_Standoffs"),
        "rack_mount_hw": bd.Compound(children=mount_hw, label="Rack_Mounting_Hardware"),
        "turning_tpl": turning_tpl,
    }
    return env, bd.Compound(children=cross_mags, label="Crossing_Target_Magnets_NdFeB")


def build_unified_column_and_plugs():
    """Drill B01 body-collar bolt holes into the dual-leg 40x25x2 column bar and create a
    clash-free 3-plug internal anti-crush stack (H2: 82..162, B01_upper: 162.5..226, H1: 258..338).
    """
    col = DL.build_column_bar()
    for zp in OB.BODY_COL_BOLTS_Z:
        for sx in (-12.0, 12.0):
            cyl = bd.Cylinder(
                radius=2.25, height=DL.COL_D + 2.0,
                align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN),
            ).moved(bd.Location((sx, DL.COL_Y0 - 1.0, zp), (-90, 0, 0)))
            col = col - cyl

    plug_w = DL.COL_W - 2 * DL.COL_WALL - 0.2
    plug_d = DL.COL_D - 2 * DL.COL_WALL - 0.2
    yc = (DL.COL_Y0 + DL.COL_Y1) / 2.0
    y_plug_front = yc - plug_d / 2.0

    plugs = []
    specs = (
        ("H2", DL.JZ_LO - 8.0, 80.0, (DL.JZ_LO + 62.0, DL.JZ_LO + 2.0, OB.BODY_COL_BOLTS_Z[0])),
        ("BODY", OB.BODY_Z0 + 34.5, OB.BODY_Z1 - (OB.BODY_Z0 + 34.5), (OB.BODY_COL_BOLTS_Z[1],)),
        ("H1", DL.JZ_UP - 72.0, 80.0, (DL.JZ_UP - 62.0, DL.JZ_UP - 2.0)),
    )
    for tag, z0, height, hole_zs in specs:
        with bd.BuildPart() as bp:
            with bd.Locations((0.0, yc, z0)):
                bd.Box(plug_w, plug_d, height, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
            for zp in hole_zs:
                for sx in (-12.0, 12.0):
                    with bd.Locations(bd.Location((sx, DL.COL_Y0 - 1.0, zp), (-90, 0, 0))):
                        bd.Cylinder(radius=2.25, height=DL.COL_D + 2.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN), mode=bd.Mode.SUBTRACT)
        plugs.append(O.engrave(bp.part, f"P07-{tag}", (0.0, y_plug_front, z0 + height / 2.0), (1, 0, 0), (0, -1, 0), 3.6))

    plugs_cmp = bd.Compound(children=plugs, label="Unified_Column_AntiCrush_Plugs")
    plugs_cmp.color = bd.Color(0.95, 0.58, 0.16)
    return col, plugs_cmp


def build_leg_protective_covers():
    """Stationary lamp-shade protective shrouds (World/Robot frame) anchored to BOTH the 40 x 25 x 2 mm
    column bar (front 2x M4 bolts per hood) AND directly bolted onto P04 body_mount side cheeks
    (2x DIN 912 M3 x 14 screws + M3 hex nuts per hood) for rigid perpendicular-axis bracing.
    Both P10-UP and P10-LO share identical compact 180-deg twin dimensions (dz_in=40.6, dz_out=84.5,
    swing radius=85.8, flat closed non-swinging side wall at |X|=52.2..55.4 mm, Y=-145.3..-63.0 mm)
    plus a partial horizontal dust-shield baffle slab (Y=-145.3..-74.5) shielding gears/motor from above
    while leaving the swing sector & rear rail mouth clear.
    """
    box = OB._box
    r_in, r_out = 82.6, 85.8
    y_front, y_back = -145.3, -63.0  # stops 3.0 mm strictly in front of -60.0 rails/racks
    x_flat_in, x_flat_out = 52.2, 55.4  # closed 3.2 mm flat wall on non-swinging side (clears P08 +-49.0)
    dz_in, dz_out = 40.6, 84.5          # identical 125.1 mm tall shell (Z=5.5..130.6 on H2, 289.7..414.8 on H1)
    y_ear_bolt = -94.475                # P04 cheek exterior M3 bolt centerline (X_pod = -55.0, Z_pod = 32.0)
    hood_hw_solids = []

    def _hex_x(af_mm, cy, cz, x0, x1):
        with bd.BuildPart() as hp:
            with bd.Locations(bd.Location((x0, cy, cz), (0, 90, 0))):
                with bd.BuildSketch():
                    bd.RegularPolygon(radius=af_mm / math.sqrt(3.0), side_count=6, rotation=30.0)
                bd.extrude(amount=x1 - x0)
        return hp.part

    def _add_m3_p04_bolt(sx: int, z_bolt: float):
        """Real DIN 912 M3 x 14 bolt inside P04 channel pointing outward + washer & DIN 934 M3 nut OUTSIDE hood ear."""
        x_hd_in = sx * 9.40
        x_hd_out = sx * 12.45
        x_ear_out = sx * 22.65
        x_nut_in = sx * 23.45
        x_nut_out = sx * 25.85
        x_tip = sx * 26.85
        cap = OB._cyl_axis(2.75, min(x_hd_in, x_hd_out), max(x_hd_in, x_hd_out), "x", y_ear_bolt, z_bolt) - _hex_x(
            2.5, y_ear_bolt, z_bolt, min(x_hd_in - sx * 0.2, x_hd_in + sx * 1.4), max(x_hd_in - sx * 0.2, x_hd_in + sx * 1.4)
        )
        shank = OB._cyl_axis(1.5, min(x_hd_out, x_tip), max(x_hd_out, x_tip), "x", y_ear_bolt, z_bolt)
        w_out = OB._cyl_axis(3.5, min(x_ear_out, x_nut_in), max(x_ear_out, x_nut_in), "x", y_ear_bolt, z_bolt)
        nut = _hex_x(5.5, y_ear_bolt, z_bolt, min(x_nut_in, x_nut_out), max(x_nut_in, x_nut_out)) - OB._cyl_axis(
            1.4, min(x_nut_in, x_nut_out) - 0.3, max(x_nut_in, x_nut_out) + 0.3, "x", y_ear_bolt, z_bolt
        )
        hood_hw_solids.append(cap + shank + w_out + nut)

    def _twin_hood(zc: float, swing_sign: int, col_z0: float, col_z1: float,
                   bolt_zs: tuple, cool_angles: tuple, label_name: str, lbl_pos: tuple):
        if swing_sign > 0:  # lower H2: swings +X, -Z; flat wall -X; stepped full dust-shield roof (<=131.7 under B02)
            x0, x1, xi0, xi1 = -x_flat_out, r_out, -x_flat_in, r_in
            z0, z1 = zc - dz_out, zc + dz_in
            zi0, zi1 = zc - (dz_out - 3.2), zc + dz_in + 2.0
            neck_z0, neck_z1 = zc - 14.0, zc + dz_in + 2.0
            slab_x0, slab_x1 = -x_flat_out, 27.0
            slab_z0, slab_z1 = z1 - 3.2, z1
            wing_x0, wing_x1 = 26.0, r_out
            wing_z0, wing_z1 = 129.7, 131.7
            ear_z0, ear_z1 = z1 - 1.5, 151.0
            z_p04_m3 = zc + 55.0  # 145.0 mm (completely outside drum, rear of B02/B03)
        else:               # upper H1: swings -X, +Z; flat wall +X; stepped full dust-shield floor (mirror twin)
            x0, x1, xi0, xi1 = -r_out, x_flat_out, -r_in, x_flat_in
            z0, z1 = zc - dz_in, zc + dz_out
            zi0, zi1 = zc - dz_in - 2.0, zc + (dz_out - 3.2)
            neck_z0, neck_z1 = zc - dz_in - 2.0, zc + 14.0
            slab_x0, slab_x1 = -27.0, x_flat_out
            slab_z0, slab_z1 = z0, z0 + 3.2
            wing_x0, wing_x1 = -r_out, -26.0
            wing_z0, wing_z1 = 288.63, 290.63
            ear_z0, ear_z1 = 269.33, z0 + 1.5
            z_p04_m3 = zc - 55.0  # 275.33 mm (completely outside drum)

        outer = OB._cyl_axis(r_out, y_front, y_back, "y", 0.0, zc) & box(x0, x1, y_front - 1.0, y_back + 1.0, z0, z1)
        collar = (
            box(-23.2, 23.2, -173.8, -170.6, col_z0, col_z1)
            + box(-23.2, -20.2, -173.8, y_front + 2.0, col_z0, col_z1)
            + box(20.2, 23.2, -173.8, y_front + 2.0, col_z0, col_z1)
        )
        # Full dust-shield top/bottom enclosure (Y=-145.3..-74.5): inner 3.2 mm baffle + 2.0 mm motor-cap wing roof
        dust_slab = (
            box(slab_x0, slab_x1, y_front, -74.5, slab_z0, slab_z1)
            + box(wing_x0, wing_x1, y_front, -74.5, wing_z0, wing_z1)
        ) & OB._cyl_axis(r_out, y_front - 1.0, y_back + 1.0, "y", 0.0, zc)
        # Exterior P04 mounting ear tabs extending OUTSIDE drum to P04 cheeks (X=+-19.05..+-22.6, Y=-101..-90.5)
        ear_tabs = (
            box(-22.6, -19.05, -101.0, -90.5, ear_z0, ear_z1)
            + box(19.05, 22.6, -101.0, -90.5, ear_z0, ear_z1)
        )
        h = outer + collar + dust_slab + ear_tabs
        cavity = OB._cyl_axis(r_in, y_front + 3.0, y_back + 2.0, "y", 0.0, zc) & box(
            xi0, xi1, y_front + 3.0, y_back + 2.0, zi0, zi1
        )
        h = h - cavity
        # Re-add dust_slab & ear_tabs (subtract P04/20T slide-on slot -19.4..+19.4 across Y=-146..-74)
        h = h + ((dust_slab + ear_tabs) - box(-19.4, 19.4, y_front - 1.0, -74.0, min(slab_z0, wing_z0, ear_z0) - 1.0, max(slab_z1, wing_z1, ear_z1) + 1.0))
        h = h - box(-19.6, 19.6, y_front - 1.0, y_front + 4.0, neck_z0, neck_z1)
        h = h - box(-20.2, 20.2, -170.6, -145.35, min(z0, col_z0) - 5.0, max(z1, col_z1) + 5.0)
        # The pod's own column bolts at bolt_zs (M4 x 35 + EPDM + steel washer, modelled on the bare column face): 11 mm access
        # holes through the strap so the heads + washers sit on the column, not on the strap (the pod's bolt stack is final);
        # the hood is held by its 2 x M3 cross-bolts into the P04 cheeks + the strap wrapping the column's 3 sides
        for zp in bolt_zs:
            for sx in (-12.0, 12.0):
                h = h - OB._cyl_axis(5.5, -175.0, -169.0, "y", sx, zp)
        h = h - OB._cyl_axis(1.70, -24.0, 24.0, "x", y_ear_bolt, z_p04_m3)
        for sx in (-1, 1):
            _add_m3_p04_bolt(sx, z_p04_m3)
        for ang_deg in cool_angles:
            rad = math.radians(ang_deg)
            sx, sz = 84.0 * math.cos(rad), zc + 84.0 * math.sin(rad)
            slot = box(-8.0, 8.0, -122.0, -82.0, -1.5, 1.5).moved(bd.Location((sx, 0.0, sz), (0, -ang_deg, 0)))
            h = h - slot
        return O.engrave(h, label_name, lbl_pos, (1, 0, 0), (0, -1, 0), 3.6)

    hood_up = _twin_hood(
        DL.JZ_UP, -1, DL.JZ_UP - 35.5, DL.JZ_UP + 15.0, (DL.JZ_UP - 2.0,),
        (25, 45, 65, 115), f"P10-UP-{O.BATCH}", (-44.0, y_front, DL.JZ_UP + 48.0),
    )
    hood_lo = _twin_hood(
        DL.JZ_LO, +1, 75.0, 125.5, (DL.JZ_LO + 2.0,),
        (-25, -45, -65, -115), f"P10-LO-{O.BATCH}", (44.0, y_front, DL.JZ_LO - 48.0),
    )
    hood_hw = bd.Compound(children=hood_hw_solids, label="Hood_P04_M3_Hardware")
    return hood_up, hood_lo, hood_hw


def check_full_clearances(col, plugs, body_parts: dict, env: dict, sensors: tuple, covers: tuple):
    """Verify full robot (H1/H2 legs + covers + column + plugs + B01..B05 + picker) against rails, crossings,
    and compact 2-bay slotted-angle rack bays across climbing, swiveling, bay travel, and shelf picking poses.
    """
    pod_parts = O.build_assembled_parts()
    s_fix, s_turn, s_hw, c_mags = sensors
    p10_cover, p11_cover, hood_hw = covers
    turning_tpl = env["turning_tpl"]

    turn_keys = ("gantry_plate", "v_wheels", "motor_housing", "motor_shaft", "pinion", "retainer",
                 "bearing_pinion", "carriage_pod", "motor_cap", "pinion_cover", "fasteners_turn", "screws_pcover", "screws_cap",
                 "screw_pinion")
    # (fasteners_fixed: the pods' own column bolts / nuts / washers - the body collar covered the mirrored lower pod's heads)
    fixed_keys = ("body_mount", "bearing_bot", "bearing_top", "m8_hardware", "servo_body", "fasteners_fixed", "gear_20t", "alum_horn",
                  "servo_spline", "fasteners_gear")
    body_keys = ("body_collar", "body_fasteners", "tray_boom", "live_wishbone", "load_cell",
                 "tray_deck", "tray_stops", "battery_pod", "battery_lid", "picker_servos",
                 "arms_outer", "arms_mid", "arms_in", "picker_belts")

    def _solid_boxes(shape):
        solids = shape.solids() if hasattr(shape, "solids") else [shape]
        return [(s, s.bounding_box()) for s in solids]

    def overlap(a, b):
        ba, bb = a.bounding_box(), b.bounding_box()
        if (ba.min.X > bb.max.X or bb.min.X > ba.max.X or ba.min.Y > bb.max.Y or bb.min.Y > ba.max.Y
                or ba.min.Z > bb.max.Z or bb.min.Z > ba.max.Z):
            return 0.0
        sa_list = a if isinstance(a, list) else _solid_boxes(a)
        sb_list = b if isinstance(b, list) else _solid_boxes(b)
        total = 0.0
        for sa, bba in sa_list:
            for sb, bbb in sb_list:
                if (bba.min.X > bbb.max.X or bbb.min.X > bba.max.X or bba.min.Y > bbb.max.Y or bbb.min.Y > bba.max.Y
                        or bba.min.Z > bbb.max.Z or bbb.min.Z > bba.max.Z):
                    continue
                inter = sa.intersect(sb)
                if inter is not None and hasattr(inter, "solids"):
                    total += sum(s.volume for s in inter.solids())
        return total

    def move_shape(shape, loc):
        if isinstance(shape, bd.Compound):
            return bd.Compound(children=[s.moved(loc) for s in shape.solids()])
        return shape.moved(loc)

    env_boxes = {ek: (env[ek], _solid_boxes(env[ek])) for ek in ("rails_vert", "racks_vert", "rails_horiz", "racks_horiz", "crossings_fixed", "rack_bays", "rack_mounts", "rack_mount_hw")}
    mag_boxes = (c_mags, _solid_boxes(c_mags))

    def overlap_pre(a, b_cmp_boxes):
        b_cmp, sb_list = b_cmp_boxes
        ba, bb = a.bounding_box(), b_cmp.bounding_box()
        if (ba.min.X > bb.max.X or bb.min.X > ba.max.X or ba.min.Y > bb.max.Y or bb.min.Y > ba.max.Y
                or ba.min.Z > bb.max.Z or bb.min.Z > ba.max.Z):
            return 0.0
        total = 0.0
        for sa, bba in _solid_boxes(a):
            for sb, bbb in sb_list:
                if (bba.min.X > bbb.max.X or bbb.min.X > bba.max.X or bba.min.Y > bbb.max.Y or bbb.min.Y > bba.max.Y
                        or bba.min.Z > bbb.max.Z or bbb.min.Z > bba.max.Z):
                    continue
                inter = sa.intersect(sb)
                if inter is not None and hasattr(inter, "solids"):
                    total += sum(s.volume for s in inter.solids())
        return total

    bad = []
    # Check internal column vs plugs & static rack_mount_hw vs holes/T-slots once
    v_col_plug = overlap(col, plugs)
    if v_col_plug > 1e-3:
        bad.append(("static", "column_bar", "anti_crush_plugs", v_col_plug))
    for ek_hw in ("rack_mounts", "crossings_fixed", "rack_bays", "rails_vert", "rails_horiz"):
        v_hw = overlap_pre(env["rack_mount_hw"], env_boxes[ek_hw])
        if v_hw > 1e-3:
            bad.append(("static", "rack_mount_hw", ek_hw, v_hw))

    # Verify ALL intermediate swivel angles (-15, -30, -45, -60, -75 deg) clear stationary leg hoods cleanly!
    hood_up, hood_lo = p10_cover, p11_cover
    for h_tag, h_sh in (("upper", hood_up), ("lower", hood_lo)):
        v_hh = overlap(hood_hw, h_sh)
        if v_hh > 1e-3:
            bad.append(("static", "hood_hw", f"{h_tag}_leg_hood", v_hh))
    for tag, zh, h_sh in (("upper", DL.JZ_UP, hood_up), ("lower", DL.JZ_LO, hood_lo)):
        if h_sh.bounding_box().min.Z < 4.0:
            bad.append(("static", f"{tag}_leg_hood", "floor_Z_lt_4mm", h_sh.bounding_box().min.Z))
        for deg in (-15.0, -30.0, -45.0, -60.0, -75.0):
            loc_sw = DL.hub_world_loc(zh, DL.HUB_BASE[tag] + deg, 0.0)
            for tk in turn_keys:
                v = overlap(h_sh, DL.to_hub(pod_parts[tk], loc_sw))
                if v > 1e-3:
                    bad.append((f"sweep_{tag}_{int(deg)}deg", "leg_hood", tk, v))
            v_s = overlap(h_sh, DL.to_hub(s_turn, loc_sw))
            if v_s > 1e-3:
                bad.append((f"sweep_{tag}_{int(deg)}deg", "leg_hood", "sensors_turn", v_s))

    # Multi-pose verification: (label, rot_y_deg, dx, dz, extend_y)
    poses = (
        ("bay0 vertical home (0 deg)", 0.0, 0.0, 0.0, 0.0),
        ("bay0 horiz swivel (-90 deg)", -90.0, 0.0, 0.0, 0.0),
        ("bay0 climb Shelf2 (Z+175)", 0.0, 0.0, 175.0, 0.0),
        ("horiz travel mid-bay (X+180)", -90.0, 180.0, 0.0, 0.0),
        (f"bay0 Shelf1 pick reach (Y+{OB.EXTEND_MAX:.0f})", 0.0, 0.0, 0.0, OB.EXTEND_MAX),
    )

    for label, rot_y, dx, dz, ext_y in poses:
        shift = bd.Location((dx, 0.0, dz))
        col_m = col.moved(shift)
        plugs_m = move_shape(plugs, shift)

        # Body parts moved with robot, with mid/inner picker stages extended when testing pick reach
        b_world = {}
        for bk in body_keys:
            dy_ext = 0.5 * ext_y if bk == "arms_mid" else (ext_y if bk == "arms_in" else 0.0)
            b_world[bk] = move_shape(body_parts[bk], bd.Location((dx, dy_ext, dz)))
        for fk, x_piv in (("fingers_left", OB.BXR - 80.5), ("fingers_right", OB.BXR + 80.5)):
            piv = bd.Location((dx + x_piv, ext_y, dz + OB.TRAY_TOP_Z + OB.FS_ZT))
            b_world[fk] = move_shape(body_parts[fk], piv)

        # Leg hubs at JZ_LO & JZ_UP
        hubs = {}
        for tag, zh in (("upper", DL.JZ_UP), ("lower", DL.JZ_LO)):
            loc_fix = DL.hub_world_loc(zh + dz, DL.HUB_BASE[tag], dx)
            loc_turn = DL.hub_world_loc(zh + dz, DL.HUB_BASE[tag] + rot_y, dx)
            for k in fixed_keys:
                hubs[f"{tag}_{k}"] = DL.to_hub(pod_parts[k], loc_fix)
            for k in turn_keys:
                hubs[f"{tag}_{k}"] = DL.to_hub(pod_parts[k], loc_turn)
            hubs[f"{tag}_sensors_fixed"] = DL.to_hub(s_fix, loc_fix)
            hubs[f"{tag}_sensors_turn"] = DL.to_hub(s_turn, loc_turn)
            hubs[f"{tag}_sensors_hw"] = DL.to_hub(s_hw, loc_turn)
        hoods_w = {
            "upper_leg_hood": hood_up.moved(shift),
            "lower_leg_hood": hood_lo.moved(shift),
        }
        # Check stationary lampshade hoods vs all pod parts (both hubs), sensors, column, plugs, body, rails, racks, mounts
        for h_k, h_s in hoods_w.items():
            for pk_c, ps_c in hubs.items():
                v = overlap(h_s, ps_c)
                if v > 1e-3:
                    bad.append((label, h_k, pk_c, v))
            for sc_k, sc_s in (("column_bar", col_m), ("anti_crush_plugs", plugs_m)):
                v = overlap(h_s, sc_s)
                if v > 1e-3:
                    bad.append((label, h_k, sc_k, v))
            for bk_c, bs_c in b_world.items():
                if bk_c == "body_fasteners":
                    continue  # head overlap check handled via M4 thru-holes on collar front
                v = overlap(h_s, bs_c)
                if v > 1e-3:
                    bad.append((label, h_k, bk_c, v))
            v_m = overlap_pre(h_s, mag_boxes)
            if v_m > 1e-3:
                bad.append((label, h_k, "crossing_magnets", v_m))
            for ek in ("rails_vert", "racks_vert", "rails_horiz", "racks_horiz", "crossings_fixed", "rack_bays", "rack_mounts", "rack_mount_hw"):
                v = overlap_pre(h_s, env_boxes[ek])
                if v > 1e-3:
                    bad.append((label, h_k, ek, v))

        # Verify turning leg parts, Hall sensors, and turning crossing pieces also clear rack_mounts
        for hk, hs in hubs.items():
            v_rm = overlap_pre(hs, env_boxes["rack_mounts"])
            if v_rm > 1e-3:
                bad.append((label, hk, "rack_mounts", v_rm))

        # Crossing turntables angled with robot when at Bay 0/Bay 1 crossing
        turn_cross_solids = []
        for bx in BAY_XS_FULL:
            ang = rot_y if (abs(dx - bx) < 0.5 and abs(dz) < 0.5) else 0.0
            for zc in (DL.JZ_LO, DL.JZ_UP):
                loc_c = DL.hub_world_loc(zc, ang, bx) * DL.LOC_POD_TO_HUB
                for tk in ("xp_rotor", "xp_pivot_608zz", "xp_pivot_m8_stack", "xp_rotor_screws", "xp_stop_pin", "xp_tnuts"):
                    turn_cross_solids.append(move_shape(turning_tpl[tk], loc_c))
        turn_cross_cmp = bd.Compound(children=turn_cross_solids)
        v_tc_rm = overlap_pre(turn_cross_cmp, env_boxes["rack_mounts"])
        if v_tc_rm > 1e-3:
            bad.append((label, "crossings_turn", "rack_mounts", v_tc_rm))

        # 1. Body vs Column & Plugs
        for bk in ("body_collar", "tray_boom", "battery_pod"):
            for tag_c, sc in (("column_bar", col_m), ("anti_crush_plugs", plugs_m)):
                v = overlap(b_world[bk], sc)
                if v > 1e-3:
                    bad.append((label, bk, tag_c, v))

        # 2. Body vs H1 / H2 pods + sensors + magnets
        for bk, bs in b_world.items():
            for hk, hs in hubs.items():
                v = overlap(bs, hs)
                if v > 1e-3:
                    bad.append((label, bk, hk, v))
            v_mag = overlap_pre(bs, mag_boxes)
            if v_mag > 1e-3:
                bad.append((label, bk, "crossing_magnets", v_mag))

        # 3. Body vs rails, crossings, mount brackets, and 2-bay slotted-angle rack bays
        for ek in ("rails_vert", "racks_vert", "rails_horiz", "racks_horiz", "crossings_fixed", "rack_bays", "rack_mounts", "rack_mount_hw"):
            es_box = env_boxes[ek]
            for bk, bs in b_world.items():
                v = overlap_pre(bs, es_box)
                if v > 1e-3:
                    bad.append((label, bk, ek, v))
        for bk in ("body_collar", "body_fasteners", "tray_boom", "battery_pod", "tray_deck"):
            v_tc = overlap(b_world[bk], turn_cross_cmp)
            if v_tc > 1e-3:
                bad.append((label, bk, "crossings_turn", v_tc))

        print(f"[{label:32s}] collisions so far: {len(bad)}")

    if bad:
        for item in bad:
            print("  COLLISION:", item)
    else:
        print("RESULT: OK - 0 collisions across unified full robot (legs + column + R40..R45 body + picker + rails + 2-bay racks)")
    return len(bad) == 0


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    os.makedirs(SIM_MESH_DIR, exist_ok=True)

    col, plugs = build_unified_column_and_plugs()
    s_fix, s_turn, s_hw, _, _ = DL.build_sensors_and_magnets()
    env, c_mags = build_compact_railing_and_magnets()
    p10_cover, p11_cover, hood_hw = build_leg_protective_covers()

    collar, fast_col = OB.build_body_column_collar()
    boom, live_wish, load_cell, lc_hw = OB.build_tray_boom_and_loadcell()
    bat_box, bat_lid, bat_pack = OB.build_battery_pod()
    picker = OB.build_tray_and_picker()
    basket, _ = OB.build_reference_env()

    for lbl, part in (("B01_collar", collar), ("B02_boom", boom), ("B03_bat", bat_box), ("B03L_lid", bat_lid),
                      ("B04_tray", picker["tray_deck"]), ("B05_wishbone", live_wish),
                      ("P10_leg_hood_upper", p10_cover), ("P10_leg_hood_lower", p11_cover)):
        assert len(part.solids()) == 1, f"{lbl} expected 1 solid, got {len(part.solids())}"

    fast_all = bd.Compound(children=[*fast_col.solids(), *lc_hw.solids(), *hood_hw.solids()], label="Body_And_Hood_Fasteners")

    body_parts = {
        "body_collar": collar,
        "body_fasteners": fast_all,
        "tray_boom": boom,
        "live_wishbone": live_wish,
        "load_cell": load_cell,
        "tray_deck": picker["tray_deck"],
        "tray_stops": picker["tray_stops"],
        "arms_outer": picker["arms_outer"],
        "arms_mid": picker["arms_mid"],
        "arms_in": picker["arms_in"],
        "fingers_left": picker["fingers_left"],
        "fingers_right": picker["fingers_right"],
        "picker_belts": picker["picker_belts"],
        "picker_servos": picker["picker_servos"],
        "battery_pod": bat_box,
        "battery_lid": bat_lid,
        "battery_pack": bat_pack,
        "ref_basket": basket,
    }

    export_meshes = {
        "column_bar": col,
        "anti_crush_inserts": plugs,
        "sensors_pod_fixed": s_fix,
        "sensors_pod_turn": s_turn,
        "sensors_hardware": s_hw,
        "leg_hood_upper": p10_cover,
        "leg_hood_lower": p11_cover,
        "crossing_magnets": c_mags,
        "rails_vert": env["rails_vert"],
        "racks_vert": env["racks_vert"],
        "rails_horiz": env["rails_horiz"],
        "racks_horiz": env["racks_horiz"],
        "crossings_fixed": env["crossings_fixed"],
        "rack_bays": env["rack_bays"],
        "rack_mounts": env["rack_mounts"],
        "rack_mount_hw": env["rack_mount_hw"],
        **body_parts,
    }
    for name, shape in export_meshes.items():
        bd.export_stl(shape, os.path.join(SIM_MESH_DIR, f"{name}.stl"), tolerance=0.05, angular_tolerance=0.2)
    bd.export_stl(p10_cover, os.path.join(OUT_DIR, "opt1_P10_UP_leg_hood.stl"), tolerance=0.01, angular_tolerance=0.1)
    bd.export_stl(p11_cover, os.path.join(OUT_DIR, "opt1_P10_LO_leg_hood.stl"), tolerance=0.01, angular_tolerance=0.1)
    print(f"Exported {len(export_meshes)} unified viewer meshes to {SIM_MESH_DIR}")

    # Export unified nominal STEP assembly (bay 0 vertical 0 deg home)
    pod_parts = O.build_assembled_parts()
    asm_children = [
        col, plugs, c_mags, p10_cover, p11_cover,
        env["rails_vert"], env["racks_vert"], env["rails_horiz"], env["racks_horiz"],
        env["crossings_fixed"], env["rack_bays"], env["rack_mounts"], env["rack_mount_hw"],
        collar, fast_all, boom, live_wish, load_cell,
        picker["tray_deck"], picker["tray_stops"],
        picker["arms_outer"], picker["arms_mid"], picker["arms_in"],
        picker["picker_belts"], picker["picker_servos"],
        picker["fingers_left"].moved(bd.Location((OB.BXR - 80.5, 0.0, OB.TRAY_TOP_Z + OB.FS_ZT))),
        picker["fingers_right"].moved(bd.Location((OB.BXR + 80.5, 0.0, OB.TRAY_TOP_Z + OB.FS_ZT))),
        bat_box, bat_lid, bat_pack, basket,
    ]
    for tag, zh in (("lower", DL.JZ_LO), ("upper", DL.JZ_UP)):
        loc = DL.hub_world_loc(zh, DL.HUB_BASE[tag], 0.0)
        asm_children.extend([DL.to_hub(s_fix, loc), DL.to_hub(s_turn, loc), DL.to_hub(s_hw, loc)])
        for pk, ps in pod_parts.items():
            if pk not in ("t_rail", "t_rack"):
                asm_children.append(DL.to_hub(ps, loc))
    for bx in BAY_XS_FULL:
        for zc in (DL.JZ_LO, DL.JZ_UP):
            loc = DL.hub_world_loc(zc, 0.0, bx)
            for ts in env["turning_tpl"].values():
                asm_children.append(ts.moved(loc * DL.LOC_POD_TO_HUB))

    step_path = os.path.join(OUT_DIR, "opt1_full_robot_on_rails.step")
    bd.export_step(bd.Compound(children=asm_children, label="Option1_Full_Robot_On_2Bay_Rails"), step_path)
    print(f"Exported unified STEP assembly -> {step_path}")

    ok = check_full_clearances(col, plugs, body_parts, env, (s_fix, s_turn, s_hw, c_mags), (p10_cover, p11_cover, hood_hw))
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
