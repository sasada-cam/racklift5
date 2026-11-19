"""Objects Compartment — 2-Stage Forklift Driven by Duino.lk CNC4759-A T8 Kit & MT0382 17HS4401.

Builds the isolated underneath load-carrying base (R41 + TAL220 + R45 + clean flat R44 tray)
with the dedicated 2-stage telescopic forklift compartment bolted on top.

Every 3D-printed part obeys CLAUDE.md ID + batch + 0.6 mm engraving + distinct colour rules:
  - BATCH = "B01"
  - R46: Forklift Sloped Skid-Wing Deck, Rear Wall & KP08/17HS4401 Cradle (mark: R46-B01)
  - R47: Fixed Base Guide Rail - Left  (mark: R47-B01)
  - R48: Fixed Base Guide Rail - Right (mark: R48-B01)
  - R49: Front 688ZZ Bearing Arch Saddle (mark: R49)
  - R50: Stage-1 Bridged H-Carriage (mark: R50-B01)
  - R51: Stage-2 Tapered Fork Tine - Left  (mark: R51-B01)
  - R52: Stage-2 Tapered Fork Tine - Right (mark: R52-B01)
  - R53: Removable Under-Tray Rear Cantilever Tie Plate (mark: R53-B01)
  - R54: Stage-2 Horizontal 1:2 Spur Pinion - Left  (16T MOD=1.0, mark: R54)
  - R55: Stage-2 Horizontal 1:2 Spur Pinion - Right (16T MOD=1.0, mark: R55)
  - R56: Removable Screwed Top Motor & Rack-Tail Enclosure Lid (mark: R56-B01)
  - R57A / R57B: Hold-Down Option A (Screwed Side T-Slot Rails) vs Option B (MR63ZZ Roller Brackets)
  - Center Trench End-Stops: 2x SS-5GL Microswitches + R47/R48 Plastic Hard-Stop Risers (±1.5 mm)

Bought hardware:
  - CNC4759-A T8 Lead Screw (2 mm pitch, 8 mm lead, ~202 mm cut), trimmed Ø22 copper flange nut,
    rear KP08 pillow bearing, 8->5 mm flexible coupling, front 688ZZ bearing
  - MT0382 17HS4401 NEMA 17 Stepper (42.3 x 42.3 x 40 mm, 0.4 Nm, Ø5 mm D-shaft)
  - 2x F683ZZ flanged bearings + vertical M3 pins (Stage-2 1:2 rack-and-pinion) + M3/M5 DIN hardware

Usage:
  .pixi/envs/default/bin/python cad/build_objects_compartment.py
"""

import glob
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import build123d as bd  # noqa: E402
import build_option1_body as OB  # noqa: E402
import build_t_slot_option1 as O  # noqa: E402

OUT_DIR = os.path.abspath(os.path.join(HERE, "..", "models", "components", "objects_compartment"))
SIM_MESH_DIR = os.path.abspath(os.path.join(HERE, "..", "sim", "build", "meshes_objects_compartment"))

# CLAUDE.md mandatory printed part IDs & print batch
BATCH = "B01"
PART_IDS = {
    "forklift_deck": "R46",
    "rail_left": "R47",
    "rail_right": "R48",
    "front_688zz_arch": "R49",
    "stage1_carriage": "R50",
    "fork_tine_left": "R51",
    "fork_tine_right": "R52",
    "under_tie": "R53",
    "pinion_left": "R54",
    "pinion_right": "R55",
    "fork_top_cover": "R56",
    "opt_a_left": "R57A",
    "opt_a_right": "R57A",
    "opt_b_left": "R57B",
    "opt_b_right": "R57B",
}
# Distinct non-adjacent hex shades (CLAUDE.md §4)
PART_COLORS = {
    "R46": "#26A69A",  # teal green
    "R47": "#00897B",  # deep cyan-teal
    "R48": "#1B5E20",  # forest green
    "R49": "#80CBC4",  # pale mint teal (adjacent to R46/R47/R48)
    "R50": "#43A047",  # medium leaf green
    "R51": "#F97316",  # safety orange (tines)
    "R52": "#EA580C",  # burnt orange (tines)
    "R53": "#2E7D32",  # deep emerald (removable rear cantilever tie under R46/R44)
    "R54": "#EAB308",  # golden amber (1:2 rack pinion left)
    "R55": "#CA8A04",  # deep gold (1:2 rack pinion right)
    "R56": "#0E7490",  # deep ocean cyan (screwed top motor & rack-tail enclosure lid)
    "R57A": "#6366F1", # indigo-violet (Option A: screwed side T-slot hold-down rails)
    "R57B": "#EC4899", # rose-magenta (Option B: screwed MR63ZZ roller hold-down brackets)
}

SLOPE_Z_BACK = 185.40
SLOPE_Z_FRONT = 181.60
BAY_FLOOR_Z = 180.50
NUT_TRENCH_Z = 175.20   # 1.2 mm deck skin + 8 mm ribs left on R44; embeds T8 & nut cleanly below 186 mm
BAY_X0, BAY_X1 = OB.BXR - 59.0, OB.BXR + 59.0  # 121.0 .. 239.0 mm
WALL_Y0, WALL_Y1 = -245.0, -238.0
WALL_TOP_Z = 200.00
CURB_W, CURB_TOP_ADD = 2.5, 2.2

FORK_DX = 48.0          # left/right tine centers at X = 132 & 228 mm
SCREW_X = OB.BXR        # 180.0 mm centered symmetrically
SCREW_Z = 180.50        # lowered -4.0 mm so T8, copper nut & 688ZZ arch all stay strictly < 189.7 mm
GEAR_Z0, GEAR_Z1 = 183.50, 187.20   # R47/R48 racks, R54/R55 cogwheels & R51/R52 rack tails stay low
FORK_Z0, FORK_Z1 = 183.50, 190.30   # orange fork blades stand proud ABOVE cogwheels (188.15) & R49 arch (189.60)
MID_STROKE_MAX = 120.0  # Stage-1 stroke (total tine reach = 240 mm)

# Stage-2 Horizontal 1:2 Rack-and-Pinion geometry (16T MOD=1.0, rp=8.00 mm)
RP_MOD, RP_Z, RP_PA = 1.0, 16, 20.0
PIN_RP = RP_MOD * RP_Z / 2.0  # 8.00 mm
PIN_DX_TINE = 8.50            # tine pitch line offset inboard from xc (140.5 / 219.5 mm)
PIN_DX_CENTER = PIN_DX_TINE + PIN_RP  # 16.50 mm inboard from xc (148.5 / 211.5 mm)
PIN_Y_HOME = -202.0           # moves -202 -> -82 mm inside base rack (-237.2..-70.6)

KP08_Y0, KP08_Y1 = -258.5, -245.5
COUP_Y0, COUP_Y1 = -286.5, -261.5
BULK_Y0, BULK_Y1 = -295.5, -291.5


def mark(key: str, id_only: bool = False) -> str:
    pid = PART_IDS[key]
    return pid if id_only else f"{pid}-{BATCH}"


def _hex_to_bd_color(hex_str: str) -> bd.Color:
    h = hex_str.lstrip("#")
    return bd.Color(int(h[0:2], 16) / 255.0, int(h[2:4], 16) / 255.0, int(h[4:6], 16) / 255.0)


def _engrave_verified(part: bd.Part, key: str, origin, x_dir, normal, size: float, id_only: bool = False) -> bd.Part:
    """Engrave 0.6 mm deep ID(-BATCH) mark and verify single solid + measurable volume cut (CLAUDE.md §3)."""
    text = mark(key, id_only=id_only)
    vol_before = part.volume
    out = O.engrave(part, text, origin, x_dir, normal, size)
    solids = out.solids() if hasattr(out, "solids") else [out]
    assert len(solids) == 1, f"{text}: engraving split into {len(solids)} solids"
    dv = vol_before - out.volume
    assert dv > 0.05, f"{text}: engraving did not cut into solid (dv={dv:.4f})"
    pid = PART_IDS[key]
    out.label = f"{text}"
    out.color = _hex_to_bd_color(PART_COLORS[pid])
    print(f"  engrave {text:8s}: 1 solid, -{dv:.2f} mm^3 ({PART_COLORS[pid]})")
    return out


def _overlap(a, b) -> float:
    if not a.bounding_box().overlaps(b.bounding_box()):
        return 0.0
    sa_list = a.solids() if hasattr(a, "solids") else [a]
    sb_list = b.solids() if hasattr(b, "solids") else [b]
    total = 0.0
    for sa in sa_list:
        ba = sa.bounding_box()
        for sb in sb_list:
            if not ba.overlaps(sb.bounding_box()):
                continue
            inter = sa.intersect(sb)
            if inter is not None and hasattr(inter, "solids"):
                total += sum(s.volume for s in inter.solids())
    return total


def _hex_nut(r_af: float, h: float, axis: str, x: float, y: float, z: float, r_hole: float) -> bd.Part:
    rad = r_af / math.sqrt(3.0)
    with bd.BuildPart() as bp:
        loc = bd.Location((x, y, z)) if axis == "z" else bd.Location((x, y, z), (-90, 0, 0))
        with bd.Locations(loc):
            with bd.BuildSketch():
                bd.RegularPolygon(radius=rad, side_count=6, rotation=30.0)
            bd.extrude(amount=h)
    bore = OB._cyl_axis(r_hole, z - 0.5, z + h + 0.5, "z", x, y) if axis == "z" else OB._cyl_axis(
        r_hole, y - 0.5, y + h + 0.5, "y", x, z
    )
    return bp.part - bore


def _socket_bolt(r_head: float, h_head: float, r_shank: float, len_shank: float, axis: str, x: float, y: float, z: float, head_dir: int = 1) -> bd.Part:
    coords = {"x": (x, y, z), "y": (y, x, z), "z": (z, x, y)}
    c0, u, v = coords[axis]
    if head_dir > 0:
        head = OB._cyl_axis(r_head, c0, c0 + h_head, axis, u, v) - OB._cyl_axis(
            r_head * 0.52, c0 + h_head * 0.45, c0 + h_head + 0.2, axis, u, v
        )
        shank = OB._cyl_axis(r_shank, c0 - len_shank, c0, axis, u, v)
    else:
        head = OB._cyl_axis(r_head, c0 - h_head, c0, axis, u, v) - OB._cyl_axis(
            r_head * 0.52, c0 - h_head - 0.2, c0 - h_head * 0.45, axis, u, v
        )
        shank = OB._cyl_axis(r_shank, c0, c0 + len_shank, axis, u, v)
    return head + shank


def _build_horizontal_rack(
    x_pitch: float, x_back: float, dir_x: int, y0: float, y1: float, z0: float, z1: float,
    y_phase: float = PIN_Y_HOME, mod: float = RP_MOD, pa_deg: float = RP_PA, clearance: float = 0.28,
) -> bd.Part:
    """Build horizontal-plane trapezoidal rack aligned with valley centered at y_phase + n*pitch."""
    pitch = math.pi * mod
    pa = math.radians(pa_deg)
    x_root = x_pitch - dir_x * (1.25 * mod)
    x_tip = x_pitch + dir_x * (0.85 * mod)
    s = 0.5 * pitch - clearance
    hw_root = 0.5 * s + 1.25 * mod * math.tan(pa)
    hw_tip = max(0.18, 0.5 * s - 0.85 * mod * math.tan(pa))

    i_min = math.ceil((y0 + hw_root + 0.15 - y_phase) / pitch - 0.5)
    i_max = math.floor((y1 - hw_root - 0.15 - y_phase) / pitch - 0.5)
    pts = [(x_back, y0), (x_root, y0)]
    for i in range(i_min, i_max + 1):
        yc = y_phase + (i + 0.5) * pitch
        pts.extend([
            (x_root, yc - hw_root),
            (x_tip, yc - hw_tip),
            (x_tip, yc + hw_tip),
            (x_root, yc + hw_root),
        ])
    pts.extend([(x_root, y1), (x_back, y1), (x_back, y0)])
    with bd.BuildPart() as bp:
        with bd.BuildSketch(bd.Plane.XY.offset(z0)):
            with bd.BuildLine():
                bd.Polyline(*pts)
            bd.make_face()
        bd.extrude(amount=z1 - z0)
    return bp.part


def _revolved_y(profile_rz, y_start: float, cx: float, cz: float) -> bd.Part:
    pts = profile_rz if profile_rz[-1] == profile_rz[0] else [*profile_rz, profile_rz[0]]
    with bd.BuildPart() as bp:
        with bd.BuildSketch(bd.Plane.XZ):
            with bd.BuildLine():
                bd.Polyline(*[(r, z) for r, z in pts])
            bd.make_face()
        bd.revolve(axis=bd.Axis.Z, revolution_arc=360.0)
    return bp.part.moved(bd.Location((cx, y_start, cz), (-90, 0, 0)))


def _build_t8_threaded_screw(y0: float, y1: float, cx: float, cz: float) -> bd.Part:
    length = y1 - y0
    pitch = 2.0
    n_pitch = int(length // pitch)
    r_root, r_crest = 3.15, 4.00
    pts = [(0.0, 0.0), (r_root, 0.0)]
    y_cur = 1.0
    for i in range(n_pitch - 1):
        yc = y_cur + i * pitch
        if yc + pitch > length - 1.0:
            break
        pts.extend([(r_root, yc), (r_crest, yc + 0.45), (r_crest, yc + 1.15), (r_root, yc + 1.60)])
    pts.extend([(r_root, length), (0.0, length)])
    return _revolved_y(pts, y0, cx, cz)


def _build_slotted_coupler(y0: float, y1: float, cx: float, cz: float) -> bd.Part:
    L = y1 - y0
    r_out, r_kerf = 9.50, 6.20
    pts = [(0.0, 0.0), (r_out, 0.0), (r_out, 7.0)]
    for k in range(5):
        gk = 7.0 + k * 2.2
        pts.extend([(r_kerf, gk), (r_kerf, gk + 0.9), (r_out, gk + 1.1), (r_out, gk + 2.2)])
    pts.extend([(r_out, L), (0.0, L)])
    body = _revolved_y(pts, y0, cx, cz)
    body = body - OB._cyl_axis(4.20, y0 + 12.5, y1 + 0.5, "y", cx, cz)
    body = body - OB._cyl_axis(2.65, y0 - 0.5, y0 + 12.5, "y", cx, cz)
    for dy in (3.5, L - 3.5):
        body = body - OB._cyl_axis(2.0, cx - 10.0, cx + 10.0, "x", y0 + dy, cz)
    return body


def _build_kp08_pillow(y0: float, y1: float, cx: float, cz: float) -> bd.Part:
    foot = OB._box(cx - 27.5, cx + 27.5, y0, y1, cz - 15.0, cz - 10.0)
    for bx in (cx - 21.0, cx + 21.0):
        foot = foot - OB._cyl_axis(2.65, cz - 15.5, cz - 9.5, "z", bx, (y0 + y1) / 2.0)
    arch = OB._box(cx - 13.5, cx + 13.5, y0, y1, cz - 11.0, cz) + OB._cyl_axis(13.5, y0, y1, "y", cx, cz)
    housing = (foot + arch) - OB._cyl_axis(10.0, y0 - 0.2, y0 + 1.8, "y", cx, cz) - OB._cyl_axis(
        10.0, y1 - 1.8, y1 + 0.2, "y", cx, cz
    )
    collar = OB._cyl_axis(6.3, y0 - 1.2, y1 + 1.2, "y", cx, cz)
    shield = OB._cyl_axis(9.8, y0 + 1.5, y1 - 1.5, "y", cx, cz)
    return (housing + collar + shield) - OB._cyl_axis(4.20, y0 - 2.0, y1 + 2.0, "y", cx, cz)


def build_forklift_deck(tray_clean: bd.Part):
    """Build R46 sloped deck, rear wall, KP08 housing & 17HS4401 cradle + all fixed fasteners."""
    box = OB._box
    z0 = OB.TRAY_TOP_Z + 0.05  # 178.05 mm

    profile_yz = [
        (WALL_Y0, z0),
        (OB.TRAY_Y1, z0),
        (OB.TRAY_Y1, SLOPE_Z_FRONT),
        (WALL_Y1, SLOPE_Z_BACK),
        (WALL_Y1, WALL_TOP_Z),
        (WALL_Y0, WALL_TOP_Z),
        (WALL_Y0, z0),
    ]
    with bd.BuildPart() as bp:
        with bd.BuildSketch(bd.Plane.YZ.offset(OB.TRAY_X0)):
            with bd.BuildLine():
                bd.Polyline(*profile_yz)
            bd.make_face()
        bd.extrude(amount=OB.TRAY_W)
    comp = bp.part

    # Central fork bay + embedded T8/nut channel into R46 & R44 + front 688ZZ cradle
    comp = comp - box(BAY_X0, BAY_X1, WALL_Y1, OB.TRAY_Y1 + 0.5, BAY_FLOOR_Z, WALL_TOP_Z + 2.0)
    comp = comp - box(SCREW_X - 13.0, SCREW_X + 13.0, WALL_Y1, -78.0, NUT_TRENCH_Z, BAY_FLOOR_Z + 0.5)
    comp = comp - box(SCREW_X - 16.5, SCREW_X + 16.5, -78.0, OB.TRAY_Y1 + 0.5, 179.05, BAY_FLOOR_Z + 0.5)
    comp = comp - box(SCREW_X - 10.2, SCREW_X + 10.2, -78.0, OB.TRAY_Y1 + 0.5, NUT_TRENCH_Z, 179.20)
    # Rear-wall pass-through tunnels for Left/Right R51/R52 Tine Rack tails (-335..-234 mm @ GEAR_Z0..GEAR_Z1)
    comp = comp - box(OB.BXR - FORK_DX + 2.4, OB.BXR - FORK_DX + 10.4, WALL_Y0 - 0.5, WALL_Y1 + 0.5, GEAR_Z0 - 0.5, GEAR_Z1 + 0.6)
    comp = comp - box(OB.BXR + FORK_DX - 10.4, OB.BXR + FORK_DX - 2.4, WALL_Y0 - 0.5, WALL_Y1 + 0.5, GEAR_Z0 - 0.5, GEAR_Z1 + 0.6)
    tray_clean = tray_clean - box(SCREW_X - 12.6, SCREW_X + 12.6, WALL_Y1, -77.5, NUT_TRENCH_Z, OB.TRAY_TOP_Z + 0.5)
    # Recessed left-trench pockets + wire groove for 2x SS-5GL end-stop microswitches (Home Y=-229.5..-215.8 & Max Y=-92.5..-78.0)
    for sw_y0, sw_y1 in ((-229.5, -215.8), (-92.5, -78.0)):
        comp = comp - box(SCREW_X - 18.4, SCREW_X - 12.4, sw_y0, sw_y1, 173.8, BAY_FLOOR_Z + 0.5)
        tray_clean = tray_clean - box(SCREW_X - 18.4, SCREW_X - 12.4, sw_y0, sw_y1, 173.8, OB.TRAY_TOP_Z + 0.5)
    comp = comp - box(SCREW_X - 18.4, SCREW_X - 16.0, WALL_Y0 - 0.5, -78.0, 175.0, 178.1)
    tray_clean = tray_clean - box(SCREW_X - 18.4, SCREW_X - 16.0, WALL_Y0 - 0.5, -78.0, 175.0, OB.TRAY_TOP_Z + 0.5)
    tray_clean = tray_clean - OB._cyl_axis(4.60, OB.TRAY_Y0 - 0.5, WALL_Y1 + 0.5, "y", SCREW_X, SCREW_Z)
    tray_clean = tray_clean - OB._cyl_axis(6.80, OB.TRAY_Y0 - 0.5, OB.TRAY_Y0 + 1.2, "y", SCREW_X, SCREW_Z)
    tray_clean = tray_clean - OB._cyl_axis(8.20, -77.8, OB.TRAY_Y1 + 0.5, "y", SCREW_X, SCREW_Z)

    for sx0, sx1 in ((OB.TRAY_X0, OB.TRAY_X0 + CURB_W), (OB.TRAY_X1 - CURB_W, OB.TRAY_X1)):
        with bd.BuildPart() as cb:
            with bd.BuildSketch(bd.Plane.YZ.offset(sx0)):
                with bd.BuildLine():
                    bd.Polyline(
                        (WALL_Y1, SLOPE_Z_BACK - 0.5),
                        (OB.TRAY_Y1, SLOPE_Z_FRONT - 0.5),
                        (OB.TRAY_Y1, SLOPE_Z_FRONT + CURB_TOP_ADD),
                        (WALL_Y1, SLOPE_Z_BACK + CURB_TOP_ADD),
                        (WALL_Y1, SLOPE_Z_BACK - 0.5),
                    )
                bd.make_face()
            bd.extrude(amount=sx1 - sx0)
        comp = comp + cb.part

    for nx0, nx1 in ((OB.TRAY_X0 + CURB_W, OB.TRAY_X0 + 16.0), (OB.TRAY_X1 - 16.0, OB.TRAY_X1 - CURB_W)):
        comp = comp + box(nx0, nx1, OB.TRAY_Y1 - 6.0, OB.TRAY_Y1, SLOPE_Z_FRONT - 0.5, 183.80)

    # Unified Rear KP08 housing + wrapped 17HS4401 cradle (drop-on R46 with pocketed floor for removable R53 tie)
    CRADLE_Z0 = 155.70
    TIE_SPLIT_Z = 160.20
    TIE_Y0 = -288.00
    REAR_CAP_Y0 = BULK_Y0 - 43.4  # -338.9 mm (3.0 mm rear cap behind 17HS4401)
    # 1) Continuous full-width cradle bed: 155.70 under motor, stepping to TIE_SPLIT_Z under Y=-288..-245 for flush R53-B01
    comp = comp + box(SCREW_X - 33.0, SCREW_X + 33.0, REAR_CAP_Y0, TIE_Y0, CRADLE_Z0, SCREW_Z - 21.45)
    comp = comp + box(SCREW_X - 33.0, SCREW_X + 33.0, BULK_Y0, TIE_Y0 + 0.1, CRADLE_Z0, 161.20)
    comp = comp + box(SCREW_X - 33.0, SCREW_X + 33.0, TIE_Y0, WALL_Y0, TIE_SPLIT_Z, 161.20)
    # 2) KP08 flat pedestal raised flush under KP08 (-259.0..WALL_Y0) + unified Z=200.00 outer cheeks (-291.5..WALL_Y0)
    comp = comp + box(SCREW_X - 33.0, SCREW_X + 33.0, KP08_Y0 - 0.5, WALL_Y0, TIE_SPLIT_Z, 165.40)
    for rx0, rx1 in ((SCREW_X - 33.0, SCREW_X - 28.0), (SCREW_X + 28.0, SCREW_X + 33.0)):
        comp = comp + box(rx0, rx1, BULK_Y1, TIE_Y0 + 0.1, CRADLE_Z0, WALL_TOP_Z)
        comp = comp + box(rx0, rx1, TIE_Y0, WALL_Y0, TIE_SPLIT_Z, WALL_TOP_Z)

    kp_housing = box(SCREW_X - 33.0, SCREW_X + 33.0, KP08_Y0 - 0.5, WALL_Y0, 165.40, WALL_TOP_Z)
    kp_foot_cut = box(SCREW_X - 27.8, SCREW_X + 27.8, KP08_Y0 - 1.0, KP08_Y1 + 0.3, 165.00, SCREW_Z - 9.7)
    kp_arch_cut = (
        box(SCREW_X - 13.8, SCREW_X + 13.8, KP08_Y0 - 1.0, KP08_Y1 + 0.3, SCREW_Z - 11.2, SCREW_Z)
        + OB._cyl_axis(13.8, KP08_Y0 - 1.0, KP08_Y1 + 0.3, "y", SCREW_X, SCREW_Z)
    )
    kp_collar_cut = OB._cyl_axis(6.8, KP08_Y0 - 2.0, WALL_Y0 + 0.8, "y", SCREW_X, SCREW_Z)
    kp_yc_cut = (KP08_Y0 + KP08_Y1) / 2.0
    kp_bolt_access = (
        OB._cyl_axis(4.8, SCREW_Z - 11.0, WALL_TOP_Z + 0.5, "z", SCREW_X - 21.0, kp_yc_cut)
        + OB._cyl_axis(4.8, SCREW_Z - 11.0, WALL_TOP_Z + 0.5, "z", SCREW_X + 21.0, kp_yc_cut)
    )
    comp = comp + (kp_housing - kp_foot_cut - kp_arch_cut - kp_collar_cut - kp_bolt_access)

    # 3) Bulkhead & snug 17HS4401 wrap pocket (octagonal lower-corner fillets eliminate any corner silver sliver)
    bulkhead = box(SCREW_X - 33.0, SCREW_X + 33.0, BULK_Y0, BULK_Y1, CRADLE_Z0, 201.80) - OB._cyl_axis(
        11.5, BULK_Y0 - 0.5, BULK_Y1 + 0.5, "y", SCREW_X, SCREW_Z
    )
    comp = comp + bulkhead
    for sx0, sx1 in ((SCREW_X - 33.0, SCREW_X - 21.45), (SCREW_X + 21.45, SCREW_X + 33.0)):
        comp = comp + box(sx0, sx1, REAR_CAP_Y0, BULK_Y0 + 0.2, CRADLE_Z0, 201.80)
    comp = comp + box(SCREW_X - 33.0, SCREW_X + 33.0, REAR_CAP_Y0, BULK_Y0 + 0.2, CRADLE_Z0, SCREW_Z - 21.45)
    # 45-degree chamfer ribs inside lower motor pocket to cradle the 17HS4401 chamfers cleanly
    for csgn in (-1, 1):
        with bd.BuildPart() as cb:
            with bd.BuildSketch(bd.Plane.XZ):
                with bd.BuildLine():
                    bd.Polyline(
                        (SCREW_X + csgn * 21.55, SCREW_Z - 15.6),
                        (SCREW_X + csgn * 15.6, SCREW_Z - 21.55),
                        (SCREW_X + csgn * 21.55, SCREW_Z - 21.55),
                        (SCREW_X + csgn * 21.55, SCREW_Z - 15.6),
                    )
                bd.make_face()
            bd.extrude(amount=40.6)
        comp = comp + cb.part.moved(bd.Location((0.0, BULK_Y0, 0.0)))
    # Closed rear protective end cap behind NEMA 17 (top center notch leaves 4-wire JST cable exit clear)
    rear_cap = box(SCREW_X - 33.0, SCREW_X + 33.0, REAR_CAP_Y0, BULK_Y0 - 40.35, CRADLE_Z0, 201.80) - box(
        SCREW_X - 7.5, SCREW_X + 7.5, REAR_CAP_Y0 - 0.2, BULK_Y0 - 40.1, 195.0, 202.2
    )
    comp = comp + rear_cap
    # Open-top U-channel rear protective rack-tail bays (bottom, outer, inner & rear bumper walls up to WALL_TOP_Z=200.0;
    # top stays open for drop-in assembly & top visibility, sealed by removable screwed enclosure R56-B01)
    # Tails span X=134.8..142.5 (L) and X=217.5..225.2 (R), Z=GEAR_Z0..GEAR_Z1 (183.5..187.2)
    REAR_OUT_PAD = 4.4  # outer wall X=127.6..134.4 (L) & 225.6..232.4 (R) = 6.8 mm wide for M3 lid screws
    for sh_x0, sh_x1, x_clear0, x_clear1 in (
        (OB.BXR - FORK_DX - REAR_OUT_PAD, SCREW_X - 32.8, OB.BXR - FORK_DX + 2.4, OB.BXR - FORK_DX + 10.4),
        (SCREW_X + 32.8, OB.BXR + FORK_DX + REAR_OUT_PAD, OB.BXR + FORK_DX - 10.4, OB.BXR + FORK_DX - 2.4),
    ):
        tail_box = box(sh_x0, sh_x1, REAR_CAP_Y0, WALL_Y0 + 0.1, 179.00, WALL_TOP_Z) - box(
            x_clear0, x_clear1, REAR_CAP_Y0 + 3.0, WALL_Y0 + 0.5, GEAR_Z0 - 0.5, WALL_TOP_Z + 0.5
        )
        comp = comp + tail_box
    comp = comp - OB._cyl_axis(4.6, WALL_Y0 - 1.0, WALL_Y1 + 1.0, "y", SCREW_X, SCREW_Z)
    comp = comp - OB._cyl_axis(6.8, WALL_Y0 - 0.5, WALL_Y0 + 1.0, "y", SCREW_X, SCREW_Z)

    # 4) Drop-on outer R46 skirts falling over R44 edges flush to bottom (Z=168.0) at 4 vertices + 4 mid-edges
    #    No inward under-hooks on R46 so R46 drops down / lifts straight off R44 cleanly once screws & R53-B01 are removed
    SK_T, SK_Z0 = 3.2, OB.TRAY_RIB_Z0
    Z_JOIN = 180.40  # ties over R44 top (Z=178.0) directly into R46 deck (SLOPE_Z >= 181.60)
    # Vertices (24 mm corner wraps)
    for cx_out, cx_in, cx_sign in ((OB.TRAY_X0 - SK_T - 0.05, OB.TRAY_X0 + 24.0, -1), (OB.TRAY_X1 - 24.0, OB.TRAY_X1 + SK_T + 0.05, 1)):
        px0, px1 = (cx_out, OB.TRAY_X0 - 0.05) if cx_sign < 0 else (OB.TRAY_X1 + 0.05, cx_in)
        tx0, tx1 = (cx_out, OB.TRAY_X0 + 2.5) if cx_sign < 0 else (OB.TRAY_X1 - 2.5, cx_in)
        for vy0, vy1, wy0, wy1, y_edge, ysgn in (
            (OB.TRAY_Y0 - SK_T - 0.05, OB.TRAY_Y0 + 24.0, OB.TRAY_Y0 - SK_T - 0.05, OB.TRAY_Y0 - 0.05, OB.TRAY_Y0 + 2.5, -1),
            (OB.TRAY_Y1 - 24.0, OB.TRAY_Y1 + SK_T + 0.05, OB.TRAY_Y1 + 0.05, OB.TRAY_Y1 + SK_T + 0.05, OB.TRAY_Y1 - 2.5, 1),
        ):
            # X-edge side drop + top bridge into R46
            comp = comp + box(px0, px1, vy0, vy1, SK_Z0, Z_JOIN)
            comp = comp + box(tx0, tx1, max(vy0, OB.TRAY_Y0), min(vy1, OB.TRAY_Y1), OB.TRAY_TOP_Z + 0.05, Z_JOIN)
            # Y-edge rear/front drop + top bridge into R46
            ux0, ux1 = (OB.TRAY_X0, OB.TRAY_X0 + 24.0) if cx_sign < 0 else (OB.TRAY_X1 - 24.0, OB.TRAY_X1)
            comp = comp + box(ux0, ux1, wy0, wy1, SK_Z0, Z_JOIN)
            comp = comp + box(ux0, ux1, wy0 if ysgn < 0 else y_edge, y_edge if ysgn < 0 else wy1, OB.TRAY_TOP_Z + 0.05, Z_JOIN)
    # Mid-edge Right (X=270, centered at Y=-157.5)
    comp = comp + box(OB.TRAY_X1 + 0.05, OB.TRAY_X1 + SK_T + 0.05, -175.5, -139.5, SK_Z0, Z_JOIN)
    comp = comp + box(OB.TRAY_X1 - 2.5, OB.TRAY_X1 + SK_T + 0.05, -175.5, -139.5, OB.TRAY_TOP_Z + 0.05, Z_JOIN)
    # Mid-edge Left (X=90, Y=-161.5..-133.5 -> clears R41 boom brace at Y=-175..-164.5 by 3 mm)
    comp = comp + box(OB.TRAY_X0 - SK_T - 0.05, OB.TRAY_X0 - 0.05, -161.5, -133.5, SK_Z0, Z_JOIN)
    comp = comp + box(OB.TRAY_X0 - SK_T - 0.05, OB.TRAY_X0 + 2.5, -161.5, -133.5, OB.TRAY_TOP_Z + 0.05, Z_JOIN)
    # Mid-edge Front (Y=-70, X=148..212 -> under-trench drop + side vertical shoulders flanking center 688ZZ bay)
    comp = comp + box(148.0, 212.0, OB.TRAY_Y1 + 0.05, OB.TRAY_Y1 + SK_T + 0.05, SK_Z0, 175.05)
    for sx0, sx1 in ((148.0, 162.5), (197.5, 212.0)):
        comp = comp + box(sx0, sx1, OB.TRAY_Y1 + 0.05, OB.TRAY_Y1 + SK_T + 0.05, 175.0, Z_JOIN)
        comp = comp + box(sx0, sx1, OB.TRAY_Y1 - 2.5, OB.TRAY_Y1 + SK_T + 0.05, OB.TRAY_TOP_Z + 0.05, Z_JOIN)

    # 5) Removable Under-Tray Rear Cantilever Tie Plate (R53-B01): docks flush beneath R46 (Y=-288..-245, Z=155.7..160.15)
    #    & extends forward under R44 rear rim (Y=-245..-218, Z=155.7..167.85); unbolts downward so R46/R44 separate freely!
    under_tie = box(SCREW_X - 33.0, SCREW_X + 33.0, TIE_Y0 + 0.05, WALL_Y0, CRADLE_Z0, TIE_SPLIT_Z - 0.05)
    with bd.BuildPart() as gus_bp:
        with bd.BuildSketch(bd.Plane.YZ.offset(SCREW_X - 33.0)):
            with bd.BuildLine():
                bd.Polyline(
                    (WALL_Y0, CRADLE_Z0),
                    (-234.5, CRADLE_Z0),
                    (-218.0, 164.80),
                    (-218.0, 167.85),
                    (WALL_Y0, 167.85),
                    (WALL_Y0, CRADLE_Z0),
                )
            bd.make_face()
        bd.extrude(amount=66.0)
    under_tie = under_tie + gus_bp.part

    hw_solids = []

    m3_pts = (
        (OB.TRAY_X0 + 5.0, OB.TRAY_Y1 - 10.0, 179.45, 1.60),
        (OB.TRAY_X1 - 5.0, OB.TRAY_Y1 - 10.0, 179.45, 1.60),
        (OB.TRAY_X0 + 5.0, WALL_Y1 + 8.0, 181.20, 3.00),
        (OB.TRAY_X1 - 5.0, WALL_Y1 + 8.0, 181.20, 3.00),
    )
    for mx, my, z_seat, hd_h in m3_pts:
        comp = comp - OB._cyl_axis(1.70, z0 - 0.5, z_seat + 0.5, "z", mx, my)
        comp = comp - OB._cyl_axis(3.10, z_seat, WALL_TOP_Z + 1.0, "z", mx, my)
        tray_clean = tray_clean - OB._cyl_axis(1.45, OB.TRAY_RIB_Z0 + 2.0, OB.TRAY_TOP_Z + 0.5, "z", mx, my)
        head = OB._cyl_axis(2.75, z_seat + 0.05, z_seat + 0.05 + hd_h, "z", mx, my)
        shank = OB._cyl_axis(1.40, OB.TRAY_RIB_Z0 + 3.0, z_seat + 0.05, "z", mx, my)
        hw_solids.append(head + shank)

    # Horizontal M3 x 10 mm clamping screws tying corner & mid-edge drop skirts firmly into R44 perimeter
    side_m3 = (
        # (axis, x, y, z, out_dir: -1 or +1)
        ("x", OB.TRAY_X0 - SK_T - 0.05, -226.0, 173.0, -1),
        ("x", OB.TRAY_X0 - SK_T - 0.05, -147.5, 173.0, -1),
        ("x", OB.TRAY_X0 - SK_T - 0.05, -88.0, 173.0, -1),
        ("x", OB.TRAY_X1 + SK_T + 0.05, -226.0, 173.0, 1),
        ("x", OB.TRAY_X1 + SK_T + 0.05, -157.5, 173.0, 1),
        ("x", OB.TRAY_X1 + SK_T + 0.05, -88.0, 173.0, 1),
        ("y", 104.0, OB.TRAY_Y0 - SK_T - 0.05, 173.0, -1),
        ("y", 256.0, OB.TRAY_Y0 - SK_T - 0.05, 173.0, -1),
        ("y", 104.0, OB.TRAY_Y1 + SK_T + 0.05, 173.0, 1),
        ("y", 155.0, OB.TRAY_Y1 + SK_T + 0.05, 173.0, 1),
        ("y", 205.0, OB.TRAY_Y1 + SK_T + 0.05, 173.0, 1),
        ("y", 256.0, OB.TRAY_Y1 + SK_T + 0.05, 173.0, 1),
    )
    for ax, sx, sy, sz, odir in side_m3:
        if ax == "x":
            p_in = OB.TRAY_X0 - SK_T - 0.6 if odir < 0 else OB.TRAY_X1 + 0.0
            p_out = OB.TRAY_X0 + 0.0 if odir < 0 else OB.TRAY_X1 + SK_T + 0.6
            comp = comp - OB._cyl_axis(1.65, min(p_in, p_out), max(p_in, p_out), "x", sy, sz)
            tray_clean = tray_clean - OB._cyl_axis(1.45, OB.TRAY_X0 - 0.2 if odir < 0 else OB.TRAY_X1 - 8.0,
                                                   OB.TRAY_X0 + 8.0 if odir < 0 else OB.TRAY_X1 + 0.2, "x", sy, sz)
            hw_solids.append(_socket_bolt(2.70, 2.4, 1.40, 10.0, "x", sx, sy, sz, head_dir=odir))
        else:
            p_in = OB.TRAY_Y0 - SK_T - 0.6 if odir < 0 else OB.TRAY_Y1 + 0.0
            p_out = OB.TRAY_Y0 + 0.0 if odir < 0 else OB.TRAY_Y1 + SK_T + 0.6
            comp = comp - OB._cyl_axis(1.65, min(p_in, p_out), max(p_in, p_out), "y", sx, sz)
            tray_clean = tray_clean - OB._cyl_axis(1.45, OB.TRAY_Y0 - 0.2 if odir < 0 else OB.TRAY_Y1 - 8.0,
                                                   OB.TRAY_Y0 + 8.0 if odir < 0 else OB.TRAY_Y1 + 0.2, "y", sx, sz)
            hw_solids.append(_socket_bolt(2.70, 2.4, 1.40, 10.0, "y", sx, sy, sz, head_dir=odir))

    # 2x rear underside M3 bolts anchoring R53-B01 straight UP into R44 rear rim (Y=-242) + 2x M3 bolts into R46 coupler floor (Y=-275)
    for rx in (155.5, 204.5):
        under_tie = under_tie - OB._cyl_axis(1.65, CRADLE_Z0 - 0.5, 168.2, "z", rx, -242.0)
        under_tie = under_tie - OB._cyl_axis(3.10, CRADLE_Z0 - 0.5, 159.0, "z", rx, -242.0)
        tray_clean = tray_clean - OB._cyl_axis(1.45, OB.TRAY_RIB_Z0 - 0.5, OB.TRAY_TOP_Z - 1.5, "z", rx, -242.0)
        hw_solids.append(_socket_bolt(2.70, 2.4, 1.40, 15.0, "z", rx, -242.0, 161.45, head_dir=-1))

    for ux in (162.0, 198.0):
        under_tie = under_tie - OB._cyl_axis(1.65, CRADLE_Z0 - 0.5, TIE_SPLIT_Z + 0.5, "z", ux, -275.0)
        under_tie = under_tie - OB._cyl_axis(3.10, CRADLE_Z0 - 0.5, 158.0, "z", ux, -275.0)
        comp = comp - OB._cyl_axis(1.45, TIE_SPLIT_Z - 0.5, 161.4, "z", ux, -275.0)
        hw_solids.append(_socket_bolt(2.70, 2.4, 1.40, 6.0, "z", ux, -275.0, 160.45, head_dir=-1))

    kp_yc = (KP08_Y0 + KP08_Y1) / 2.0
    for bx in (SCREW_X - 21.0, SCREW_X + 21.0):
        comp = comp - OB._cyl_axis(2.65, TIE_SPLIT_Z - 0.5, 166.0, "z", bx, kp_yc)
        under_tie = under_tie - OB._cyl_axis(2.65, CRADLE_Z0 - 0.5, TIE_SPLIT_Z + 0.5, "z", bx, kp_yc)
        under_tie = under_tie - OB._cyl_axis(5.10, CRADLE_Z0 - 0.5, 159.8, "z", bx, kp_yc)
        hw_solids.append(_socket_bolt(4.25, 4.5, 2.40, 15.2, "z", bx, kp_yc, 170.60, head_dir=1))
        hw_solids.append(_hex_nut(8.0, 3.6, "z", bx, kp_yc, 155.90, 2.45))

    for sx in (-15.5, 15.5):
        for sz in (-15.5, 15.5):
            nx, nz = SCREW_X + sx, SCREW_Z + sz
            comp = comp - OB._cyl_axis(1.65, BULK_Y0 - 0.5, BULK_Y1 + 0.5, "y", nx, nz)
            hw_solids.append(_socket_bolt(2.75, 2.8, 1.45, 7.5, "y", nx, BULK_Y1 + 0.1, nz, head_dir=1))

    for fx in (SCREW_X - 12.6, SCREW_X + 12.6):
        comp = comp - OB._cyl_axis(1.65, 178.0, 179.5, "z", fx, -74.0)
        tray_clean = tray_clean - OB._cyl_axis(1.65, OB.TRAY_RIB_Z0 - 0.5, OB.TRAY_TOP_Z + 0.5, "z", fx, -74.0)
        hw_solids.append(_socket_bolt(2.70, 2.8, 1.45, 14.5, "z", fx, -74.0, 182.25, head_dir=1))
        hw_solids.append(_hex_nut(5.5, 2.4, "z", fx, -74.0, 163.40, 1.50))

    # 6) Removable Screwed Top Motor + Fork-Blade Tail Enclosure Lid (R56-B01):
    #    Covers BOTH the open-top 17HS4401/coupler/KP08 motor bay AND both open-top left/right fork rack-tail channels
    #    across X=127.6..232.4, Y=-338.9..-245.0, bolted down via 6x M3 x 10 mm socket cap screws.
    LID_X0, LID_X1 = OB.BXR - FORK_DX - REAR_OUT_PAD, OB.BXR + FORK_DX + REAR_OUT_PAD  # 127.6..232.4
    LID_Z0, LID_Z1 = WALL_TOP_Z + 0.05, WALL_TOP_Z + 5.20  # 200.05..205.20 (0.05 mm clear of R46 Z=200.00 rim)
    top_cover = (
        box(LID_X0, LID_X1, REAR_CAP_Y0, WALL_Y0, LID_Z0, LID_Z1)
        - box(SCREW_X - 33.2, SCREW_X + 33.2, REAR_CAP_Y0 - 0.2, BULK_Y1 + 0.3, LID_Z0 - 0.2, 202.05)  # clears Z=201.80 NEMA walls
    ) + box(SCREW_X - 9.5, SCREW_X + 9.5, REAR_CAP_Y0, BULK_Y0 - 26.5, LID_Z1 - 0.4, 209.00)  # raised JST connector hood
    # Internal JST socket & rear 4-wire exit tunnel underneath hood (201.80..206.80 -> clears 206.15 JST plug)
    top_cover = top_cover - box(SCREW_X - 6.8, SCREW_X + 6.8, REAR_CAP_Y0 - 0.2, BULK_Y0 - 29.0, 201.50, 206.80)

    cover_hw = []
    lid_m3_pts = (
        (LID_X0 + 3.4, -252.0), (LID_X1 - 3.4, -252.0),   # front outer cheeks (X=131.0, 229.0)
        (LID_X0 + 3.4, -331.5), (LID_X1 - 3.4, -331.5),   # rear outer cheeks (X=131.0, 229.0)
        (147.5, -286.0), (212.5, -286.0),                  # center pylons between motor bay & rack channels
    )
    for lx, ly in lid_m3_pts:
        top_cover = top_cover - OB._cyl_axis(1.70, LID_Z0 - 0.5, LID_Z1 + 0.5, "z", lx, ly)
        top_cover = top_cover - OB._cyl_axis(3.05, 202.40, LID_Z1 + 0.5, "z", lx, ly)
        comp = comp - OB._cyl_axis(1.45, 191.00, WALL_TOP_Z + 0.5, "z", lx, ly)
        cover_hw.append(_socket_bolt(2.70, 2.40, 1.40, 10.0, "z", lx, ly, 202.45, head_dir=1))

    # Engrave R46-B01 on front face of stop wall, R53-B01 on under-tie, and R56-B01 on top lid deck
    comp_part = _engrave_verified(
        comp, "forklift_deck", (OB.BXR - 52.0, WALL_Y1, 193.0), (1, 0, 0), (0, 1, 0), 4.2
    )
    under_tie_part = _engrave_verified(
        under_tie, "under_tie", (SCREW_X - 18.0, -262.0, CRADLE_Z0), (1, 0, 0), (0, 0, -1), 4.2
    )
    top_cover_part = _engrave_verified(
        top_cover, "fork_top_cover", (SCREW_X - 18.0, -270.0, LID_Z1), (1, 0, 0), (0, 0, 1), 4.5
    )
    return tray_clean, comp_part, under_tie_part, hw_solids, top_cover_part, cover_hw


def build_forklift_mechanism():
    """Build printed R47..R55 parts + Horizontal 1:2 Rack-and-Pinion (2x F683ZZ + CNC4759-A/MT0382)."""
    box = OB._box
    f04 = []
    moving_hw = []

    # R47 & R48: Left & Right Fixed Base Guide Tracks + Elevated MOD=1.0 Fixed Rack + Dual Plastic Dead-Stops (1 solid each!)
    rails = {}
    HOLD_Y_M3 = (-226.0, -117.5, -75.5)
    for key, sgn in (("rail_left", -1), ("rail_right", 1)):
        xc = OB.BXR + sgn * FORK_DX
        outer_x0 = (xc - 11.0) if sgn < 0 else (xc + 5.6)  # 121.0..126.4 (L) / 233.6..239.0 (R) -> 5.4 mm solid for M3
        outer_x1 = (xc - 5.6) if sgn < 0 else (xc + 11.0)
        # Base Rack sits at GEAR_Z0..GEAR_Z1 (183.5..187.2); risers act as ±1.5 mm plastic hard-stops behind switches!
        x_pitch_base = OB.BXR + sgn * (FORK_DX - PIN_DX_CENTER - PIN_RP)  # 156.5 (L) / 203.5 (R)
        spine_x0 = (x_pitch_base + 1.25 * RP_MOD) if sgn < 0 else (x_pitch_base - 1.25 * RP_MOD - 4.5)
        spine_x1 = (spine_x0 + 4.5) if sgn < 0 else (x_pitch_base - 1.25 * RP_MOD)
        floor_x0 = min(xc - 8.0, spine_x0)
        floor_x1 = max(xc + 8.0, spine_x1)
        base_rack = _build_horizontal_rack(
            x_pitch_base, spine_x1 if sgn < 0 else spine_x0, -1 if sgn < 0 else 1,
            -237.2, -70.6, GEAR_Z0, GEAR_Z1, y_phase=PIN_Y_HOME,
        )
        riser_rear = box(floor_x0, floor_x1, -237.2, -217.0, 180.70, 181.10) + box(
            spine_x0, spine_x1, -237.2, -217.0, 181.00, GEAR_Z1
        )
        riser_front = box(floor_x0, floor_x1, -82.2, -70.6, 180.70, 181.10) + box(
            spine_x0, spine_x1, -82.2, -70.6, 181.00, GEAR_Z1
        )
        rail = (
            box(outer_x0, outer_x1, -237.2, -70.6, 180.70, 185.00)
            + box(xc - 8.0, xc + 8.0, -237.2, -70.6, 180.70, 181.10)
            + base_rack + riser_rear + riser_front
        )
        mx_hole = 0.5 * (outer_x0 + outer_x1)  # 123.7 / 236.3
        for my_h in HOLD_Y_M3:
            rail = rail - OB._cyl_axis(1.45, 181.00, 185.40, "z", mx_hole, my_h)
        face_x = outer_x0 if sgn < 0 else outer_x1
        x_dir = (0, 1, 0) if sgn < 0 else (0, -1, 0)
        norm = (-1, 0, 0) if sgn < 0 else (1, 0, 0)
        rails[key] = _engrave_verified(rail, key, (face_x, -154.0, 182.85), x_dir, norm, 2.2)

    # R49: Front 688ZZ Arch Saddle with flattened 189.60 mm crown (still -0.70 mm below raised 190.30 mm orange forks)
    front_arch = (
        (
            box(SCREW_X - 16.0, SCREW_X + 16.0, -77.5, -70.5, 179.10, 182.20)
            + OB._cyl_axis(9.2, -77.5, -70.5, "y", SCREW_X, SCREW_Z)
        )
        & box(SCREW_X - 16.2, SCREW_X + 16.2, -78.0, -70.0, 179.10, 189.60)
    ) - OB._cyl_axis(8.05, -77.8, -70.2, "y", SCREW_X, SCREW_Z)
    for fx in (SCREW_X - 12.6, SCREW_X + 12.6):
        front_arch = front_arch - OB._cyl_axis(1.65, 178.5, 183.0, "z", fx, -74.0)
    r49_arch = _engrave_verified(front_arch, "front_688zz_arch", (SCREW_X, -74.0, 189.60), (1, 0, 0), (0, 0, 1), 2.3, id_only=True)

    # R50: Reinforced Stage-1 C-Channel H-Carriage + Side Glide Shelf + Dual R51 Hold-Down Hooks + Trench Switch Tab
    r50_parts = []
    pinions = {}
    gear_pts = O.involute_gear_pts(RP_Z, RP_MOD, RP_PA, backlash=0.14)
    for pkey, sgn in (("pinion_left", -1), ("pinion_right", 1)):
        xc = OB.BXR + sgn * FORK_DX
        x_pin = OB.BXR + sgn * (FORK_DX - PIN_DX_CENTER)  # 148.5 (L) / 211.5 (R)
        # Main rail plate (2.0 mm thick) + outboard low glide shelf for R57A / R57B (X=126.7..129.0 L / 231.0..233.3 R)
        x_flat0 = (xc - 5.3) if sgn < 0 else (xc - 6.6)
        x_flat1 = (xc + 6.6) if sgn < 0 else (xc + 5.3)
        r50_parts.append(box(x_flat0, x_flat1, -234.0, -71.5, 181.30, 183.30))
        # Alignment key (clear of R51 rack spine 135.0/225.0) engaging underside slot on R51/R52
        x_key = xc - sgn * 1.5  # 133.5 (L) / 226.5 (R)
        r50_parts.append(box(x_key - 0.7, x_key + 0.7, -234.0, -78.5, 183.10, 184.60))
        # Rigid outer vertical C-wall (Z=181.30..189.90) + dual top L-hooks holding R51/R52 down along entire 0..+240 stroke
        cw0, cw1 = ((xc - 2.8, xc - 1.2) if sgn < 0 else (xc + 1.2, xc + 2.8))  # 129.2..130.8 (L) / 229.2..230.8 (R)
        r50_parts.append(box(cw0, cw1, -234.0, -71.5, 181.30, 189.90))
        hk0, hk1 = ((cw0, xc + 0.2) if sgn < 0 else (xc - 0.2, cw1))  # 129.2..132.2 (L) / 227.8..230.8 (R)
        for hy0, hy1 in ((-112.0, -98.0), (-84.0, -71.5)):
            r50_parts.append(box(hk0, hk1, hy0, hy1, 186.90, 189.90))

        # Boss pad rising from cross-bridge forward to Y_PIN=-202 with M3 threaded bore
        boss = (
            OB._cyl_axis(5.2, 181.30, 183.35, "z", x_pin, PIN_Y_HOME)
            + box(x_pin - 4.8, x_pin + 4.8, -210.0, PIN_Y_HOME, 181.30, 183.10)
        ) - OB._cyl_axis(1.48, 181.00, 183.60, "z", x_pin, PIN_Y_HOME)
        r50_parts.append(boss)

        # Horizontal 16T Spur Pinion (R54 Left / R55 Right) at GEAR_Z0..GEAR_Z1 (strictly lower than raised forks!)
        with bd.BuildPart() as gp:
            with bd.BuildSketch(bd.Plane.XY.offset(GEAR_Z0)):
                with bd.BuildLine():
                    bd.Polyline(*gear_pts, gear_pts[0])
                bd.make_face()
            bd.extrude(amount=GEAR_Z1 - GEAR_Z0)
        pin_raw = gp.part.moved(bd.Location((x_pin, PIN_Y_HOME, 0.0)))
        pin_raw = pin_raw - OB._cyl_axis(3.55, GEAR_Z0 - 0.5, GEAR_Z1 + 0.5, "z", x_pin, PIN_Y_HOME)
        pin_raw = pin_raw - OB._cyl_axis(4.10, GEAR_Z1 - 0.90, GEAR_Z1 + 0.5, "z", x_pin, PIN_Y_HOME)
        pinions[pkey] = _engrave_verified(pin_raw, pkey, (x_pin, PIN_Y_HOME - 5.4, GEAR_Z1), (1, 0, 0), (0, 0, 1), 1.6, id_only=True)

        # Bought hardware: 1x F683ZZ flanged bearing (183.55..187.15) + low-profile M3 cap screw (top Z <= 188.35 < FORK_Z1=190.30!)
        f683 = (
            OB._cyl_axis(3.48, GEAR_Z0 + 0.05, GEAR_Z1 - 0.85, "z", x_pin, PIN_Y_HOME)
            + OB._cyl_axis(4.02, GEAR_Z1 - 0.85, GEAR_Z1 - 0.05, "z", x_pin, PIN_Y_HOME)
        ) - OB._cyl_axis(1.55, GEAR_Z0 - 0.2, GEAR_Z1 + 0.2, "z", x_pin, PIN_Y_HOME)
        moving_hw.append(f683)
        moving_hw.append(_socket_bolt(2.50, 1.20, 1.42, 6.2, "z", x_pin, PIN_Y_HOME, GEAR_Z1 - 0.05, head_dir=1))

    # Flat horizontal cross-wings (Y=-215.5..-203.7, 11.8 mm deep, Z=181.30..183.10): clear copper flange via -215.5..-207.8 neck
    r50_parts.append(box(OB.BXR - FORK_DX + 5.8, SCREW_X - 11.4, -215.5, -203.7, 181.30, 183.10))
    r50_parts.append(box(SCREW_X + 11.4, OB.BXR + FORK_DX - 5.8, -215.5, -203.7, 181.30, 183.10))
    r50_parts.append(box(SCREW_X - 11.5, SCREW_X - 10.8, -215.5, -207.8, 181.30, 183.10))
    r50_parts.append(box(SCREW_X + 10.8, SCREW_X + 11.5, -215.5, -207.8, 181.30, 183.10))
    nut_plate = box(SCREW_X - 11.2, SCREW_X + 11.2, -215.5, -207.7, 175.60, 187.00) - OB._cyl_axis(
        5.30, -216.0, -207.2, "y", SCREW_X, SCREW_Z
    )
    for hx in (SCREW_X - 8.0, SCREW_X + 8.0):
        nut_plate = nut_plate - OB._cyl_axis(1.65, -216.0, -207.2, "y", hx, SCREW_Z)
    r50_parts.append(nut_plate)

    r50_raw = r50_parts[0]
    for p in r50_parts[1:]:
        r50_raw = r50_raw + p
    r50_carriage = _engrave_verified(
        r50_raw, "stage1_carriage", (SCREW_X - 21.5, -212.8, 183.10), (1, 0, 0), (0, 0, 1), 2.5
    )

    # Bought Copper T8 Nut + 2x M3 Flange Bolts & Hex Nuts
    flange = OB._cyl_axis(11.0, -207.5, -204.0, "y", SCREW_X, SCREW_Z) & box(
        SCREW_X - 11.5, SCREW_X + 11.5, -207.6, -203.9, 175.60, 185.40
    )
    barrel = OB._cyl_axis(5.10, -218.0, -202.5, "y", SCREW_X, SCREW_Z)
    copper_nut = (flange + barrel) - OB._cyl_axis(4.20, -218.5, -202.0, "y", SCREW_X, SCREW_Z)
    for hx in (SCREW_X - 8.0, SCREW_X + 8.0):
        copper_nut = copper_nut - OB._cyl_axis(1.65, -208.0, -203.5, "y", hx, SCREW_Z)
        moving_hw.append(_socket_bolt(2.55, 2.5, 1.45, 11.5, "y", hx, -203.9, SCREW_Z, head_dir=1))
        moving_hw.append(_hex_nut(5.5, 2.4, "y", hx, -218.1, SCREW_Z, 1.50))

    # R51 & R52: Twin Tapered Forklift Tines with Outer Stepped L-Shoulder (captured by R50 L-hooks at Z=186.9..189.9)
    tines = {}
    for key, sgn in (("fork_tine_left", -1), ("fork_tine_right", 1)):
        xc = OB.BXR + sgn * FORK_DX
        # Lower L-shoulder (131.1..138.6 L / 221.4..228.9 R @ Z=183.5..186.5) + Upper tall beam (132.6..138.6 @ Z=186.4..190.3)
        bl0_lo, bl1_lo = ((xc - 0.9, xc + 6.6) if sgn < 0 else (xc - 6.6, xc + 0.9))
        bl0_hi, bl1_hi = ((xc + 0.6, xc + 6.6) if sgn < 0 else (xc - 6.6, xc - 0.6))
        x_key = xc - sgn * 1.5
        blade = (
            box(bl0_lo, bl1_lo, -234.0, -78.0, FORK_Z0, 186.50)
            + box(bl0_hi, bl1_hi, -234.0, -78.0, 186.40, FORK_Z1)
        ) - box(x_key - 1.0, x_key + 1.0, -234.5, -77.5, FORK_Z0 - 0.1, 185.00)
        x_pitch_tine = OB.BXR + sgn * (FORK_DX - PIN_DX_TINE)  # 140.5 (L) / 219.5 (R)
        tine_rack = _build_horizontal_rack(
            x_pitch_tine, xc - sgn * 3.0, 1 if sgn < 0 else -1,
            -335.0, -78.0, GEAR_Z0, GEAR_Z1, y_phase=PIN_Y_HOME,
        )
        with bd.BuildPart() as tn:
            with bd.BuildSketch(bd.Plane.YZ.offset(bl0_hi)):
                with bd.BuildLine():
                    bd.Polyline(
                        (-78.2, FORK_Z0), (-63.0, FORK_Z0),
                        (-63.0, FORK_Z0 + 1.5), (-78.2, FORK_Z1), (-78.2, FORK_Z0)
                    )
                bd.make_face()
            bd.extrude(amount=bl1_hi - bl0_hi)
        tine_raw = blade + tine_rack + tn.part
        tines[key] = _engrave_verified(tine_raw, key, (xc - sgn * 3.6, -185.0, FORK_Z1), (0, 1, 0), (0, 0, 1), 2.4)

    # Option A (R57A-B01): Screwed Full-Length Side T-Slot Hold-Down Rails (Z=185.05..190.10, clears 192.00 crate!)
    opt_a_parts, opt_a_hw = {}, []
    for key, sgn in (("opt_a_left", -1), ("opt_a_right", 1)):
        xc = OB.BXR + sgn * FORK_DX
        ox0, ox1 = ((xc - 11.0, xc - 5.6) if sgn < 0 else (xc + 5.6, xc + 11.0))  # 121.0..126.4 / 233.6..239.0 (5.4 mm solid!)
        lx0, lx1 = ((xc - 5.5, xc - 3.2) if sgn < 0 else (xc + 3.2, xc + 5.5))    # inner lip ONLY 126.5..128.8 / 231.2..233.5 (clears R47!)
        rail_a = (
            box(ox0, ox1, -237.2, -70.6, 185.05, 190.10)
            + box(min(ox0, lx0), max(ox1, lx1), -237.2, -70.6, 186.60, 190.10)  # top bridge tying mount + L-lip
            + box(lx0, lx1, -237.2, -70.6, 183.60, 190.10)
        )
        mx_h = 0.5 * (ox0 + ox1)  # 123.7 / 236.3
        for my_h in HOLD_Y_M3:
            rail_a = rail_a - OB._cyl_axis(1.60, 184.8, 190.4, "z", mx_h, my_h)
            rail_a = rail_a - OB._cyl_axis(2.55, 188.3, 190.4, "z", mx_h, my_h)
            opt_a_hw.append(_socket_bolt(2.45, 1.60, 1.40, 6.2, "z", mx_h, my_h, 188.35, head_dir=1))
        face_x = ox0 if sgn < 0 else ox1
        x_dir = (0, 1, 0) if sgn < 0 else (0, -1, 0)
        norm = (-1, 0, 0) if sgn < 0 else (1, 0, 0)
        opt_a_parts[key] = _engrave_verified(rail_a, key, (face_x, -154.0, 187.6), x_dir, norm, 2.2)

    # Option B (R57B-B01): Screwed Front+Mid Hold-Down Twin MR63ZZ Bearing Brackets (4x Ø6xØ3x2.5 mm bearings, Z<=190.10)
    opt_b_parts, opt_b_hw = {}, []
    ROLL_YS = (-111.0, -83.0)
    for key, sgn in (("opt_b_left", -1), ("opt_b_right", 1)):
        xc = OB.BXR + sgn * FORK_DX
        ox0, ox1 = ((xc - 11.0, xc - 5.6) if sgn < 0 else (xc + 5.6, xc + 11.0))  # 121.0..126.4 / 233.6..239.0 (5.4 mm solid!)
        bx0, bx1 = ((xc - 5.5, xc - 3.1) if sgn < 0 else (xc + 3.1, xc + 5.5))    # bearing pocket X=126.5..128.9 / 231.1..233.5
        br_b = box(ox0, ox1, -122.0, -70.6, 185.05, 190.10)
        mx_h = 0.5 * (ox0 + ox1)
        for my_h in (-117.5, -75.5):
            br_b = br_b - OB._cyl_axis(1.60, 184.8, 190.4, "z", mx_h, my_h)
            br_b = br_b - OB._cyl_axis(2.55, 188.3, 190.4, "z", mx_h, my_h)
            opt_b_hw.append(_socket_bolt(2.45, 1.60, 1.40, 6.2, "z", mx_h, my_h, 188.35, head_dir=1))
        for ry in ROLL_YS:
            br_b = br_b - OB._cyl_axis(1.60, ox0 - 0.5, ox1 + 0.5, "x", ry, 186.55)
            # Outer race MR63ZZ (OD=6.0 mm, width=2.4 mm) riding at Z=183.55..189.55 (0.25 mm above R50 shelf 183.30)
            mr63 = OB._cyl_axis(3.00, bx0, bx1, "x", ry, 186.55) - OB._cyl_axis(1.50, bx0 - 0.2, bx1 + 0.2, "x", ry, 186.55)
            opt_b_hw.append(mr63)
            pin_x0 = ox0 if sgn < 0 else ox1
            opt_b_hw.append(_socket_bolt(2.20, 1.40, 1.42, 7.9, "x", pin_x0, ry, 186.55, head_dir=sgn))
        face_x = ox0 if sgn < 0 else ox1
        x_dir = (0, 1, 0) if sgn < 0 else (0, -1, 0)
        norm = (-1, 0, 0) if sgn < 0 else (1, 0, 0)
        txt_y = -104.5 if sgn < 0 else -89.5
        opt_b_parts[key] = _engrave_verified(br_b, key, (face_x, txt_y, 187.0), x_dir, norm, 1.9)

    # 2x Center-Trench End-Stop Limit Switches (Home @ Y=-225, Max-Extend @ Y=-83) recessed in left trench flank
    sw_solids = []
    for sw_y0, sw_y1, lvy0, lvy1 in (
        (-228.5, -216.2, -217.5, -215.8),  # Home switch body + rear actuation plunger
        (-87.3,  -78.3,  -87.9,  -87.4),   # Max-Extend detector switch body + front actuation plunger
    ):
        sw_solids.append(box(SCREW_X - 17.5, SCREW_X - 12.6, sw_y0, sw_y1, 174.0, 180.2))  # switch housing
        sw_solids.append(box(SCREW_X - 13.0, SCREW_X - 11.7, lvy0, lvy1, 176.5, 179.4))    # roller / actuator tab
    limit_switches = bd.Compound(children=sw_solids, label="Center_Trench_Limit_Switches_2x")

    # F04: Realistic CNC4759-A T8 Screw + KP08 + Slotted Coupler + 688ZZ + MT0382 17HS4401 NEMA 17
    f04.append(_build_t8_threaded_screw(-274.0, -72.0, SCREW_X, SCREW_Z))
    f04.append(OB._cyl_axis(7.90, -76.5, -71.5, "y", SCREW_X, SCREW_Z) - OB._cyl_axis(4.20, -77.0, -71.0, "y", SCREW_X, SCREW_Z))
    f04.append(_build_kp08_pillow(KP08_Y0, KP08_Y1, SCREW_X, SCREW_Z))
    f04.append(_build_slotted_coupler(COUP_Y0, COUP_Y1, SCREW_X, SCREW_Z))
    hw, ch = 21.15, 5.2
    oct_xz = [
        (SCREW_X - hw + ch, SCREW_Z - hw), (SCREW_X + hw - ch, SCREW_Z - hw),
        (SCREW_X + hw, SCREW_Z - hw + ch), (SCREW_X + hw, SCREW_Z + hw - ch),
        (SCREW_X + hw - ch, SCREW_Z + hw), (SCREW_X - hw + ch, SCREW_Z + hw),
        (SCREW_X - hw, SCREW_Z + hw - ch), (SCREW_X - hw, SCREW_Z - hw + ch),
    ]
    nema = OB._extrude_xz_poly(oct_xz, BULK_Y0 - 40.2, BULK_Y0 - 0.2)
    for gy0, gy1 in ((BULK_Y0 - 31.0, BULK_Y0 - 30.2), (BULK_Y0 - 10.0, BULK_Y0 - 9.2)):
        nema = nema - (box(SCREW_X - 22.0, SCREW_X + 22.0, gy0, gy1, SCREW_Z - 22.0, SCREW_Z + 22.0)
                       - OB._cyl_axis(24.0, gy0 - 0.2, gy1 + 0.2, "y", SCREW_X, SCREW_Z))
    nema = nema + box(SCREW_X - 6.0, SCREW_X + 6.0, BULK_Y0 - 38.0, BULK_Y0 - 30.0, SCREW_Z + hw - 0.2, SCREW_Z + hw + 4.5)
    for sx in (-15.5, 15.5):
        for sz in (-15.5, 15.5):
            nema = nema - OB._cyl_axis(1.65, BULK_Y0 - 6.0, BULK_Y0 + 0.2, "y", SCREW_X + sx, SCREW_Z + sz)
    f04.append(nema)
    f04.append(OB._cyl_axis(11.0, BULK_Y0 - 0.2, BULK_Y0 + 1.8, "y", SCREW_X, SCREW_Z))
    d_shaft = OB._cyl_axis(2.50, BULK_Y0 - 0.2, COUP_Y0 + 12.0, "y", SCREW_X, SCREW_Z) - box(
        SCREW_X - 3.0, SCREW_X + 3.0, BULK_Y0 + 2.5, COUP_Y0 + 12.5, SCREW_Z + 2.0, SCREW_Z + 3.0
    )
    f04.append(d_shaft)

    return {
        "fork_base_rails": bd.Compound(children=[rails["rail_left"], rails["rail_right"]], label="R47_R48_Base_Rails"),
        "fork_front_arch": r49_arch,
        "fork_mid_stage": r50_carriage,
        "fork_pinion_left": pinions["pinion_left"],
        "fork_pinion_right": pinions["pinion_right"],
        "fork_t8_nut": copper_nut,
        "fork_mid_fasteners": bd.Compound(children=moving_hw, label="R50_Pinion_F683ZZ_And_Nut_HW"),
        "fork_tines": bd.Compound(children=[tines["fork_tine_left"], tines["fork_tine_right"]], label="R51_R52_Fork_Tines"),
        "fork_drive": bd.Compound(children=f04, label="CNC4759A_T8_KP08_MT0382_17HS4401"),
        "fork_limit_switches": limit_switches,
        "fork_opt_a_tslot": bd.Compound(children=[opt_a_parts["opt_a_left"], opt_a_parts["opt_a_right"]], label="R57A_Side_TSlot_Rails"),
        "fork_opt_a_hw": bd.Compound(children=opt_a_hw, label="R57A_M3_Screws"),
        "fork_opt_b_rollers": bd.Compound(children=[opt_b_parts["opt_b_left"], opt_b_parts["opt_b_right"]], label="R57B_MR63ZZ_Roller_Brackets"),
        "fork_opt_b_hw": bd.Compound(children=opt_b_hw, label="R57B_MR63ZZ_Bearings_And_Screws"),
        "_print_solids": {
            "rail_left": rails["rail_left"],
            "rail_right": rails["rail_right"],
            "front_688zz_arch": r49_arch,
            "stage1_carriage": r50_carriage,
            "fork_tine_left": tines["fork_tine_left"],
            "fork_tine_right": tines["fork_tine_right"],
            "pinion_left": pinions["pinion_left"],
            "pinion_right": pinions["pinion_right"],
            "opt_a_left": opt_a_parts["opt_a_left"],
            "opt_a_right": opt_a_parts["opt_a_right"],
            "opt_b_left": opt_b_parts["opt_b_left"],
            "opt_b_right": opt_b_parts["opt_b_right"],
        },
    }


def build_objects_base_and_forklift():
    """Assemble base stack (R41 + TAL220 + R45 + R44) + R46 deck + R47..R52 forklift + fasteners."""
    boom, live_wish, load_cell, lc_hw = OB.build_tray_boom_and_loadcell()
    picker = OB.build_tray_and_picker()
    tray_clean = picker["tray_deck"] & OB._box(
        OB.TRAY_X0 - 1.0, OB.TRAY_X1 + 1.0, OB.TRAY_Y0 - 1.0, OB.TRAY_Y1 + 1.0,
        OB.TRAY_RIB_Z0 - 1.0, OB.TRAY_TOP_Z
    )
    tray_clean, comp_deck, under_tie, comp_hw, top_cover, cover_hw = build_forklift_deck(tray_clean)
    all_hw = bd.Compound(children=[*lc_hw.solids(), *comp_hw], label="Base_And_Compartment_Hardware")
    cover_fasteners = bd.Compound(children=cover_hw, label="R56_Cover_M3_Screws")
    fork_parts = build_forklift_mechanism()

    box = OB._box
    ly0, ly1 = -236.0, -106.0
    skid_l = box(OB.BXR - 70.0, OB.BXR - 59.0, ly0, ly1, 183.60, 195.60)
    skid_r = box(OB.BXR + 59.0, OB.BXR + 70.0, ly0, ly1, 183.60, 195.60)
    crate = box(OB.BXR - 70.0, OB.BXR + 70.0, ly0, ly1, 195.60, 263.60)
    ref_load = bd.Compound(children=[skid_l, skid_r, crate], label="Ref_Skidded_Load_140x130x80")

    _, ref_rack = OB.build_reference_env()
    print_solids = fork_parts.pop("_print_solids")
    return {
        "base_boom": boom,
        "load_cell": load_cell,
        "live_wishbone": live_wish,
        "tray_deck": tray_clean,
        "compartment_deck": comp_deck,
        "fork_under_tie": under_tie,
        "fork_top_cover": top_cover,
        "fork_cover_fasteners": cover_fasteners,
        "base_fasteners": all_hw,
        **fork_parts,
        "ref_load": ref_load,
        "ref_rack_bay": ref_rack,
        "_print_solids": {"forklift_deck": comp_deck, "under_tie": under_tie, "fork_top_cover": top_cover, **print_solids},
    }


def check_forklift_clearances(parts: dict) -> bool:
    """Verify 0 solid collisions across base + R46..R57 + switches at Home (0 mm) & +240 mm for Option A and Option B."""
    base_keys = (
        "base_boom", "load_cell", "live_wishbone", "tray_deck", "compartment_deck",
        "fork_under_tie", "fork_top_cover", "fork_cover_fasteners", "base_fasteners",
        "fork_base_rails", "fork_front_arch", "fork_mid_stage", "fork_limit_switches",
        "fork_pinion_left", "fork_pinion_right", "fork_t8_nut", "fork_mid_fasteners", "fork_tines", "fork_drive",
    )
    bad = []
    for opt_lbl, opt_keys in (
        ("optA_tslot", ("fork_opt_a_tslot", "fork_opt_a_hw")),
        ("optB_rollers", ("fork_opt_b_rollers", "fork_opt_b_hw")),
    ):
        keys = (*base_keys, *opt_keys)
        for i, k1 in enumerate(keys):
            for k2 in keys[i + 1:]:
                v = _overlap(parts[k1], parts[k2])
                if v > 1e-3:
                    bad.append((f"home_{opt_lbl}", k1, k2, v))

        fixed_plus_f04 = bd.Compound(children=[parts[k] for k in (
            "base_boom", "load_cell", "live_wishbone", "tray_deck", "compartment_deck",
            "fork_under_tie", "fork_top_cover", "fork_cover_fasteners", "base_fasteners",
            "fork_base_rails", "fork_front_arch", "fork_drive", "fork_limit_switches", *opt_keys,
        )])
        mid_ext = {k: parts[k].moved(bd.Location((0.0, MID_STROKE_MAX, 0.0)))
                   for k in ("fork_mid_stage", "fork_t8_nut", "fork_mid_fasteners")}
        rot_deg = math.degrees(MID_STROKE_MAX / PIN_RP)
        for pk, sgn, ang in (("fork_pinion_left", -1, -rot_deg), ("fork_pinion_right", 1, rot_deg)):
            xp = OB.BXR + sgn * (FORK_DX - PIN_DX_CENTER)
            p_rot = parts[pk].moved(bd.Location((-xp, -PIN_Y_HOME, 0.0)))
            p_rot = p_rot.moved(bd.Location((0.0, 0.0, 0.0), (0.0, 0.0, ang)))
            mid_ext[pk] = p_rot.moved(bd.Location((xp, PIN_Y_HOME + MID_STROKE_MAX, 0.0)))
        f03_ext = parts["fork_tines"].moved(bd.Location((0.0, 2.0 * MID_STROKE_MAX, 0.0)))

        for name, sh in (*mid_ext.items(), ("f03_ext", f03_ext)):
            v = _overlap(sh, fixed_plus_f04)
            if v > 1e-3:
                bad.append((f"ext240_{opt_lbl}", name, "fixed_base", v))
        if opt_lbl == "optA_tslot":
            for k_ext, sh_ext in mid_ext.items():
                v_23 = _overlap(sh_ext, f03_ext)
                if v_23 > 1e-3:
                    bad.append(("extended_240mm", k_ext, "f03_ext", v_23))

    f03_shelf = f03_ext.moved(bd.Location((0.0, 0.0, 10.0)))
    v_rack = _overlap(f03_shelf, parts["ref_rack_bay"])
    if v_rack > 1e-3:
        bad.append(("extended_lifted_10mm", "f03_tines", "ref_rack_bay", v_rack))

    # Verify orange fork blades (FORK_Z1 = 190.30 mm) stay clear of cogwheels, screws, arch, AND hold-down options <= 190.10 mm
    z_fork_top = parts["fork_tines"].bounding_box().max.Z
    for low_k in ("fork_base_rails", "fork_front_arch", "fork_pinion_left", "fork_pinion_right", "fork_mid_fasteners", "fork_t8_nut"):
        z_low = parts[low_k].bounding_box().max.Z
        if z_low >= z_fork_top - 0.4:
            bad.append(("z_clearance", low_k, f"max.Z={z_low:.2f} >= fork_top={z_fork_top:.2f}", z_low - z_fork_top))
    for opt_k in ("fork_opt_a_tslot", "fork_opt_b_rollers", "fork_opt_b_hw"):
        z_opt = parts[opt_k].bounding_box().max.Z
        if z_opt >= 190.25:
            bad.append(("crate_clearance", opt_k, f"max.Z={z_opt:.2f} >= 190.25", z_opt - 190.25))

    if bad:
        for item in bad:
            print("  COLLISION:", item)
    else:
        print("RESULT: OK - 0 collisions at Home & +240 mm across BOTH Option A (Side T-Slot) & Option B (MR63ZZ Rollers) + Limit Switches")
    return len(bad) == 0


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    os.makedirs(SIM_MESH_DIR, exist_ok=True)

    for old in glob.glob(os.path.join(SIM_MESH_DIR, "*.stl")):
        os.remove(old)

    parts = build_objects_base_and_forklift()
    print_solids = parts.pop("_print_solids")
    for lbl in (
        "base_boom", "live_wishbone", "tray_deck", "compartment_deck", "fork_under_tie", "fork_top_cover",
        "fork_front_arch", "fork_mid_stage", "fork_pinion_left", "fork_pinion_right", "fork_t8_nut",
    ):
        assert len(parts[lbl].solids()) == 1, f"{lbl} expected 1 solid"

    for name, shape in parts.items():
        bd.export_stl(shape, os.path.join(SIM_MESH_DIR, f"{name}.stl"), tolerance=0.03, angular_tolerance=0.15)

    # Export clean 1-solid-per-file printable STLs stamped with CLAUDE.md R46..R56 + B01 IDs
    for old_p in glob.glob(os.path.join(OUT_DIR, "*.stl")):
        os.remove(old_p)
    for k, sh in print_solids.items():
        pid = PART_IDS[k]
        bd.export_stl(sh, os.path.join(OUT_DIR, f"{pid}_{k}_{BATCH}.stl"), tolerance=0.01, angular_tolerance=0.1)
    print(f"Exported {len(parts)} viewer meshes + {len(print_solids)} printable R46..R56-{BATCH} STLs")

    asm = bd.Compound(
        children=[v for k, v in parts.items() if not k.startswith("ref_")],
        label="Objects_Compartment_Forklift_CNC4759A_R46_R56",
    )
    step_path = os.path.join(OUT_DIR, "objects_forklift_assembly.step")
    bd.export_step(asm, step_path)
    print(f"Exported STEP -> {step_path}")

    ok = check_forklift_clearances(parts)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
