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
  - R57: Screwed MGN/T-Slot Hold-Down Linear Guide Rails (L/R, mark: R57-B01)
  - R50 <-> R51/R52 Interlock: 55-deg Steepened Dovetail MGN Linear Rail + Wrap-Around Carriage
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
    "forklift_deck": "R46A",
    "fork_rear_cradle": "R46B",
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
    "hold_rail_left": "R57",
    "hold_rail_right": "R57",
    "fork_bay_cover": "R58",
}
# Distinct non-adjacent hex shades (CLAUDE.md §4)
PART_COLORS = {
    "R46A": "#26A69A",  # teal green (front deck & stop wall <= 220x220 bed)
    "R46B": "#0D9488",  # deep sea teal (bolt-on rear KP08/17HS4401 & rack-tail cradle <= 220x220 bed)
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
    "R57": "#6366F1",  # indigo-violet (screwed MGN/T-slot linear hold-down rails)
    "R58": "#0284C7",  # sky cobalt blue (removable center bay overhead shroud hiding all internal guts)
}

SLOPE_Z_BACK = 185.40
SLOPE_Z_FRONT = 181.60
BAY_FLOOR_Z = 180.50
NUT_TRENCH_Z = 173.40   # 2.6 mm closed R44 sewer bed (170.8..173.4) + 2.9 mm clear of R45; houses thick R50 sewer chord
BAY_X0, BAY_X1 = OB.BXR - 59.0, OB.BXR + 59.0  # 121.0 .. 239.0 mm
HOLD_Y_M2 = (-226.0, -190.0, -154.0, -117.5, -75.5)
RACK_BOLT_YS = (-234.6, -72.4)
WALL_Y0, WALL_Y1 = -245.0, -238.0
WALL_TOP_Z = 200.00
CURB_W, CURB_TOP_ADD = 2.5, 2.2

FORK_DX = 48.0          # left/right tine centers at X = 132 & 228 mm
SCREW_X = OB.BXR        # 180.0 mm centered symmetrically
SCREW_Z = 180.50        # lowered -4.0 mm so T8, copper nut & 688ZZ arch all stay strictly < 189.7 mm
GEAR_Z0, GEAR_Z1 = 183.50, 186.60   # R47/R48 racks (F683ZZ 3.0mm), pinions & tails leave room for R50 chord + R58 shroud
FORK_Z0, FORK_Z1 = 183.50, 191.60   # +1.30 mm taller orange blades -> +1.50 mm crate lift clearance above R57/R58 (190.10)!
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


def _bb_hit(ba, bb) -> bool:
    return (
        ba.min.X <= bb.max.X and ba.max.X >= bb.min.X
        and ba.min.Y <= bb.max.Y and ba.max.Y >= bb.min.Y
        and ba.min.Z <= bb.max.Z and ba.max.Z >= bb.min.Z
    )


def _overlap(a, b) -> float:
    if not _bb_hit(a.bounding_box(), b.bounding_box()):
        return 0.0
    sa_list = a.solids() if hasattr(a, "solids") else [a]
    sb_list = b.solids() if hasattr(b, "solids") else [b]
    total = 0.0
    for sa in sa_list:
        ba = sa.bounding_box()
        for sb in sb_list:
            if not _bb_hit(ba, sb.bounding_box()):
                continue
            inter = sa & sb
            if inter is not None:
                sols = inter.solids() if hasattr(inter, "solids") else [inter]
                total += sum(getattr(s, "volume", 0.0) for s in sols)
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
    # Lowered flat skid landing channels on R46 (X=108.5..121.0 & 239.0..251.5, Z=178.60 mm -> +1.00 mm clear below
    # 12.0 mm crate skids @ 179.60 mm when carried on 191.60 mm forks, while keeping sloped outer decks & M3 screws!)
    comp = comp - box(OB.BXR - 71.5, BAY_X0 + 0.1, WALL_Y1, OB.TRAY_Y1 + 0.5, 178.60, WALL_TOP_Z + 2.0)
    comp = comp - box(BAY_X1 - 0.1, OB.BXR + 71.5, WALL_Y1, OB.TRAY_Y1 + 0.5, 178.60, WALL_TOP_Z + 2.0)
    comp = comp - box(SCREW_X - 12.2, SCREW_X + 12.2, WALL_Y1, -78.0, NUT_TRENCH_Z, BAY_FLOOR_Z + 0.5)
    comp = comp - box(SCREW_X - 16.5, SCREW_X + 16.5, -78.0, OB.TRAY_Y1 + 0.5, 179.05, BAY_FLOOR_Z + 0.5)
    comp = comp - box(SCREW_X - 10.2, SCREW_X + 10.2, -78.0, OB.TRAY_Y1 + 0.5, NUT_TRENCH_Z, 179.20)
    # Rear-wall pass-through tunnels for Left/Right R51/R52 Tine Rack tails (-335..-234 mm @ GEAR_Z0..GEAR_Z1)
    comp = comp - box(OB.BXR - FORK_DX + 2.4, OB.BXR - FORK_DX + 10.4, WALL_Y0 - 0.5, WALL_Y1 + 0.5, GEAR_Z0 - 0.5, GEAR_Z1 + 0.6)
    comp = comp - box(OB.BXR + FORK_DX - 10.4, OB.BXR + FORK_DX - 2.4, WALL_Y0 - 0.5, WALL_Y1 + 0.5, GEAR_Z0 - 0.5, GEAR_Z1 + 0.6)
    tray_clean = tray_clean - box(SCREW_X - 12.6, SCREW_X + 12.6, WALL_Y1, -77.5, NUT_TRENCH_Z, OB.TRAY_TOP_Z + 0.5)
    # Recessed under-floor sewer duct in R44 (Y=-217..-89 L / -217..-86 C+R) + R50 pylon travel slots through R46 ceiling;
    # 170.80..173.40 sewer bed ties all R44 waffle ribs into 1 solid (+2.9 mm clear of R45), while R46 bridges R47/R48 overhead!
    tray_clean = tray_clean + box(SCREW_X - 38.0, SCREW_X + 38.5, -217.5, -89.2, 170.80, NUT_TRENCH_Z)
    tray_clean = tray_clean + box(SCREW_X - 15.5, SCREW_X + 38.5, -89.4, -85.5, 170.80, NUT_TRENCH_Z)
    for lx, ly in ((208.5, -174.0), (208.5, -142.0)):  # re-open R45 right-wing captive M4 hex sockets & bores cleanly
        tray_clean = tray_clean - OB._cyl_axis(2.20, 167.5, 174.2, "z", lx, ly)
        with bd.BuildPart() as hx_cut:
            with bd.Locations((lx, ly, 170.20)):
                with bd.BuildSketch():
                    bd.RegularPolygon(radius=7.30 / math.sqrt(3.0), side_count=6, rotation=0.0)
                bd.extrude(amount=3.60)
        tray_clean = tray_clean - hx_cut.part
    tray_clean = tray_clean - box(SCREW_X - 37.0, SCREW_X + 37.0, -217.0, -89.0, NUT_TRENCH_Z, OB.TRAY_TOP_Z + 0.2)
    tray_clean = tray_clean - box(SCREW_X - 15.5, SCREW_X + 37.0, -89.5, -86.0, NUT_TRENCH_Z, OB.TRAY_TOP_Z + 0.2)
    for py_x0, py_x1, py_y1 in (
        (SCREW_X - 37.0, SCREW_X - 26.0, -89.0),
        (SCREW_X + 26.0, SCREW_X + 37.0, -86.0),
    ):
        comp = comp - box(py_x0, py_x1, -217.0, py_y1, NUT_TRENCH_Z, BAY_FLOOR_Z + 0.5)
        tray_clean = tray_clean - box(py_x0, py_x1, -217.0, py_y1, NUT_TRENCH_Z, OB.TRAY_TOP_Z + 0.5)
    # Sewer plunger entry notches at Home (-218.2..-216.8) & Max (-89.2..-88.2) for KW4-3Z-3 plungers (R46 ceiling intact overhead!)
    for pl_n0, pl_n1 in ((-218.2, -216.8), (-89.2, -88.2)):
        tray_clean = tray_clean - box(SCREW_X - 22.2, SCREW_X - 17.4, pl_n0, pl_n1, NUT_TRENCH_Z, OB.TRAY_TOP_Z + 0.2)
    # Recessed KW4-3Z-3 pockets (20x10.6x7.0 mm + C/NO/NC lug relief) & 9.5 mm M2 pin holes
    for sw_y0, sw_y1, t_y0, t_y1, yc_sw in (
        (-229.0, -217.4, -233.6, -228.5, -223.2),
        (-89.1,  -77.7,  -78.2,  -73.6,  -83.3),
    ):
        comp = comp - box(SCREW_X - 36.8, SCREW_X - 15.6, sw_y0, sw_y1, 173.0, BAY_FLOOR_Z + 0.5)
        tray_clean = tray_clean - box(SCREW_X - 36.8, SCREW_X - 15.6, sw_y0, sw_y1, 173.0, OB.TRAY_TOP_Z + 0.5)
        tray_clean = tray_clean - box(SCREW_X - 36.5, SCREW_X - 16.0, t_y0, t_y1, 173.0, OB.TRAY_TOP_Z + 0.5)
        for pin_x in (SCREW_X - 30.75, SCREW_X - 21.25):  # exact 9.5 mm KW4-3Z-3 mounting hole pitch
            tray_clean = tray_clean - OB._cyl_axis(1.05, 169.8, 173.4, "z", pin_x, yc_sw)
    comp = comp - box(SCREW_X - 37.5, SCREW_X - 34.0, WALL_Y0 - 0.5, -75.0, 174.5, 178.1)
    tray_clean = tray_clean - box(SCREW_X - 37.5, SCREW_X - 34.0, WALL_Y0 - 0.5, -75.0, 174.5, OB.TRAY_TOP_Z + 0.5)
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

    # Unified Rear KP08 housing + wrapped 17HS4401 cradle (R46B-B01: split at Y = WALL_Y0 = -245.0 mm so BOTH R46A & R46B
    # print flat inside a 220 x 220 mm bed, triple-clamped across Y = -245.0 by R53 bottom tie, 2x shear tenons + 2x M3
    # horizontal stop-wall bolts, and R56 top cover bridge!)
    CRADLE_Z0 = 155.70
    TIE_SPLIT_Z = 160.20
    TIE_Y0 = -288.00
    REAR_CAP_Y0 = BULK_Y0 - 43.4  # -338.9 mm (3.0 mm rear cap behind 17HS4401)
    SEAM_Y = WALL_Y0 - 0.05       # -245.05 mm (0.05 mm assembly clearance plane against R46A stop wall at -245.00)
    # 1) Continuous full-width cradle bed: 155.70 under motor, stepping to TIE_SPLIT_Z under Y=-288..-245.05 for flush R53-B01
    rear_cradle = box(SCREW_X - 33.0, SCREW_X + 33.0, REAR_CAP_Y0, TIE_Y0, CRADLE_Z0, SCREW_Z - 21.45)
    rear_cradle = rear_cradle + box(SCREW_X - 33.0, SCREW_X + 33.0, BULK_Y0, TIE_Y0 + 0.1, CRADLE_Z0, 161.20)
    rear_cradle = rear_cradle + box(SCREW_X - 33.0, SCREW_X + 33.0, TIE_Y0, SEAM_Y, TIE_SPLIT_Z, 161.20)
    # 2) KP08 flat pedestal raised flush under KP08 (-259.0..SEAM_Y) + unified Z=200.00 outer cheeks (-291.5..SEAM_Y)
    rear_cradle = rear_cradle + box(SCREW_X - 33.0, SCREW_X + 33.0, KP08_Y0 - 0.5, SEAM_Y, TIE_SPLIT_Z, 165.40)
    for rx0, rx1 in ((SCREW_X - 33.0, SCREW_X - 28.0), (SCREW_X + 28.0, SCREW_X + 33.0)):
        rear_cradle = rear_cradle + box(rx0, rx1, BULK_Y1, TIE_Y0 + 0.1, CRADLE_Z0, WALL_TOP_Z)
        rear_cradle = rear_cradle + box(rx0, rx1, TIE_Y0, SEAM_Y, TIE_SPLIT_Z, WALL_TOP_Z)

    kp_housing = box(SCREW_X - 33.0, SCREW_X + 33.0, KP08_Y0 - 0.5, SEAM_Y, 165.40, WALL_TOP_Z)
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
    rear_cradle = rear_cradle + (kp_housing - kp_foot_cut - kp_arch_cut - kp_collar_cut - kp_bolt_access)

    # 3) Bulkhead & snug 17HS4401 wrap pocket (octagonal lower-corner fillets eliminate any corner silver sliver)
    bulkhead = box(SCREW_X - 33.0, SCREW_X + 33.0, BULK_Y0, BULK_Y1, CRADLE_Z0, 201.80) - OB._cyl_axis(
        11.5, BULK_Y0 - 0.5, BULK_Y1 + 0.5, "y", SCREW_X, SCREW_Z
    )
    rear_cradle = rear_cradle + bulkhead
    for sx0, sx1 in ((SCREW_X - 33.0, SCREW_X - 21.45), (SCREW_X + 21.45, SCREW_X + 33.0)):
        rear_cradle = rear_cradle + box(sx0, sx1, REAR_CAP_Y0, BULK_Y0 + 0.2, CRADLE_Z0, 201.80)
    rear_cradle = rear_cradle + box(SCREW_X - 33.0, SCREW_X + 33.0, REAR_CAP_Y0, BULK_Y0 + 0.2, CRADLE_Z0, SCREW_Z - 21.45)
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
        rear_cradle = rear_cradle + cb.part.moved(bd.Location((0.0, BULK_Y0, 0.0)))
    # Closed rear protective end cap behind NEMA 17 (top center notch leaves 4-wire JST cable exit clear)
    rear_cap = box(SCREW_X - 33.0, SCREW_X + 33.0, REAR_CAP_Y0, BULK_Y0 - 40.35, CRADLE_Z0, 201.80) - box(
        SCREW_X - 7.5, SCREW_X + 7.5, REAR_CAP_Y0 - 0.2, BULK_Y0 - 40.1, 195.0, 202.2
    )
    rear_cradle = rear_cradle + rear_cap
    # Open-top U-channel rear protective rack-tail bays (bottom, outer, inner & rear bumper walls up to WALL_TOP_Z=200.0;
    # top stays open for drop-in assembly & top visibility, sealed by removable screwed enclosure R56-B01)
    # Tails span X=134.8..142.5 (L) and X=217.5..225.2 (R), Z=GEAR_Z0..GEAR_Z1 (183.5..187.2)
    REAR_OUT_PAD = 4.4  # outer wall X=127.6..134.4 (L) & 225.6..232.4 (R) = 6.8 mm wide for M3 lid screws
    for sh_x0, sh_x1, x_clear0, x_clear1 in (
        (OB.BXR - FORK_DX - REAR_OUT_PAD, SCREW_X - 32.8, OB.BXR - FORK_DX + 2.4, OB.BXR - FORK_DX + 10.4),
        (SCREW_X + 32.8, OB.BXR + FORK_DX + REAR_OUT_PAD, OB.BXR + FORK_DX - 10.4, OB.BXR + FORK_DX - 2.4),
    ):
        tail_box = box(sh_x0, sh_x1, REAR_CAP_Y0, SEAM_Y, 179.00, WALL_TOP_Z) - box(
            x_clear0, x_clear1, REAR_CAP_Y0 + 3.0, SEAM_Y + 0.5, GEAR_Z0 - 0.5, WALL_TOP_Z + 0.5
        )
        rear_cradle = rear_cradle + tail_box
    # Reinforced upper front boss pillars on R46B (X=145.0..153.5 & 206.5..215.0, Z=188.0..200.0) + 2x interlocking shear
    # alignment tenons (X=147.6..151.6 & 208.4..212.4, Z=181.5..187.5, Y=-245.05..-242.05) plugging into R46A stop wall!
    for tx0, tx1, bx0, bx1 in ((147.6, 151.6, 145.0, 153.5), (208.4, 212.4, 206.5, 215.0)):
        rear_cradle = rear_cradle + box(bx0, bx1, KP08_Y0 - 0.5, SEAM_Y, 188.00, WALL_TOP_Z)
        rear_cradle = rear_cradle + box(tx0, tx1, SEAM_Y - 0.5, WALL_Y0 + 2.95, 181.50, 187.50)
        comp = comp - box(tx0 - 0.15, tx1 + 0.15, WALL_Y0 - 0.2, WALL_Y0 + 3.15, 181.35, 187.65)
    comp = comp - OB._cyl_axis(4.6, WALL_Y0 - 1.0, WALL_Y1 + 1.0, "y", SCREW_X, SCREW_Z)
    comp = comp - OB._cyl_axis(6.8, WALL_Y0 - 0.5, WALL_Y0 + 1.0, "y", SCREW_X, SCREW_Z)
    rear_cradle = rear_cradle - OB._cyl_axis(6.8, KP08_Y0 - 2.0, SEAM_Y + 0.5, "y", SCREW_X, SCREW_Z)

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

    # Drill M2 pilot bores AFTER drop skirts so front skirt (-72.5..-66.75) cannot close -72.4 end bores
    for sgn_m in (-1, 1):
        mx_out = OB.BXR + sgn_m * (FORK_DX + 8.3)  # 123.7 / 236.3
        for my_h in HOLD_Y_M2:
            comp = comp - OB._cyl_axis(0.95, 174.20, BAY_FLOOR_Z + 0.5, "z", mx_out, my_h)
            tray_clean = tray_clean - OB._cyl_axis(0.95, 174.20, OB.TRAY_TOP_Z + 0.5, "z", mx_out, my_h)
        mx_end = SCREW_X + sgn_m * 19.5  # 160.5 / 199.5 (R47/R48 + R58 shared M2 x 14 anchors)
        for my_r in RACK_BOLT_YS:
            comp = comp - OB._cyl_axis(0.95, 174.50, BAY_FLOOR_Z + 0.5, "z", mx_end, my_r)
            tray_clean = tray_clean - OB._cyl_axis(0.95, 174.50, OB.TRAY_TOP_Z + 0.5, "z", mx_end, my_r)
        mx_flg = SCREW_X + sgn_m * 14.5  # 165.5 / 194.5 (4x M2 x 4 mm hold-downs into R46 bridge)
        for my_f in (-202.0, -166.0, -130.0, -94.0):
            comp = comp - OB._cyl_axis(0.95, 178.40, BAY_FLOOR_Z + 0.5, "z", mx_flg, my_f)

    # 5) Removable Under-Tray Rear Cantilever Tie Plate (R53-B01): docks cleanly beneath R46 (Y=-287.85..-245, Z=155.7..160.15)
    #    & extends forward under R44 rear rim (Y=-245..-218, Z=155.7..167.85); unbolts downward so R46/R44 separate freely!
    under_tie = box(SCREW_X - 33.0, SCREW_X + 33.0, TIE_Y0 + 0.15, WALL_Y0, CRADLE_Z0, TIE_SPLIT_Z - 0.05)
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
        under_tie = under_tie - OB._cyl_axis(3.10, CRADLE_Z0 - 0.5, 161.55, "z", rx, -242.0)
        tray_clean = tray_clean - OB._cyl_axis(1.45, OB.TRAY_RIB_Z0 - 0.5, OB.TRAY_TOP_Z - 1.5, "z", rx, -242.0)
        hw_solids.append(_socket_bolt(2.70, 2.4, 1.40, 15.0, "z", rx, -242.0, 161.45, head_dir=-1))

    for ux in (162.0, 198.0):
        under_tie = under_tie - OB._cyl_axis(1.65, CRADLE_Z0 - 0.5, TIE_SPLIT_Z + 0.5, "z", ux, -275.0)
        under_tie = under_tie - OB._cyl_axis(3.10, CRADLE_Z0 - 0.5, 158.30, "z", ux, -275.0)
        rear_cradle = rear_cradle - OB._cyl_axis(1.45, TIE_SPLIT_Z - 0.5, 161.4, "z", ux, -275.0)
        hw_solids.append(_socket_bolt(2.70, 2.0, 1.40, 3.1, "z", ux, -275.0, 158.25, head_dir=-1))

    kp_yc = (KP08_Y0 + KP08_Y1) / 2.0
    for bx in (SCREW_X - 21.0, SCREW_X + 21.0):
        rear_cradle = rear_cradle - OB._cyl_axis(2.65, TIE_SPLIT_Z - 0.5, 166.0, "z", bx, kp_yc)
        under_tie = under_tie - OB._cyl_axis(2.65, CRADLE_Z0 - 0.5, TIE_SPLIT_Z + 0.5, "z", bx, kp_yc)
        under_tie = under_tie - OB._cyl_axis(5.10, CRADLE_Z0 - 0.5, 159.8, "z", bx, kp_yc)
        hw_solids.append(_socket_bolt(4.25, 4.5, 2.40, 15.2, "z", bx, kp_yc, 170.60, head_dir=1))
        hw_solids.append(_hex_nut(8.0, 3.6, "z", bx, kp_yc, 155.90, 2.45))

    for sx in (-15.5, 15.5):
        for sz in (-15.5, 15.5):
            nx, nz = SCREW_X + sx, SCREW_Z + sz
            rear_cradle = rear_cradle - OB._cyl_axis(1.65, BULK_Y0 - 0.5, BULK_Y1 + 0.5, "y", nx, nz)
            hw_solids.append(_socket_bolt(2.75, 2.8, 1.45, 7.5, "y", nx, BULK_Y1 + 0.1, nz, head_dir=1))

    # 2x Horizontal counterbored M3 x 14 mm stop-wall seam bolts clamping R46A (-238..-245) into R46B (-245.05..-255)
    for wx in (149.25, 210.75):
        wz = 194.0
        comp = comp - OB._cyl_axis(3.05, WALL_Y1 - 2.60, WALL_Y1 + 0.4, "y", wx, wz)
        comp = comp - OB._cyl_axis(1.65, WALL_Y0 - 0.3, WALL_Y1 - 2.40, "y", wx, wz)
        rear_cradle = rear_cradle - OB._cyl_axis(1.45, -255.5, SEAM_Y + 0.2, "y", wx, wz)
        hw_solids.append(_socket_bolt(2.70, 2.40, 1.40, 14.0, "y", wx, WALL_Y1 - 2.55, wz, head_dir=1))

    for fx in (SCREW_X - 12.6, SCREW_X + 12.6):
        comp = comp - OB._cyl_axis(1.65, 178.0, 179.5, "z", fx, -74.0)
        tray_clean = tray_clean - OB._cyl_axis(1.65, OB.TRAY_RIB_Z0 - 0.5, OB.TRAY_TOP_Z + 0.5, "z", fx, -74.0)
        hw_solids.append(_socket_bolt(2.70, 2.8, 1.45, 14.5, "z", fx, -74.0, 182.25, head_dir=1))
        hw_solids.append(_hex_nut(5.5, 2.4, "z", fx, -74.0, 163.40, 1.50))

    # 6) Removable Screwed Top Motor + Fork-Blade Tail Enclosure Lid (R56-B01):
    #    Extended 7.0 mm forward over the R46A stop wall (Y=-338.9..-238.0, 104.8 x 100.9 mm) so R56-B01 bolts into BOTH
    #    R46B-B01 (6x M3x10) AND R46A-B01's stop wall (2x M3x10 at Y=-241.5), forming a rigid top structural flange bridge!
    LID_X0, LID_X1 = OB.BXR - FORK_DX - REAR_OUT_PAD, OB.BXR + FORK_DX + REAR_OUT_PAD  # 127.6..232.4
    LID_Z0, LID_Z1 = WALL_TOP_Z + 0.05, WALL_TOP_Z + 5.20  # 200.05..205.20 (0.05 mm clear of R46A/R46B Z=200.00 rim)
    top_cover = (
        box(LID_X0, LID_X1, REAR_CAP_Y0, WALL_Y1, LID_Z0, LID_Z1)
        - box(SCREW_X - 33.2, SCREW_X + 33.2, REAR_CAP_Y0 - 0.2, BULK_Y1 + 0.3, LID_Z0 - 0.2, 202.05)  # clears Z=201.80 NEMA walls
    ) + box(SCREW_X - 9.5, SCREW_X + 9.5, REAR_CAP_Y0, BULK_Y0 - 26.5, LID_Z1 - 0.4, 209.00)  # raised JST connector hood
    # Internal JST socket & rear 4-wire exit tunnel underneath hood (201.80..206.80 -> clears 206.15 JST plug)
    top_cover = top_cover - box(SCREW_X - 6.8, SCREW_X + 6.8, REAR_CAP_Y0 - 0.2, BULK_Y0 - 29.0, 201.50, 206.80)

    cover_hw = []
    lid_m3_rear = (
        (LID_X0 + 3.4, -252.0), (LID_X1 - 3.4, -252.0),   # front outer cheeks of R46B (X=131.0, 229.0)
        (LID_X0 + 3.4, -331.5), (LID_X1 - 3.4, -331.5),   # rear outer cheeks of R46B (X=131.0, 229.0)
        (147.5, -286.0), (212.5, -286.0),                  # center pylons of R46B
    )
    for lx, ly in lid_m3_rear:
        top_cover = top_cover - OB._cyl_axis(1.70, LID_Z0 - 0.5, LID_Z1 + 0.5, "z", lx, ly)
        top_cover = top_cover - OB._cyl_axis(3.05, 202.40, LID_Z1 + 0.5, "z", lx, ly)
        rear_cradle = rear_cradle - OB._cyl_axis(1.45, 191.00, WALL_TOP_Z + 0.5, "z", lx, ly)
        cover_hw.append(_socket_bolt(2.70, 2.40, 1.40, 10.0, "z", lx, ly, 202.45, head_dir=1))
    # 2x forward M3 x 10 mm bridge bolts through R56-B01 into top of R46A-B01 stop wall (Y=-241.5, X=162.0 & 198.0)
    for lx, ly in ((162.0, -241.5), (198.0, -241.5)):
        top_cover = top_cover - OB._cyl_axis(1.70, LID_Z0 - 0.5, LID_Z1 + 0.5, "z", lx, ly)
        top_cover = top_cover - OB._cyl_axis(3.05, 202.40, LID_Z1 + 0.5, "z", lx, ly)
        comp = comp - OB._cyl_axis(1.45, 191.00, WALL_TOP_Z + 0.5, "z", lx, ly)
        cover_hw.append(_socket_bolt(2.70, 2.40, 1.40, 10.0, "z", lx, ly, 202.45, head_dir=1))

    # 7) Removable Screwed Center-Bay Overhead Shroud (R58-B01) with Integral Cog-Boundary C-Channel Ribs:
    #    - Longitudinal 3.20 mm boundary ribs along X=139.4..144.4 & 215.6..220.6 (Z=186.90..190.10) stiffen the shroud
    #      and guard both cogwheel / tine-rack tracks (+0.60 mm clear of R51/R52, +0.80 mm clear of R50 chord).
    #    - Integral end mounting feet at (160.5 / 199.5, -234.6 & -72.4) share the EXACT 4 end screw holes with R47/R48
    #      (4x M2 x 14 mm clamping R58 -> R47/R48 -> R46 -> R44 with zero extra R46 pillars needed!).
    bay_cover = (
        box(139.4, 220.6, -238.0, -70.0, 188.65, 190.10)
        + box(139.4, 144.4, -238.0, -70.0, 186.90, 190.10)
        + box(215.6, 220.6, -238.0, -70.0, 186.90, 190.10)
        - box(163.6, 196.4, -78.2, -69.5, 186.50, 190.30)  # front cutout flush around R49 189.60 mm arch crown
    )
    for sx0, sx1 in ((157.8, 163.2), (196.8, 202.2)):
        bay_cover = bay_cover + box(sx0, sx1, -237.2, -231.8, 186.65, 188.70)  # rear feet onto R47/R48 (+0.05 mm clear)
        bay_cover = bay_cover + box(sx0, sx1, -75.2, -70.0, 186.65, 188.70)   # front feet onto R47/R48 (+0.05 mm clear)
    bay_cover_hw = []
    for sgn_c in (-1, 1):
        bx = SCREW_X + sgn_c * 19.5  # 160.5 / 199.5 (shared end anchor bores with R47/R48 -> R46 -> R44!)
        for by in RACK_BOLT_YS:
            bay_cover = bay_cover - OB._cyl_axis(1.10, 186.50, 190.30, "z", bx, by)
            bay_cover = bay_cover - OB._cyl_axis(1.90, 188.90, 190.30, "z", bx, by)
            bay_cover_hw.append(_socket_bolt(1.75, 1.15, 0.95, 14.0, "z", bx, by, 188.95, head_dir=1))

    # Engrave R46A-B01 on front face of stop wall, R46B-B01 on rear face of motor cap, R53-B01 on under-tie, R56-B01 & R58-B01 on lids
    comp_part = _engrave_verified(
        comp, "forklift_deck", (OB.BXR - 54.0, WALL_Y1, 193.0), (1, 0, 0), (0, 1, 0), 4.0
    )
    rear_cradle_part = _engrave_verified(
        rear_cradle, "fork_rear_cradle", (SCREW_X - 18.0, REAR_CAP_Y0, 176.0), (1, 0, 0), (0, -1, 0), 4.0
    )
    under_tie_part = _engrave_verified(
        under_tie, "under_tie", (SCREW_X - 18.0, -262.0, CRADLE_Z0), (1, 0, 0), (0, 0, -1), 4.2
    )
    top_cover_part = _engrave_verified(
        top_cover, "fork_top_cover", (SCREW_X - 18.0, -270.0, LID_Z1), (1, 0, 0), (0, 0, 1), 4.5
    )
    bay_cover_part = _engrave_verified(
        bay_cover, "fork_bay_cover", (SCREW_X - 10.0, -155.0, 190.10), (1, 0, 0), (0, 0, 1), 4.0
    )
    return tray_clean, comp_part, rear_cradle_part, under_tie_part, hw_solids, top_cover_part, cover_hw, bay_cover_part, bay_cover_hw


def build_forklift_mechanism():
    """Build printed R47..R55 parts + Horizontal 1:2 Rack-and-Pinion (2x F683ZZ + CNC4759-A/MT0382)."""
    box = OB._box
    f04 = []
    moving_hw = []

    # R47 & R48: Left & Right Full-Length Solid Base Guide Tracks + Inner Bridge Flange + M3 End & Mid-Span Anchors
    rails = {}
    rack_hw = []
    for key, sgn in (("rail_left", -1), ("rail_right", 1)):
        xc = OB.BXR + sgn * FORK_DX
        outer_x0 = (xc - 11.0) if sgn < 0 else (xc + 5.6)  # 121.0..126.4 (L) / 233.6..239.0 (R) -> 5.4 mm solid for M3
        outer_x1 = (xc - 5.6) if sgn < 0 else (xc + 11.0)
        # Base Rack sits continuously at 180.55..187.20 on solid R46 bridge (zero floating middle span!)
        x_pitch_base = OB.BXR + sgn * (FORK_DX - PIN_DX_CENTER - PIN_RP)  # 156.5 (L) / 203.5 (R)
        spine_x0 = (x_pitch_base + 1.25 * RP_MOD) if sgn < 0 else (x_pitch_base - 1.25 * RP_MOD - 5.5)  # 157.75 / 196.75
        spine_x1 = (spine_x0 + 5.5) if sgn < 0 else (x_pitch_base - 1.25 * RP_MOD)                      # 163.25 / 202.25
        flg_x0 = (spine_x1 - 0.5) if sgn < 0 else (SCREW_X + 12.2)  # 162.75..167.8 (L) / 192.2..197.25 (R)
        flg_x1 = (SCREW_X - 12.2) if sgn < 0 else (spine_x0 + 0.5)
        floor_x0 = min(xc - 8.0, spine_x0)
        floor_x1 = max(xc + 8.0, spine_x1)
        base_rack = _build_horizontal_rack(
            x_pitch_base, spine_x1 if sgn < 0 else spine_x0, -1 if sgn < 0 else 1,
            -237.2, -70.0, GEAR_Z0, GEAR_Z1, y_phase=PIN_Y_HOME,
        )
        full_spine = box(spine_x0, spine_x1, -237.2, -70.0, 180.55, GEAR_Z1)
        inner_flange = box(flg_x0, flg_x1, -217.0, -86.5, 180.55, 182.65)
        riser_rear = box(floor_x0, floor_x1, -237.2, -217.0, 180.55, 181.10)
        riser_front = box(floor_x0, floor_x1, -82.2, -70.0, 180.55, 181.10)
        rail = (
            box(outer_x0, outer_x1, -237.2, -70.6, 180.55, 185.00)
            + box(xc - 8.0, xc + 8.0, -237.2, -70.6, 180.55, 181.10)
            + base_rack + full_spine + inner_flange + riser_rear + riser_front
        )
        mx_hole = 0.5 * (outer_x0 + outer_x1)  # 123.7 / 236.3
        for my_h in HOLD_Y_M2:
            rail = rail - OB._cyl_axis(1.10, 180.40, 185.40, "z", mx_hole, my_h)
        mx_end = SCREW_X + sgn * 19.5  # 160.5 / 199.5 (shared M2 x 14 mm bores clamping R58 -> R47/R48 -> R46 -> R44)
        for my_r in RACK_BOLT_YS:
            rail = rail - OB._cyl_axis(1.10, 180.40, 186.80, "z", mx_end, my_r)
        mx_flg = SCREW_X + sgn * 14.5  # 165.5 / 194.5 (4x M2 x 4 mm hold-downs into R46 bridge, >=1.15 mm edge rim)
        for my_f in (-202.0, -166.0, -130.0, -94.0):
            rail = rail - OB._cyl_axis(1.10, 180.40, 182.80, "z", mx_flg, my_f)
            rack_hw.append(_socket_bolt(1.75, 1.15, 0.95, 4.0, "z", mx_flg, my_f, 182.65, head_dir=1))
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

    # R50: Stage-1 MGN Linear-Rail H-Carriage (outer MGN wing captured by R57 + full-length 45-deg T-rail capturing R51/R52!)
    r50_parts = []
    pinions = {}
    gear_pts = O.involute_gear_pts(RP_Z, RP_MOD, RP_PA, backlash=0.14)
    for pkey, sgn in (("pinion_left", -1), ("pinion_right", 1)):
        xc = OB.BXR + sgn * FORK_DX
        x_pin = OB.BXR + sgn * (FORK_DX - PIN_DX_CENTER)  # 148.5 (L) / 211.5 (R)
        # Main base runner (126.7..138.6 L / 221.4..233.3 R) with 55-deg steepened outer MGN shoulder captured by R57
        x_out = (xc - 5.3) if sgn < 0 else (xc + 5.3)
        x_in = (xc + 6.6) if sgn < 0 else (xc - 6.6)
        r50_base_xz = [
            (x_out, 181.30), (x_out, 183.10), (x_out - sgn * 0.65, 184.00),
            (x_out - sgn * 2.70, 184.00), (x_out - sgn * 2.70, 183.30),
            (x_in, 183.30), (x_in, 181.30),
        ]
        r50_parts.append(OB._extrude_xz_poly(r50_base_xz, -234.0, -71.5))
        # Centered 55-deg Steepened MGN / Dovetail T-Rail (Y=-234.0..-78.6): 30% lower pry force, support-free FDM print
        x_mgn = OB.BXR + sgn * (FORK_DX - 1.9)  # 133.9 (L) / 226.1 (R)
        mgn_rail_xz = [
            (x_mgn - 1.20, 183.20), (x_mgn - 1.20, 183.85),
            (x_mgn - 2.50, 185.70), (x_mgn - 2.50, 186.60), (x_mgn - 1.95, 187.15),
            (x_mgn + 1.95, 187.15), (x_mgn + 2.50, 186.60), (x_mgn + 2.50, 185.70),
            (x_mgn + 1.20, 183.85), (x_mgn + 1.20, 183.20),
        ]
        r50_parts.append(OB._extrude_xz_poly(mgn_rail_xz, -234.0, -78.6))

        # Boss pad rising from pylon arm forward to Y_PIN=-202 (strictly inside X pylon slot 143..154 / 206..217)
        boss = (
            OB._cyl_axis(5.2, 181.30, 183.35, "z", x_pin, PIN_Y_HOME)
            + box(x_pin - 4.8, x_pin + 4.8, -215.5, PIN_Y_HOME, 181.30, 183.10)
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
        moving_hw.append(_socket_bolt(2.50, 0.95, 1.42, 5.2, "z", x_pin, PIN_Y_HOME, GEAR_Z1 - 0.05, head_dir=1))

    # Closed Double-Chord Box-Truss Cross-Bridge on R50-B01 (solves mid-bridge torsion/bending completely!):
    # 1) Thick 3.35 mm lower sewer chord (Z=174.35..177.70 -> +0.75 mm clear above R45 M4 nuts/screws!)
    # 2) Wide 24.0 mm upper lateral gusset wings (Y=-220..-196, Z=181.30..183.30) tying pinion bosses & runners into pylons
    # 3) Full-height vertical shear posts (Z=174.35..188.35) + 8.2 mm deep rear-offset posts behind R54/R55 pinions (-220..-211.8)
    # 4) Overhead top box chord (Z=186.90..188.35, Y=-218.5..-207.7) clears BOTH racks (186.60) & R58 shroud (188.65)!
    r50_parts.append(box(144.2, 165.0, -215.5, -210.6, 174.35, 177.70))
    r50_parts.append(box(164.6, SCREW_X - 10.8, -215.5, -207.7, 174.35, 177.70))
    r50_parts.append(box(SCREW_X + 10.8, 215.8, -215.5, -207.7, 174.35, 177.70))
    r50_parts.append(box(144.2, 152.8, -215.5, -210.6, 174.35, 181.50))
    r50_parts.append(box(207.2, 215.8, -215.5, -207.7, 174.35, 181.50))
    r50_parts.append(box(OB.BXR - FORK_DX + 5.8, 153.3, -220.0, -196.0, 181.30, 183.30))
    r50_parts.append(box(206.7, OB.BXR + FORK_DX - 5.8, -220.0, -196.0, 181.30, 183.30))
    r50_parts.append(box(145.2, 152.8, -220.0, -211.8, 181.30, 188.35))
    r50_parts.append(box(207.2, 214.8, -220.0, -211.8, 181.30, 188.35))
    r50_parts.append(box(145.2, 214.8, -218.5, -207.7, 186.90, 188.35))
    nut_plate = box(SCREW_X - 11.2, SCREW_X + 11.2, -215.5, -207.7, 174.35, 188.35) - OB._cyl_axis(
        5.30, -216.0, -207.2, "y", SCREW_X, SCREW_Z
    )
    for hx in (SCREW_X - 8.0, SCREW_X + 8.0):
        nut_plate = nut_plate - OB._cyl_axis(1.65, -216.0, -207.2, "y", hx, SCREW_Z)
    r50_parts.append(nut_plate)

    r50_raw = r50_parts[0]
    for p in r50_parts[1:]:
        r50_raw = r50_raw + p
    for sgn in (-1, 1):  # re-drill F683ZZ M3 pilot bores after unioning lateral gusset wings
        r50_raw = r50_raw - OB._cyl_axis(1.48, 181.00, 183.80, "z", OB.BXR + sgn * (FORK_DX - PIN_DX_CENTER), PIN_Y_HOME)
    r50_carriage = _engrave_verified(
        r50_raw, "stage1_carriage", (SCREW_X - 7.5, -213.0, 188.35), (1, 0, 0), (0, 0, 1), 2.4
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

    # R51 & R52: MGN Wrap-Around Linear Carriage Fork Tines (locks onto R50 45-deg T-rail + tail hook across 0..+240!)
    tines = {}
    for key, sgn in (("fork_tine_left", -1), ("fork_tine_right", 1)):
        xc = OB.BXR + sgn * FORK_DX
        x_mgn = OB.BXR + sgn * (FORK_DX - 1.9)  # 133.9 (L) / 226.1 (R)
        bl0 = (xc - 2.4) if sgn < 0 else (xc - 6.8)  # 129.6..138.8 (L) / 221.2..230.4 (R)
        bl1 = (xc + 6.8) if sgn < 0 else (xc + 2.4)
        # Inverted-U 55-deg MGN carriage groove with >=0.37 mm perpendicular clearance over R50 dovetail T-rail
        mgn_cut_xz = [
            (x_mgn - 1.58, FORK_Z0 - 0.2), (x_mgn - 1.58, 183.75),
            (x_mgn - 2.90, 185.62), (x_mgn - 2.90, 186.80), (x_mgn - 2.15, 187.55),
            (x_mgn + 2.15, 187.55), (x_mgn + 2.90, 186.80), (x_mgn + 2.90, 185.62),
            (x_mgn + 1.58, 183.75), (x_mgn + 1.58, FORK_Z0 - 0.2),
        ]
        blade = box(bl0, bl1, -234.0, -78.0, 183.60, FORK_Z1) - OB._extrude_xz_poly(mgn_cut_xz, -234.5, -77.5)
        x_pitch_tine = OB.BXR + sgn * (FORK_DX - PIN_DX_TINE)  # 140.5 (L) / 219.5 (R)
        # Rack back spine starts at 136.9 (0.5 mm clear of R50 T-head) + continuous inner 55-deg MGN claw down tail (-335..-234)
        x_rack_back = xc - sgn * 4.9  # 136.9 (L) / 223.1 (R)
        tine_rack = _build_horizontal_rack(
            x_pitch_tine, x_rack_back, 1 if sgn < 0 else -1,
            -335.0, -78.0, GEAR_Z0, GEAR_Z1, y_phase=PIN_Y_HOME,
        )
        # Continuous inner 55-deg MGN lower dovetail hook along rear rack tail (-335..-233.5 -> 136.4 mm lock at +240 mm!)
        tail_hook_xz = [
            (x_mgn - sgn * 1.58, 183.60),
            (x_mgn - sgn * 2.90, 185.45),
            (x_mgn - sgn * 3.40, 185.45),
            (x_mgn - sgn * 3.40, 183.60),
        ]
        tine_rack = tine_rack + OB._extrude_xz_poly(tail_hook_xz, -335.0, -233.5)
        with bd.BuildPart() as tn:
            with bd.BuildSketch(bd.Plane.YZ.offset(bl0)):
                with bd.BuildLine():
                    bd.Polyline(
                        (-78.2, 183.60), (-63.0, 183.60),
                        (-63.0, 183.60 + 1.5), (-78.2, FORK_Z1), (-78.2, 183.60)
                    )
                bd.make_face()
            bd.extrude(amount=bl1 - bl0)
        tine_raw = blade + tine_rack + tn.part
        tines[key] = _engrave_verified(tine_raw, key, (xc - sgn * 2.2, -185.0, FORK_Z1), (0, 1, 0), (0, 0, 1), 2.5)

    # R57-B01: 100% Printable Screwed 55-deg MGN Hold-Down Rails (capturing R50 outer 55-deg wing, Z=184.35..190.10)
    hold_parts, hold_hw = {}, []
    for key, sgn in (("hold_rail_left", -1), ("hold_rail_right", 1)):
        xc = OB.BXR + sgn * FORK_DX
        ox0, ox1 = ((xc - 11.0, xc - 5.6) if sgn < 0 else (xc + 5.6, xc + 11.0))  # 121.0..126.4 / 233.6..239.0 (5.4 mm solid!)
        lx0, lx1 = ((xc - 5.5, xc - 3.0) if sgn < 0 else (xc + 3.0, xc + 5.5))    # 126.5..129.0 / 231.0..233.5 (0.6 mm clear of R51!)
        rail_a = (
            box(ox0, ox1, -237.2, -70.6, 185.05, 190.10)
            + box(min(ox0, lx0), max(ox1, lx1), -237.2, -70.6, 186.60, 190.10)
            + box(lx0, lx1, -237.2, -70.6, 184.35, 190.10)  # 0.35 mm clear above R50 184.00 outer MGN wing
        )
        mx_h = 0.5 * (ox0 + ox1)  # 123.7 / 236.3 (5x M2 x 14 mm -> 0.80 mm solid side walls!)
        for my_h in HOLD_Y_M2:
            rail_a = rail_a - OB._cyl_axis(1.10, 184.8, 190.4, "z", mx_h, my_h)
            rail_a = rail_a - OB._cyl_axis(1.90, 188.3, 190.4, "z", mx_h, my_h)
            hold_hw.append(_socket_bolt(1.75, 1.15, 0.95, 14.0, "z", mx_h, my_h, 188.35, head_dir=1))
        face_x = ox0 if sgn < 0 else ox1
        x_dir = (0, 1, 0) if sgn < 0 else (0, -1, 0)
        norm = (-1, 0, 0) if sgn < 0 else (1, 0, 0)
        hold_parts[key] = _engrave_verified(rail_a, key, (face_x, -154.0, 187.6), x_dir, norm, 2.2)

    # 2x Accurate Donghai KW4-3Z-3 Sub-Miniature Limit Switches (Duino.lk SW0160: 20.0x10.6x6.8 mm body + 9.5 mm M2 bores
    # + off-center black push-button plunger at X=158.5..161.5 + 3x pierced SPDT C/NO/NC spade terminals, 20x16x7 mm overall)
    sw_solids = []
    SW_X0, SW_X1 = SCREW_X - 36.0, SCREW_X - 16.0  # 144.0 .. 164.0 (20.0 mm long)
    for sw_y0, sw_y1, pl_y0, pl_y1, term_y0, term_y1 in (
        (-228.5, -217.9, -217.9, -215.8, -232.9, -228.5),  # Home KW4-3Z-3: plunger +Y (-215.8), 3 lugs -Y
        (-88.6,  -78.0,  -90.5,  -88.6,  -78.0,  -73.6),   # Max  KW4-3Z-3: plunger -Y (-90.5), 3 lugs +Y (clears R49!)
    ):
        body = box(SW_X0, SW_X1, sw_y0, sw_y1, 173.20, 180.00)
        # Cut realistic Donghai stepped end shoulders near terminal face + 2x Ø2.25 mm mounting holes on 9.5 mm pitch
        yc_sw = 0.5 * (sw_y0 + sw_y1)
        for pin_x in (SW_X0 + 5.25, SW_X0 + 14.75):  # 149.25 & 158.75 (exact 9.50 mm pitch)
            body = body - OB._cyl_axis(1.12, 172.8, 180.4, "z", pin_x, yc_sw - 2.2 if sw_y0 < -200 else yc_sw + 2.2)
        sw_solids.append(body)
        # Off-center 3.0 x 2.0 mm push-button plunger (~5.5 mm from C-end, aligned with R50 bumper X=158.0..162.4)
        sw_solids.append(box(SW_X1 - 5.5, SW_X1 - 2.5, pl_y0, pl_y1, 175.40, 177.80))
        # 3x flat perforated SPDT solder lugs (NC @ 146.5, NO @ 154.0, C @ 162.0 -> 20x16x7 mm overall envelope!)
        for lx in (SW_X0 + 2.5, SW_X0 + 10.0, SW_X1 - 2.0):
            lug = box(lx - 1.5, lx + 1.5, term_y0, term_y1, 176.25, 176.95) - OB._cyl_axis(
                0.70, 176.0, 177.2, "z", lx, 0.5 * (term_y0 + term_y1)
            )
            sw_solids.append(lug)
    limit_switches = bd.Compound(children=sw_solids, label="DuinoLK_KW4_3Z_3_Limit_Switches_2x")

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
        "fork_hold_rails": bd.Compound(children=[hold_parts["hold_rail_left"], hold_parts["hold_rail_right"]], label="R57_MGN_Hold_Down_Rails"),
        "fork_hold_fasteners": bd.Compound(children=hold_hw, label="R57_M3_Screws"),
        "_rack_fasteners": rack_hw,
        "_print_solids": {
            "rail_left": rails["rail_left"],
            "rail_right": rails["rail_right"],
            "front_688zz_arch": r49_arch,
            "stage1_carriage": r50_carriage,
            "fork_tine_left": tines["fork_tine_left"],
            "fork_tine_right": tines["fork_tine_right"],
            "pinion_left": pinions["pinion_left"],
            "pinion_right": pinions["pinion_right"],
            "hold_rail_left": hold_parts["hold_rail_left"],
            "hold_rail_right": hold_parts["hold_rail_right"],
        },
    }


def build_objects_base_and_forklift():
    """Assemble base stack (R41 + TAL220 + R45 + R44) + R46A/R46B deck + R47..R58 forklift + fasteners."""
    boom, live_wish, load_cell, lc_hw = OB.build_tray_boom_and_loadcell()
    picker = OB.build_tray_and_picker()
    tray_clean = picker["tray_deck"] & OB._box(
        OB.TRAY_X0 - 1.0, OB.TRAY_X1 + 1.0, OB.TRAY_Y0 - 1.0, OB.TRAY_Y1 + 1.0,
        OB.TRAY_RIB_Z0 - 1.0, OB.TRAY_TOP_Z
    )
    tray_clean, comp_deck, rear_cradle, under_tie, comp_hw, top_cover, cover_hw, bay_cover, bay_cover_hw = build_forklift_deck(tray_clean)
    fork_parts = build_forklift_mechanism()
    rack_hw = fork_parts.pop("_rack_fasteners")
    all_hw = bd.Compound(children=[*lc_hw.solids(), *comp_hw, *rack_hw], label="Base_And_Compartment_Hardware")
    cover_fasteners = bd.Compound(children=cover_hw, label="R56_Cover_M3_Screws")
    bay_cover_fasteners = bd.Compound(children=bay_cover_hw, label="R58_Bay_Cover_M2_Screws")

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
        "fork_rear_cradle": rear_cradle,
        "fork_under_tie": under_tie,
        "fork_top_cover": top_cover,
        "fork_cover_fasteners": cover_fasteners,
        "fork_bay_cover": bay_cover,
        "fork_bay_cover_fasteners": bay_cover_fasteners,
        "base_fasteners": all_hw,
        **fork_parts,
        "ref_load": ref_load,
        "ref_rack_bay": ref_rack,
        "_print_solids": {
            "forklift_deck": comp_deck, "fork_rear_cradle": rear_cradle, "under_tie": under_tie,
            "fork_top_cover": top_cover, "fork_bay_cover": bay_cover, **print_solids,
        },
    }


def check_forklift_clearances(parts: dict) -> bool:
    """Verify 0 solid collisions across base + R46A/R46B..R58 MGN rails + shroud + switches at Home (0 mm) & +240 mm."""
    keys = (
        "base_boom", "load_cell", "live_wishbone", "tray_deck", "compartment_deck", "fork_rear_cradle",
        "fork_under_tie", "fork_top_cover", "fork_cover_fasteners",
        "fork_bay_cover", "fork_bay_cover_fasteners", "base_fasteners",
        "fork_base_rails", "fork_front_arch", "fork_mid_stage", "fork_limit_switches",
        "fork_hold_rails", "fork_hold_fasteners",
        "fork_pinion_left", "fork_pinion_right", "fork_t8_nut", "fork_mid_fasteners", "fork_tines", "fork_drive",
    )
    bad = []
    for i, k1 in enumerate(keys):
        for k2 in keys[i + 1:]:
            v = _overlap(parts[k1], parts[k2])
            if v > 1e-3:
                bad.append(("home", k1, k2, v))

    fixed_plus_f04 = bd.Compound(children=[parts[k] for k in (
        "base_boom", "load_cell", "live_wishbone", "tray_deck", "compartment_deck", "fork_rear_cradle",
        "fork_under_tie", "fork_top_cover", "fork_cover_fasteners",
        "fork_bay_cover", "fork_bay_cover_fasteners", "base_fasteners",
        "fork_base_rails", "fork_front_arch", "fork_drive", "fork_limit_switches",
        "fork_hold_rails", "fork_hold_fasteners",
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
            bad.append(("extended_240mm", name, "fixed_base", v))
    for k_ext, sh_ext in mid_ext.items():
        v_23 = _overlap(sh_ext, f03_ext)
        if v_23 > 1e-3:
            bad.append(("extended_240mm", k_ext, "f03_ext", v_23))

    f03_shelf = f03_ext.moved(bd.Location((0.0, 0.0, 10.0)))
    v_rack = _overlap(f03_shelf, parts["ref_rack_bay"])
    if v_rack > 1e-3:
        bad.append(("extended_lifted_10mm", "f03_tines", "ref_rack_bay", v_rack))

    # Verify orange fork blades (FORK_Z1 = 191.60 mm) stay clear of cogwheels, screws, arch, AND R57 hold-down <= 190.10 mm
    z_fork_top = parts["fork_tines"].bounding_box().max.Z
    for low_k in ("fork_base_rails", "fork_front_arch", "fork_mid_stage", "fork_pinion_left", "fork_pinion_right", "fork_mid_fasteners", "fork_t8_nut"):
        z_low = parts[low_k].bounding_box().max.Z
        if z_low >= z_fork_top - 0.4:
            bad.append(("z_clearance", low_k, f"max.Z={z_low:.2f} >= fork_top={z_fork_top:.2f}", z_low - z_fork_top))
    if parts["fork_hold_rails"].bounding_box().max.Z >= 190.25:
        bad.append(("crate_clearance", "fork_hold_rails", "max.Z >= 190.25", 0.0))
    if parts["fork_bay_cover"].bounding_box().max.Z >= 190.25:
        bad.append(("crate_clearance", "fork_bay_cover", "max.Z >= 190.25", 0.0))

    # Carried 12 mm-skid crate test (crate bottom seated on FORK_Z1=191.60 mm, skids hanging down to Z=179.60 mm)
    carried_load = parts["ref_load"].moved(bd.Location((0.0, 0.0, FORK_Z1 - 195.60)))
    for ck in ("tray_deck", "compartment_deck", "fork_rear_cradle", "fork_bay_cover", "fork_hold_rails", "fork_top_cover",
               "fork_front_arch", "fork_tines", "fork_cover_fasteners", "fork_bay_cover_fasteners", "fork_hold_fasteners"):
        vc = _overlap(carried_load, parts[ck])
        if vc > 1e-3:
            bad.append(("carried_crate_on_forks", ck, "collision", vc))

    if bad:
        for item in bad:
            print("  COLLISION:", item)
    else:
        print("RESULT: OK - 0 collisions at Home & +240 mm with MGN T-Rail Interlock + R57 Rails + Limit Switches")
    return len(bad) == 0


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    os.makedirs(SIM_MESH_DIR, exist_ok=True)

    for old in glob.glob(os.path.join(SIM_MESH_DIR, "*.stl")):
        os.remove(old)

    parts = build_objects_base_and_forklift()
    print_solids = parts.pop("_print_solids")
    for lbl in (
        "base_boom", "live_wishbone", "tray_deck", "compartment_deck", "fork_rear_cradle", "fork_under_tie", "fork_top_cover",
        "fork_bay_cover", "fork_front_arch", "fork_mid_stage", "fork_pinion_left", "fork_pinion_right", "fork_t8_nut",
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
    print(f"Exported {len(parts)} viewer meshes + {len(print_solids)} printable R46..R57-{BATCH} STLs")

    asm = bd.Compound(
        children=[v for k, v in parts.items() if not k.startswith("ref_")],
        label="Objects_Compartment_Forklift_CNC4759A_R46_R57",
    )
    step_path = os.path.join(OUT_DIR, "objects_forklift_assembly.step")
    bd.export_step(asm, step_path)
    print(f"Exported STEP -> {step_path}")

    ok = check_forklift_clearances(parts)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
