"""v3 assembly (concept): Squid-style robot on the v2 rack and rails.  See params_v3.py for the layout sketch.

Frames (all y = global y):
  beam    : origin on the centre joint; always level (x right, z up).  Tray, counterweight, slide, leg drives.
  leg i   : origin on the centre joint, leg along local +x; placed with rot('y', -angle).
  hub i   : origin on the hub centre, rail along local z (vertical); placed with rot('y', hub angle): 0 vertical, -90 horizontal.
  carriage: like beam, but y measured from the pin axis.   pin: like carriage, plus the spring lift in z.
Works on both csg backends (mesh and exact B-rep), like assembly_v2.
"""
import math
from csg import *                     # noqa: F401,F403
from params_v3_legs import *               # noqa: F401,F403
import assembly_v2 as A
import parts as P1
import parts_v2 as P2

MT = A.MT
C_HUB, C_CW = (240, 120, 40), (60, 60, 70)
basket_solid, rotor_items = A.basket_solid, A.rotor_items
BX = RAIL_OFF                     # bay centre (tray, slide, pin) relative to the centre joint


# ------------------------------------------------------------------ static world (v2 + longer rails + stub + servos)
_S = None


def static_items():
    global _S
    if _S is not None:
        return _S
    it = []
    add = lambda n, s, c: it.append((n, s, c))
    for bx in BAY_X:
        add("slotted angle rack (bay)", A.rack_bay(bx), A.C_ANGLE)
    for jx, jz in JUNCTIONS:
        z0 = jz + STUB0
        add("V-slot rail (vertical)", A.vslot_z(z0, RAIL_TOP_V3).translate(jx), A.C_RAIL)
        add("printed in-slot rack", P2.rack_strip(STUB0, RAIL_TOP_V3 - jz).translate(jx, 0, z0), A.C_PRINT)
        add("printed rail bridge", P2.rail_bridge().translate(jx, 0, RAIL_TOP - 45), A.C_PRINT)
        bp = extrude(A.base_plate_outline(), -31, -22, 'y').translate(jx, 0, jz)
        add("junction base plate (9 mm plywood)", bp, A.C_WOOD)
        for (rx, rz) in RECV:
            add("torque pin sleeve", P2.pin_sleeve().rotate('x', 90).translate(jx + rx, -31, jz + rz), A.C_JUNC)
            add("M8 nut (pin tip)", hexagon(13, PIN_TIP_Y, PIN_TIP_Y + 6.5, (jx + rx, jz + rz), 'y'), A.C_BOLT)
        add("printed rail bridge", P2.rail_bridge().translate(jx, 0, jz + 165), A.C_PRINT)
        add("printed detent", P2.detent().rotate('y', 180).translate(jx, 0, jz), A.C_JUNC)
        add("printed detent", P2.detent().transform(A.MX()).rotate('y', -90).translate(jx, 0, jz), A.C_JUNC)
        add("MG996R junction servo (sets the empty piece)", box(jx - 20, jx + 20, -22, -3, jz + 60, jz + 100), A.C_SERVO)
    (x1, jz), (x2, _) = JUNCTIONS[0], JUNCTIONS[1]
    for xa, xb in ((x1 + STUB0, x2 - STUB0), (x2 + STUB0, X_STUB_END)):
        add("V-slot rail (horizontal)", A.vslot_z(0, xb - xa).rotate('y', 90).translate(xa, 0, jz), A.C_RAIL)
        add("printed in-slot rack", P2.rack_strip(0, xb - xa).rotate('y', 90).translate(xa, 0, jz), A.C_PRINT)
    _S = it
    return it


# ------------------------------------------------------------------ robot pieces
def hub_local(i):
    """hub unit: plate, 4 V-wheels, climb pinion + JGY-370 under a printed cover, swivel disc to leg i"""
    it = []
    add = lambda n, s, c: it.append((n, s, c))
    h = HUB_PLATE_L / 2
    add("hub plate (6 mm aluminium)", box(-PLATE_W / 2, PLATE_W / 2, PLATE_Y0, PLATE_Y1, -h, h), A.C_PLATE)
    for a in (-HUB_WHEEL_A, HUB_WHEEL_A):
        for sx in (-1, 1):
            add("V wheel", cyl(WHEEL_R, WHEEL_YC - WHEEL_W / 2, WHEEL_YC + WHEEL_W / 2, 'y', (sx * WHEEL_DX, a), 28), A.C_WHEEL)
            add("spacer", cyl(5, PLATE_Y1, WHEEL_YC - WHEEL_W / 2, 'y', (sx * WHEEL_DX, a), 12), A.C_STEEL)
    pa = CP_OUT
    add("printed climber pinion", P2.climber_pinion().translate(0, 0, -CP_W / 2).rotate('y', 90).translate(0, CP_Y, pa), A.C_PRINT2)
    add("climber shaft", cyl(3, 0, 13, 'x', (CP_Y, pa), 12), A.C_STEEL)
    add("JGY-370 climb motor", box(11, 33.5, CP_Y - 26, CP_Y + 10, pa - 16, pa + 16), A.C_GB)
    add("JGY-370 climb motor can", cyl(12.2, pa + 16, pa + 46, 'z', (22.25, CP_Y - 13), 24), A.C_CAN)
    cover = cyl(HUB_R1, HUB_Y[0], HUB_Y[1], 'y', (0, 0), 48) - cyl(HUB_R0, HUB_Y[0] + 3, HUB_Y[1] + 1, 'y', (0, 0), 48)
    add("printed hub cover", cover, C_HUB)
    add("swivel bearing (hub)", cyl(22, LEG_Y[i][1], HUB_Y[0], 'y', (0, 0), 32), A.C_STEEL)
    return it


def leg_local(i):
    """leg i from the centre joint (0) to the hub (L_LEG along +x): printed link, 60T gear, swivel motor at the hub end"""
    y0, y1 = LEG_Y[i]
    L = L_LEG
    link = box(0, L, y0, y1, -9, 9) + cyl(14, y0, y1, 'y', (0, 0), 32) + cyl(14, y0, y1, 'y', (L, 0), 32)
    return [("printed leg", link, (40, 170, 160) if i == 1 else (130, 90, 230)),
            ("printed leg gear 60T (envelope)", cyl(GEAR_R, y0, y1, 'y', (0, 0), 48) - cyl(15, y0 - 1, y1 + 1, 'y', (0, 0), 24), A.C_PRINT),
            ("JGY-370 swivel motor", box(L - 11.25, L + 11.25, y0 - 22.5, y0, -23, 23), A.C_GB),
            ("JGY-370 swivel motor can", cyl(12.2, L - 11.25 - 33, L - 11.25, 'x', (y0 - 11.25, 0), 24), A.C_CAN)]


_BM = None


def beam_local():
    """level beam: centre shaft, two leg drives, tray, counterweight, slide rail + motor"""
    global _BM
    if _BM is not None:
        return _BM
    it = []
    add = lambda n, s, c: it.append((n, s, c))
    T, by0, by1 = T_V3, BEAM_Y[0], BEAM_Y[1]
    add("aluminium beam 20x20", box(CW_X[0], 105, by0, by1, -10, 10), A.C_PLATE)
    add("centre shaft (M10)", cyl(5, by0 - 1, LEG_Y[1][1] + 2, 'y', (0, 0), 16), A.C_STEEL)
    for i in (1, 2):
        a = math.radians(PINION_ANG[i]); r = GEAR_R + 10.5
        px, pz = r * math.cos(a), r * math.sin(a)
        add("printed leg pinion 12T", cyl(10.5, LEG_Y[i][0], LEG_Y[i][1], 'y', (px, pz), 24), A.C_PRINT2)
        add("leg drive shaft", cyl(3, by1 + 2, LEG_Y[i][0], 'y', (px, pz), 12), A.C_STEEL)
        add("JGY-370 leg drive motor", box(px - 16, px + 16, by1 + 2, by1 + 24.5, pz - 16, pz + 16), A.C_GB)
    add("counterweight: battery + electronics box", box(CW_X[0], CW_X[1], CW_Y[0], CW_Y[1], CW_Z[0], CW_Z[1]), C_CW)
    add("tray bracket", box(95, 105, by0, -3, T - TRAY_T, 10), A.C_PLATE)
    add("plywood tray 12 mm", box(105, 380, -330, -3, T - TRAY_T, T), A.C_WOOD)
    for sx in (-80, 80):
        add("printed basket stop", P1.basket_stop().transform(MT(BX + sx, -193, T)), A.C_PRINT)
    add("limit switch (basket on tray)", box(-10, 10, -193, -186.6, 13, 23).translate(BX - 80, 0, T), A.C_SW)
    zs = T + SLIDE_Z
    add("V-slot 2020 slide rail", box(BX - 10, BX + 10, SLIDE_Y[0], SLIDE_Y[1], zs, zs + 20), A.C_RAIL)
    add("slide rear post", box(BX - 10, BX + 10, -330, SLIDE_Y[0], T, zs + 20), A.C_PLATE)
    add("N20 gear motor + GT2 pulley (slide)", box(BX + 10, BX + 32, -330, -305, zs, zs + 20), A.C_GB)
    add("GT2 belt", box(BX + 10, BX + 12, SLIDE_Y[0], SLIDE_Y[1], zs + 4, zs + 16), A.C_SW)
    _BM = it
    return it


def carriage_local():
    """slide carriage + pin arm + spring barrel + lock servo; y measured from the pin axis"""
    T, zs = T_V3, T_V3 + SLIDE_Z
    return [("printed slide carriage", box(BX - 15, BX + 15, -PIN_ARM - CARR_L, -PIN_ARM, zs - 25, zs), A.C_PRINT),
            ("printed pin arm", box(BX - 6, BX + 6, -PIN_ARM, 8, T + 118, T + 128), A.C_PRINT),
            ("printed spring barrel", cyl(8, T + 112, T + 140, 'z', (BX, 0), 24) - cyl(4.3, T + 111, T + 141, 'z', (BX, 0), 16), A.C_PRINT2),
            ("SG90 pin lock servo", box(BX + 8, BX + 31, -11, 1, T + 128, T + 150), A.C_SERVO)]


def pin_local():
    T = T_V3
    return [("spring-loaded hook pin", cyl(1.5, T + PIN_DOWN, T + PIN_DOWN + 8, 'z', (BX, 0), 16, r1=4)
             + cyl(4, T + PIN_DOWN + 8, T + 130, 'z', (BX, 0), 16), A.C_STEEL)]


# ------------------------------------------------------------------ poses
def state(cx, cz, a1, a2, h1=0.0, h2=0.0, rot=(0.0, -90.0), pin=PIN_STOW, lift=0.0):
    return dict(cx=cx, cz=cz, a1=a1, a2=a2, h1=h1, h2=h2, rot=list(rot), pin=pin, lift=lift)


def build(S, baskets=None):
    """S: state() dict.  baskets: list of (bay, level) on the shelf, 'tray', or ('at', y0) = front wall y on the tray line"""
    items = [(n, s, c, "static") for n, s, c in static_items()]
    for j, a in enumerate(S["rot"]):
        items += [(n, s, c, "rotor") for n, s, c in rotor_items(j, a)]
    cx, cz = S["cx"], S["cz"]
    Mc = MT(cx, 0, cz)
    items += [(n, s.transform(Mc), c, "beam") for n, s, c in beam_local()]
    items += [(n, s.transform(MT(cx, S["pin"], cz)), c, "slide") for n, s, c in carriage_local()]
    items += [(n, s.transform(MT(cx, S["pin"], cz + S["lift"])), c, "slide") for n, s, c in pin_local()]
    for i in (1, 2):
        items += [(f"{n} ({i})", s.transform(Mc @ rot('y', -S["a%d" % i])), c, f"leg{i}") for n, s, c in leg_local(i)]
        hx, hz = hub_pos(S, i)
        items += [(f"{n} (hub {i})", s.transform(MT(hx, 0, hz) @ rot('y', S["h%d" % i])), c, f"hub{i}") for n, s, c in hub_local(i)]
    bs = basket_solid()
    T = cz + T_V3
    for k, b in enumerate(baskets or []):
        if b == 'tray':
            items.append(("basket (on tray)", bs.translate(cx + BX, -193, T), A.BASKET_COLS[k % 10], "beam"))
        else:
            bay, lv = b
            items.append((f"basket bay{bay+1}-L{lv+1}", bs.translate(BAY_X[bay], BASKET_Y0, LEVELS[lv]), A.BASKET_COLS[k % 10], "static"))
    return items


def junction_state(j, a, b, h1, h2, rot):
    jx, jz = JUNCTIONS[j]
    s = walk_state(jx, jz, a, b)
    return state(s["cx"], s["cz"], s["a1"], s["a2"], h1, h2, rot)


def pose(name):
    """named poses for the CAD export"""
    B_all = [(b, l) for b in range(N_BAYS) for l in range(len(LEVELS))]
    others = [x for x in B_all if x != (0, 2)]
    J1 = JUNCTIONS[0][0]
    if name == "pick":       # bay 1, shelf 3, pin hooked behind the basket wall
        v = vertical_state(J1, cz_for_level(2))
        return build(state(v["cx"], v["cz"], v["a1"], v["a2"], 0, 0, (0, -90), PIN_PULL), B_all)
    if name == "walk":       # half way round junction 1, hub 1 on the vertical rail, hub 2 on the horizontal rail
        return build(junction_state(0, *WALK[1], 0, -90, (0, -90)), others + ['tray'])
    if name == "swivel":     # hub 2 on the turntable, swivelled half way
        return build(junction_state(0, *WALK[2], 0, -45, (-45, -90)), others + ['tray'])
    if name == "travel":     # on the bottom rail between the bays
        t = travel_state((JUNCTIONS[0][0] + JUNCTIONS[1][0]) / 2 + TRAVEL_HALF)
        return build(state(t["cx"], t["cz"], t["a1"], t["a2"], -90, -90, (-90, -90)), others + ['tray'])
    raise ValueError(name)


POSES = ("pick", "walk", "swivel", "travel")
