"""v3 assembly (concept): compact Squid layout on the v2 rack.  See the sketch in params_v3.py.

Frames (face-local: x along the face, y into the rack; face k = face 0 turned k*90 deg about the tower centre):
  robot : origin C = midway between the two hubs; never rotates.  Column, swivel drives, battery box, tray, arms (outer).
  hub   : origin on the hub centre, rail along local z; placed with rot('y', h): 0 = vertical rail, -90 = horizontal.
  crossing piece: origin on the crossing, rot('y', r): 0 = vertical, -90 = horizontal.
  arm bars: robot frame shifted along y (middle d, inner 2 d); fingers turn about y on the finger shafts.
Works on both csg backends (mesh and exact B-rep), like assembly_v2.
"""
import math
from functools import lru_cache
import numpy as np
from csg import *                     # noqa: F401,F403
from params_v3 import *               # noqa: F401,F403
import assembly_v2 as A
import parts as P1
import parts_v2 as P2

MT = A.MT
C_HUB, C_BAT, C_ARM = (240, 120, 40), (60, 60, 70), (175, 180, 190)
basket_solid = A.basket_solid
SIDES = (-1, 1)                        # left / right arm


# ------------------------------------------------------------------ floor layout: rows + transfer cart
def row_M(r):
    """row r: front plane y = 0 in its frame, bays at ROW_BAY_X, rack behind (y > 0); rows are ROW_P apart, all facing -y"""
    return MT(0, r * ROW_P, 0)


def robot_world(S):
    """robot frame (origin C, between the hubs) -> world.  The robot never turns: it is at y = S['ry'] (a row's y, or
    the cart's y while it rides the cart)"""
    return MT(S["cx"], S["ry"], S["cz"])


def bay_row(b):
    return b // ROW_BAYS


def bay_rail_x(b):
    """vertical rail of (global) bay b, row-frame x"""
    return ROW_BAY_X[b % ROW_BAYS] - RAIL_OFF


# ------------------------------------------------------------------ static world
_S = None


def row_static_local():
    """one row in its own frame: the bays, a vertical rail + crossing brackets per bay, the row's two horizontal rails
    (they end at the cart's rail pieces)"""
    it = []
    add = lambda n, s, c: it.append((n, s, c))
    g = SEG_V3 + GAP_V3
    for bx in ROW_BAY_X:
        add("slotted angle rack (bay)", A.rack_bay(bx), A.C_ANGLE)
        jx = bx - RAIL_OFF
        z0 = JZ_UP + g
        add("V-slot rail (vertical)", A.vslot_z(z0, RAIL_TOP_V3).translate(jx), A.C_RAIL)
        add("printed in-slot rack", P2.rack_strip(0, RAIL_TOP_V3 - z0).translate(jx, 0, z0), A.C_PRINT)
        add("V-slot rail (vertical, between crossings)", A.vslot_z(JZ_LO + g, JZ_UP - g).translate(jx), A.C_RAIL)
        add("printed in-slot rack", P2.rack_strip(0, JZ_UP - JZ_LO - 2 * g).translate(jx, 0, JZ_LO + g), A.C_PRINT)
        for zb in (RAIL_TOP - 45, 650.0):
            add("printed rail bridge", P2.rail_bridge().translate(jx, 0, zb), A.C_PRINT)
        for zc in CROSS_Z:      # small printed bracket from the bearing to the next rack post
            add("printed crossing bracket", box(jx - 18, jx + 60, -31, -22, zc - 15, zc + 15), A.C_PRINT)
    cuts = [ROW_RAIL_X[0]] + [v for bx in ROW_BAY_X for v in (bx - RAIL_OFF - g, bx - RAIL_OFF + g)] + [ROW_RAIL_X[1]]
    for x1, x2 in zip(cuts[0::2], cuts[1::2]):
        for zc in CROSS_Z:
            add("V-slot rail (horizontal)", A.vslot_z(0, x2 - x1).rotate('y', 90).translate(x1, 0, zc), A.C_RAIL)
            add("printed in-slot rack", P2.rack_strip(0, x2 - x1).rotate('y', 90).translate(x1, 0, zc), A.C_PRINT)
    # rail end support beside the last rack (the rails run on ~130 mm past it to meet the cart)
    add("printed rail end bracket", box(ROW_RAIL_X[1] - 20, ROW_RAIL_X[1], RAIL_Y + 10, RAIL_Y + 40, 0, JZ_UP + 12), A.C_PRINT)
    return it


def xfer_rack():
    """fixed straight gear rack (module 1.5) along the cart track, teeth facing -x toward the robot's pinion"""
    p, m = math.pi * 1.5, 1.5
    x0 = XFER_RACK_X
    y0, y1 = TRACK_Y
    pts = [(x0 + 1.25 * m + 4, y0)]
    y = y0
    while y + p <= y1:
        pts += [(x0 + 1.25 * m, y), (x0 - m, y + p * 0.25), (x0 - m, y + p * 0.5), (x0 + 1.25 * m, y + p * 0.75)]
        y += p
    pts += [(x0 + 1.25 * m, y), (x0 + 1.25 * m + 4, y)]
    zg0, zg1 = CZ_HOME + TURN_GEAR_Z[0], CZ_HOME + TURN_GEAR_Z[1]
    return extrude(pts, zg0, zg1)


def xfer_static():
    """the side bar: floor track for the cart, the transfer gear rack on a beam, and its posts beyond the robot's tray"""
    zg0, zg1 = CZ_HOME + TURN_GEAR_Z[0], CZ_HOME + TURN_GEAR_Z[1]
    y0, y1 = TRACK_Y
    xb = XFER_RACK_X + 1.25 * 1.5 + 4
    it = [("V-slot cart track (floor, 20x40)", box(X_CART - 20, X_CART + 20, y0, y1, 0, 20), A.C_RAIL),
          ("printed transfer rack (module 1.5)", xfer_rack(), A.C_PRINT),
          ("transfer rack beam (20x20 V-slot)", box(xb, xb + 20, y0, y1, zg0 - 8, zg1 + 4), A.C_RAIL)]
    for y in [y0 + 20] + [r * ROW_P + RACK_D / 2 for r in range(N_ROWS)] + [y1 - 20]:   # posts in the racks' depth band
        it += [("transfer beam arm", box(xb + 20, XFER_POST_X, y - 10, y + 10, zg1 - 4, zg1 + 4), A.C_RAIL),
               ("transfer beam post", box(XFER_POST_X, XFER_POST_X + 20, y - 10, y + 10, 0, zg1 + 4)
                + box(XFER_POST_X - 40, XFER_POST_X + 60, y - 30, y + 30, 0, 6), A.C_RAIL)]
    return it


def static_items():
    global _S
    if _S is None:
        loc = row_static_local()
        _S = [(f"{n} (row {r+1})", s.transform(row_M(r)), c) for r in range(N_ROWS) for n, s, c in loc]
        _S += xfer_static()
    return _S


_CA = None


def cart_local():
    """transfer cart at y = 0 (lined up with row 1): two short rail pieces (with racks) that continue the rows' rails, a
    frame behind them, wheels on the floor track and a guide roller on the rack beam.  No motor."""
    global _CA
    if _CA is None:
        x0, x1 = CART_PIECE
        it = []
        for zc in CROSS_Z:
            it += [("rail piece on cart", A.vslot_z(0, x1 - x0).rotate('y', 90).translate(x0, 0, zc), A.C_RAIL),
                   ("printed in-slot rack (cart)", P2.rack_strip(0, x1 - x0).rotate('y', 90).translate(x0, 0, zc), A.C_PRINT)]
        yb = RAIL_Y + 10                       # behind the rails (the robot stays in front of y -40)
        it += [("cart frame (20x40 V-slot)", box(x0, x0 + 20, yb, yb + 30, 20, JZ_UP + 15) + box(x1 - 20, x1, yb, yb + 30, 20, JZ_UP + 15)
                + box(x0, x1, yb, yb + 30, JZ_UP + 15, JZ_UP + 35) + box(x0, x1, yb, yb + 30, 20, 40), A.C_JUNC),
               ("cart wheels (on the floor track)", box(X_CART - 30, X_CART + 30, yb - 6, yb + 36, 0, 20) - box(X_CART - 21, X_CART + 21, yb - 7, yb + 37, -1, 21), A.C_WHEEL),
               ("cart guide arm + roller (on the rack beam)", box(x1 - 20, XFER_RACK_X + 30, yb, yb + 20, JZ_UP + 15, JZ_UP + 30), A.C_JUNC),
               ("spring detent (holds the cart at a row)", box(X_CART - 6, X_CART + 6, yb, yb + 30, 40, 52), A.C_SW)]
        _CA = it
    return _CA


_R = None


def crossing_local():
    global _R
    if _R is None:
        h = SEG_V3
        _R = [("50 mm turntable bearing", cyl(BEARING_R, -40, -32, 'y', (0, 0), 32), A.C_JUNC),
              ("rail piece on turntable", A.vslot_z(-h, h), A.C_RAIL),
              ("printed in-slot rack (turntable)", P2.rack_strip(0, 2 * h).translate(0, 0, -h), A.C_PRINT)]
    return _R


def crossing_items(b, k, ang):
    """crossing piece k (0 lower, 1 upper) at (global) bay b's vertical rail"""
    M = row_M(bay_row(b)) @ MT(bay_rail_x(b), 0, CROSS_Z[k]) @ rot('y', ang)
    return [(n, s.transform(M), c) for n, s, c in crossing_local()]


# ------------------------------------------------------------------ robot
_H = None


def hub_local():
    global _H
    if _H is not None:
        return _H
    it = []
    add = lambda n, s, c: it.append((n, s, c))
    h = HUB_PLATE_L / 2
    add("hub plate (6 mm aluminium)", box(-PLATE_W / 2, PLATE_W / 2, PLATE_Y0, PLATE_Y1, -h, h), A.C_PLATE)
    for a in (-HUB_WHEEL_A, HUB_WHEEL_A):
        for sx in (-1, 1):
            add("V wheel", cyl(WHEEL_R, WHEEL_YC - WHEEL_W / 2, WHEEL_YC + WHEEL_W / 2, 'y', (sx * WHEEL_DX, a), 28), A.C_WHEEL)
            add("spacer", cyl(5, PLATE_Y1, WHEEL_YC - WHEEL_W / 2, 'y', (sx * WHEEL_DX, a), 12), A.C_STEEL)
    pa = CP_OUT
    add("climber shaft", cyl(3, 0, 13, 'x', (CP_Y, pa), 12), A.C_STEEL)
    add("JGY-370 climb motor", box(11, 33.5, CP_Y - 26, CP_Y + 10, pa - 16, pa + 16), A.C_GB)
    add("JGY-370 climb motor can", cyl(12.2, pa + 16, pa + 46, 'z', (22.25, CP_Y - 13), 24), A.C_CAN)
    add("printed hub cover", cyl(HUB_R1, HUB_Y[0], HUB_Y[1], 'y', (0, 0), 48) - cyl(HUB_R0, HUB_Y[0] - 1, HUB_Y[1] + 1, 'y', (0, 0), 48), C_HUB)
    add("swivel axle", cyl(8, COL_Y[1], PLATE_Y0, 'y', (0, 0), 20), A.C_STEEL)
    _H = it
    return it


def hub_pinion_local():
    """climb pinion of a hub, centred on its own axle (axis x); placed at (0, CP_Y, CP_OUT) in the hub frame and
    turned by (distance travelled) / pitch radius"""
    return [("printed climber pinion", P2.climber_pinion().translate(0, 0, -CP_W / 2).rotate('y', 90), A.C_PRINT2)]


PINION_AT = (0.0, CP_Y, CP_OUT)


# ------------------------------------------------------------------ picker (2 servos, every moving part modelled)
#
#   side view of one arm (y ->, into the rack)                      front view of the drive (tray front)
#
#     inner bar   =============[D]  <- fingers on a square tube       rod L   servo 2    rod R
#     B]=====upper runs=========(P)  2:1 pulleys on the middle bar      (G)<->(g)~~~belt~~~(p)
#     middle bar (P)=============                                       gear pair reverses the left side
#     A]=====lower runs====[C]                                        servo 1 -> step-up belt -> cross shaft
#     outer bar  ================  fixed on the tray                   -> one belt loop beside each arm
#
C_PAD, C_BELT = (235, 235, 225), (30, 30, 30)
BELT_T = 1.2
MID_C = (MID_Z[0] + MID_Z[1]) / 2           # stage-2 pulley axis height (tray-relative)


def T_(zt):
    """tray-relative height -> robot frame z"""
    return T_V3 + zt


def lane(sx, u0, u1):
    """robot-frame x range (lo, hi) of a lane u0..u1 measured from the arm's inner face (basket side) outward"""
    a, b = BXR + sx * (ARM_X[0] + u0), BXR + sx * (ARM_X[0] + u1)
    return min(a, b), max(a, b)


def arm_xm(sx):
    return BXR + sx * (ARM_X[0] + ARM_W / 2)


ARM_DRIVE_AT = (BXR, S1_Y[0], T_V3 + S1_ZT)          # cross shaft axis (robot frame)
IDLER_AT = (BXR, S1_Y[1], T_V3 + S1_ZT)              # stage-1 idlers (both arms, one axis)
SERVO1_POS = (BXR, SERVO1_AT[0], T_V3 + SERVO1_AT[1])
FSERVO_POS = (arm_xm(-1) + FSERVO_DX, 0.0, T_V3 + FS_ZT)


def fshaft_at(sx):
    return (arm_xm(sx), 0.0, T_V3 + FS_ZT)


def dpul_at(sx, k):
    """stage-2 pulley k (0 extend, 1 retract) centre, stowed"""
    L = LANE_EXT if k == 0 else LANE_RET
    xa, xb = lane(sx, *L)
    return ((xa + xb) / 2, DPUL_Y[k], T_(MID_C))


def _hull(circles, n=40):
    """convex hull (CCW) of circles (a, b, r)"""
    pts = sorted({(round(a + r * math.cos(t), 5), round(b + r * math.sin(t), 5))
                  for a, b, r in circles for t in np.linspace(0, 2 * math.pi, n, endpoint=False)})

    def cr(o, p, q):
        return (p[0] - o[0]) * (q[1] - o[1]) - (p[1] - o[1]) * (q[0] - o[0])
    lo, up = [], []
    for p in pts:
        while len(lo) >= 2 and cr(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(up) >= 2 and cr(up[-2], up[-1], p) <= 0:
            up.pop()
        up.append(p)
    return lo[:-1] + up[:-1]


def belt_loop(p, q, plane, h0, h1):
    """closed belt around two pulleys p, q = (a, b, pitch r).  plane 'yz': (a, b) = (y, z), width along x h0..h1;
    plane 'xz': (a, b) = (x, z), width along y h0..h1"""
    ma, mb = (p[0] + q[0]) / 2, (p[1] + q[1]) / 2
    rel = lambda c, dr: (c[0] - ma, c[1] - mb, c[2] + dr)
    out, inn = _hull([rel(p, BELT_T / 2), rel(q, BELT_T / 2)]), _hull([rel(p, -BELT_T / 2), rel(q, -BELT_T / 2)])
    if plane == 'yz':
        M = np.array([[0, 0, 1, 0], [1, 0, 0, ma], [0, 1, 0, mb], [0, 0, 0, 1]], float)
        return extrude_ring(out, inn, h0, h1).transform(M)
    M = np.array([[1, 0, 0, ma], [0, 0, -1, 0], [0, 1, 0, mb], [0, 0, 0, 1]], float)
    return extrude_ring(out, inn, -h1, -h0).transform(M)


def half_wrap(xr, yc, zc, r, side):
    """belt wrapped half way round a pulley (axis x) on its +y (side 1) or -y (side -1) side"""
    t = np.linspace(-math.pi / 2, math.pi / 2, 13)
    ro, ri = r + BELT_T / 2, r - BELT_T / 2
    pts = [(yc + side * ro * math.cos(a), zc + ro * math.sin(a)) for a in t] + \
          [(yc + side * ri * math.cos(a), zc + ri * math.sin(a)) for a in t[::-1]]
    return extrude(pts, xr[0], xr[1], 'x')


def gear_x(z, x0, w):
    """spur gear / toothed pulley on an x axis through the origin, face x0..x0+w"""
    return P2.gear_part(z, w).rotate('y', 90).translate(x0, 0, 0)


def gear_y(z, y0, w):
    return P2.gear_part(z, w).rotate('x', -90).translate(0, y0, 0)


def pulley_x(r, x0, w):
    """smooth flanged pulley (stage-2), three holes so its turning shows"""
    s = cyl(r - BELT_T / 2, x0, x0 + w, 'x', (0, 0), 28) + cyl(r + 1.0, x0, x0 + 0.6, 'x', (0, 0), 28) \
        + cyl(r + 1.0, x0 + w - 0.6, x0 + w, 'x', (0, 0), 28)
    for k in range(3):
        a = 2 * math.pi * k / 3
        s = s - cyl(1.1, x0 - 1, x0 + w + 1, 'x', (2.9 * math.cos(a), 2.9 * math.sin(a)), 12)
    return s


def mech_angles(arm, f):
    """joint angles (deg) of every turning part for arm tip position `arm` and finger angle f (0 up, 90 in)"""
    d = (arm - TIP0) / 2                                    # middle bar travel; the inner bar moves 2 d
    drive = -math.degrees(d / PIN_R)                        # 33T pulleys (pitch r = PIN_R) on the cross shaft
    return dict(d=d, drive=drive, servo1=drive * STEP_T[1] / STEP_T[0], dpul=-math.degrees(d / DPUL_R),
                finger_l=f, finger_r=-f, servo2=-f)        # belt: servo 2 turns with the right rod; gear: left opposite


def turn_pinion_angle(cart_y):
    """the robot's pinion rolls along the fixed transfer rack while it pushes the cart"""
    return -math.degrees(cart_y / TURN_PIN_R)


def finger_angle(sx, f):
    return -sx * f


@lru_cache(None)
def arm_drive_local():
    """cross shaft: a 33T pulley for each arm's stage-1 belt + the 8T pulley of the step-up belt; centred on its axis"""
    xe = ARM_X[0] + LANE_S1[1] + 1.5
    it = [("arm drive cross shaft (both arms locked together)", cyl(3, -xe, xe, 'x', (0, 0), 12), A.C_STEEL)]
    for sx in SIDES:
        xa, _ = lane(sx, *LANE_S1)
        it.append(("GT2 33T pulley (stage-1 drive)", gear_x(14, xa - BXR, LANE_S1[1] - LANE_S1[0]), A.C_PRINT2))
    it.append(("GT2 8T pulley (step-up, cross shaft)", gear_x(STEP_T[1], SERVO1_X - 2.5, 5), A.C_PRINT2))
    return it


@lru_cache(None)
def idler_local():
    it = []
    for sx in SIDES:
        xa, _ = lane(sx, *LANE_S1)
        it.append(("GT2 idler (stage-1 belt)", gear_x(14, xa - BXR, LANE_S1[1] - LANE_S1[0]), A.C_PRINT2))
    return it


@lru_cache(None)
def servo1_local():
    return [("GT2 25T pulley (servo 1)", gear_x(STEP_T[0], SERVO1_X - 2.5, 5), A.C_PRINT2)]


@lru_cache(None)
def fshaft_local(sx):
    """fixed square rod of one arm's finger shaft (+ its drive gear or belt pulley), axis y through (0, 0)"""
    h = FS_SQ[0] / 2
    it = [("finger shaft: fixed square rod", box(-h, h, FRONT_Y[0] - 2, TIP0, -h, h), A.C_STEEL)]
    if sx < 0:
        it.append(("finger drive gear (left rod)", gear_y(14, FG_Y[0], FG_Y[1] - FG_Y[0]), A.C_PRINT2))
    else:
        it.append(("GT2 pulley (right rod)", gear_y(14, FP_Y[0], FP_Y[1] - FP_Y[0]), A.C_PRINT2))
    return it


@lru_cache(None)
def fservo_local():
    return [("finger drive gear (servo 2)", gear_y(14, FG_Y[0], FG_Y[1] - FG_Y[0]), A.C_PRINT2),
            ("GT2 pulley (servo 2)", gear_y(14, FP_Y[0], FP_Y[1] - FP_Y[0]), A.C_PRINT2),
            ("servo 2 output shaft", cyl(2.5, FP_Y[0] - 3, FG_Y[1], 'y', (0, 0), 12), A.C_STEEL)]


@lru_cache(None)
def ftube_mid_local(sx):
    h = FS_SQ[1] / 2
    return [("finger shaft: middle square tube", box(-h, h, FT_MID_Y[0], FT_MID_Y[1], -h, h), A.C_STEEL)]


@lru_cache(None)
def finger_local(sx, end=None):
    """inner square tube + front and rear flip finger, axis y through (0, 0), stowed (fingers pointing up);
    turned by finger_angle(sx, f) about y they swing in over the basket walls"""
    h, w = FS_SQ[2] / 2, FINGER_W / 2
    it = [("finger shaft: inner square tube", box(-h, h, TIP0 - REAR_OFF - FINGER_T - 2, TIP0, -h, h), A.C_STEEL)]
    for e, y1 in (("front", TIP0), ("rear", TIP0 - REAR_OFF)):
        if end and e != end:
            continue
        f = box(-w, w, y1 - FINGER_T, y1, 0, FINGER_L) + cyl(h + 2, y1 - FINGER_T, y1, 'y', (0, 0), 24)   # round collar
        it.append((f"printed flip finger ({e})", f, A.C_PRINT2))
    return it if end is None else it[1:]


_RB = None


def robot_local():
    """fixed to the column: swivel drives, battery box, tray on a load cell, outer bars, both servos, belts that don't
    change shape"""
    global _RB
    if _RB is not None:
        return _RB
    it = []
    add = lambda n, s, c: it.append((n, s, c))
    hz = D_HUB / 2
    add("printed column", box(-14, 14, COL_Y[0], COL_Y[1], -hz - 20, hz + 20), A.C_PRINT)
    for z in (hz, -hz):
        add("JGY-370 swivel motor", box(-11.25, 11.25, COL_Y[0] - 22.5, COL_Y[0], z - 23, z + 23), A.C_GB)
        add("JGY-370 swivel motor can", cyl(12.2, 11.25, 44.25, 'x', (COL_Y[0] - 11.25, z), 24), A.C_CAN)
    add("battery + electronics box (fixed counterweight)", box(BAT_X[0], BAT_X[1], BAT_Y[0], BAT_Y[1], BAT_Z[0], BAT_Z[1]), C_BAT)
    add("battery bracket", box(BAT_X[1], -14, COL_Y[0], COL_Y[1], -15, 15), A.C_PLATE)
    T, x = T_V3, BXR
    add("tray bracket", box(14, x - ARM_X[1] - 2, COL_Y[0], COL_Y[1], T - TRAY_T, T), A.C_PLATE)
    add("bar load cell 5 kg + HX711", box(40, 76, COL_Y[0] + 2, COL_Y[1] - 2, T - TRAY_T - 13, T - TRAY_T), A.C_PCB)
    add("plywood tray 12 mm", box(x - ARM_X[1], x + ARM_X[1], TRAY_Y[0], TRAY_Y[1], T - TRAY_T, T), A.C_WOOD)
    for sx in (-80, 80):
        add("printed basket stop", P1.basket_stop().transform(MT(x + sx, BASKET_TRAY_Y0, T)), A.C_PRINT)
    add("limit switch (basket on tray)", box(-10, 10, BASKET_TRAY_Y0, BASKET_TRAY_Y0 + 6.4, 13, 23).translate(x - 80, 0, T), A.C_SW)
    for sx in SIDES:
        xa, xb = lane(sx, 0, ARM_W)
        add("telescopic arm, outer bar (fixed)", box(xa, xb, ARM_Y0, TIP0, T_(OUT_Z[0]), T_(OUT_Z[1])), C_ARM)
        add("printed arm post", box(xa, xb, ARM_Y0, ARM_Y0 + 15, T, T_(OUT_Z[0])), A.C_PRINT)
        add("printed arm post (rear, idler axle)", box(xa, xb, S1_Y[1] - 6, S1_Y[1] + 6, T, T_(OUT_Z[0])), A.C_PRINT)
        add("idler axle", cyl(2, *lane(sx, ARM_W, LANE_S1[1] + 1), 'x', (S1_Y[1], T_(S1_ZT)), 12), A.C_STEEL)
        # front post: carries the finger rod's bearing (top) and the cross shaft's bearing (boss in front)
        add("printed front post (rod + cross-shaft bearings)",
            box(xa, xb, FRONT_Y[0], FRONT_Y[1], T, T_(FS_ZT + 8)) + box(xa, xb, S1_Y[0] - 6, FRONT_Y[0], T_(S1_ZT - 9), T_(S1_ZT + 9)),
            A.C_PRINT)
        # stage 1: belt loop beside the arm, from the cross-shaft pulley to the idler
        add("GT2 belt loop (stage 1)", belt_loop((S1_Y[0], T_(S1_ZT), PIN_R), (S1_Y[1], T_(S1_ZT), PIN_R), 'yz', *lane(sx, *LANE_S1)), C_BELT)
        # stage 2 belt anchors on the outer bar (extend belt at its rear, retract belt at its front)
        add("belt anchor (outer bar)", box(*lane(sx, *LANE_EXT), ARM_Y0 + 2, ARM_Y0 + 6, T_(OUT_Z[1]), T_(OUT_Z[1] + 2.5)), A.C_SW)
        add("belt anchor (outer bar)", box(*lane(sx, *LANE_RET), TIP0 - 6, TIP0 - 2, T_(OUT_Z[1]), T_(OUT_Z[1] + 2.5)), A.C_SW)
    # servo 1 (arm stroke) under the tray front + step-up belt to the cross shaft
    sx0 = BXR + SERVO1_X
    sy, sz = SERVO1_AT
    add("servo 1: DS3218 270 deg (arm stroke)", box(sx0 + 3, sx0 + 23, sy - 22, sy + 18, T_(sz - 20), T_(sz + 14)), A.C_SERVO)
    add("servo 1 bracket", box(sx0 + 3, sx0 + 23, sy + 12, TRAY_Y[0] + 8, T_(sz + 14), T - TRAY_T), A.C_PLATE)
    add("GT2 belt (step-up 25T -> 8T)", belt_loop((S1_Y[0], T_(S1_ZT), 1.5 * STEP_T[1] / 2), (sy, T_(sz), 1.5 * STEP_T[0] / 2),
                                                   'yz', sx0 - 2.5, sx0 + 2.5), C_BELT)
    # servo 2 (fingers) in front of the left rod: gear pair to the left rod, belt across the tray front to the right rod
    fx, fz = FSERVO_POS[0], FSERVO_POS[2]
    add("servo 2: MG90S (all four fingers)", box(fx - 6, fx + 17, FRONT_Y[1] - 30, FRONT_Y[1] - 2, fz - 6, fz + 6), A.C_SERVO)
    add("servo 2 bracket", box(lane(-1, 0, ARM_W)[1], fx - 6, FRONT_Y[0] - 6, FRONT_Y[0] + 2, fz - 6, fz + 6), A.C_PLATE)
    add("GT2 belt (servo 2 -> right rod)", belt_loop((fx, fz, PIN_R), (arm_xm(1), fz, PIN_R), 'xz', *FP_Y), C_BELT)
    # turn drive: a 16T pinion under a vertical N20 on a small lift carriage (SG90), on a mast above the column.  Lowered,
    # the pinion meshes the corner post's fixed sector; lifted (default) it passes over the sectors of other corners.
    px, py = TURN_PIN_XY
    zg0, zg1 = TURN_GEAR_Z
    top = zg0 + TURN_LIFT + 42
    add("printed turn-drive mast", box(-10, 10, COL_Y[0] + 2, COL_Y[1] - 2, hz + 20, top + 4)
        + box(-10, px + 8, py - 9, COL_Y[1] - 2, top, top + 4)
        + box(px - 14, px - 8, py - 9, py + 9, zg0 + 10, top), A.C_PRINT)
    add("SG90 micro servo (turn pinion lift)", box(px - 26, px - 14, py - 6, py + 17, top - 30, top - 8), A.C_SERVO)
    _RB = it
    return it


TURN_PIN_AT = (TURN_PIN_XY[0], TURN_PIN_XY[1], TURN_GEAR_Z[0])     # pinion lowered (in mesh)


@lru_cache(None)
def turn_carriage_local():
    """lift carriage of the turn drive (moves up/down with the pinion, does not turn), in the pinion frame"""
    h = TURN_GEAR_Z[1] - TURN_GEAR_Z[0]
    return [("N20 gear motor (turn drive, on the lift carriage)", box(-6, 6, -6, 6, h + 2, h + 11)
             + cyl(6, h + 11, h + 29, 'z', (0, 0), 20), A.C_GB),
            ("printed lift carriage", box(-8, -6, -8, 8, h + 2, h + 29), A.C_PRINT)]


@lru_cache(None)
def turn_pinion_local():
    """the robot's 16T turn pinion, axis z through (0, 0), face 0..8 mm"""
    return [("turn drive pinion 16T (robot)", P2.gear_part(TURN_PIN_T, TURN_GEAR_Z[1] - TURN_GEAR_Z[0]), A.C_PRINT2)]


@lru_cache(None)
def arm_mid_local():
    """middle bars (stowed) with their PTFE pads, stage-1 belt clamp, and the belt wrapped round the stage-2 pulleys"""
    it = []
    for sx in SIDES:
        xa, xb = lane(sx, 0, ARM_W)
        it.append(("telescopic arm, middle bar", box(xa, xb, ARM_Y0, TIP0, T_(MID_Z[0]), T_(MID_Z[1])), C_ARM))
        for u0, u1 in ((0, 2), (ARM_W - 2, ARM_W)):
            it.append(("PTFE slide pad", box(*lane(sx, u0, u1), ARM_Y0, TIP0, T_(OUT_Z[1]), T_(MID_Z[0])), C_PAD))
        y0, zt = ARM_Y0 + 2, S1_ZT + PIN_R
        clamp = box(*lane(sx, ARM_W, LANE_S1[1] + 0.5), y0, y0 + 10, T_(zt - 1.5), T_(zt + 2)) \
            + box(*lane(sx, ARM_W, ARM_W + 2), y0, y0 + 10, T_(zt + 2), T_(MID_Z[1]))
        it.append(("printed stage-1 belt clamp (middle bar)", clamp, A.C_PRINT))
        for k, L in ((0, LANE_EXT), (1, LANE_RET)):
            it.append(("stage-2 belt (round its pulley)", half_wrap(lane(sx, *L), DPUL_Y[k], T_(MID_C), DPUL_R, 1 if k == 0 else -1), C_BELT))
    return it


@lru_cache(None)
def dpulley_local(sx, k):
    L = LANE_EXT if k == 0 else LANE_RET
    w = L[1] - L[0]
    return [(f"stage-2 pulley ({'extend' if k == 0 else 'retract'})", pulley_x(DPUL_R, -w / 2, w), A.C_PRINT2)]


@lru_cache(None)
def arm_in_local():
    """inner bars (stowed) with PTFE pads, the stage-2 belt anchors and the finger-tube bearings"""
    it = []
    for sx in SIDES:
        xa, xb = lane(sx, 0, ARM_W)
        it.append(("telescopic arm, inner bar", box(xa, xb, ARM_Y0, TIP0, T_(IN_Z[0]), T_(IN_Z[1])), C_ARM))
        for u0, u1 in ((0, 2), (ARM_W - 2, ARM_W)):
            it.append(("PTFE slide pad", box(*lane(sx, u0, u1), ARM_Y0, TIP0, T_(MID_Z[1]), T_(IN_Z[0])), C_PAD))
        it.append(("belt anchor (inner bar)", box(*lane(sx, *LANE_EXT), ARM_Y0 + 2, ARM_Y0 + 6, T_(IN_Z[0] - 2.5), T_(IN_Z[0])), A.C_SW))
        it.append(("belt anchor (inner bar)", box(*lane(sx, *LANE_RET), TIP0 - 6, TIP0 - 2, T_(IN_Z[0] - 2.5), T_(IN_Z[0])), A.C_SW))
        for y0, y1 in ((TIP0 - 12, TIP0 - 6), (ARM_Y0, ARM_Y0 + 6)):
            it.append(("finger shaft bearing (inner bar)", box(xa, xb, y0, y1, T_(IN_Z[1]), T_(FS_ZT + FS_SQ[2] / 2 + 3)), A.C_PRINT))
    return it


def stage2_runs(arm):
    """the straight stage-2 belt runs, which change length: [(name, sx, k, (x0, x1, y0, y1, z0, z1))] robot frame.
    extend belt: outer-bar anchor -> under the extend pulley -> over it -> inner-bar anchor (rear ends);
    retract belt: the same from the front ends round the retract pulley.  Total length of each belt stays constant."""
    d = (arm - TIP0) / 2
    zl, zu = T_(MID_C - DPUL_R), T_(MID_C + DPUL_R)
    ya, yc = ARM_Y0 + 4, TIP0 - 4
    pe, pr = DPUL_Y[0] + d, DPUL_Y[1] + d
    out = []
    for sx in SIDES:
        for k, (L, nm, y0, y1, z) in enumerate(((LANE_EXT, "extend, lower", ya, pe, zl), (LANE_EXT, "extend, upper", ya + 2 * d, pe, zu),
                                                (LANE_RET, "retract, lower", pr, yc, zl), (LANE_RET, "retract, upper", pr, yc + 2 * d, zu))):
            xa, xb = lane(sx, *L)
            out.append((f"stage-2 belt ({nm})", sx, k, (xa, xb, min(y0, y1), max(y0, y1), z - BELT_T / 2, z + BELT_T / 2)))
    return out


# ------------------------------------------------------------------ poses
def state(cx, cz=CZ_HOME, h1=-90.0, h2=-90.0, rot=None, arm=TIP0, f=0.0, ry=0.0, cart=None, lift=None):
    """robot column at row-frame x = cx, at y = ry (a row's y, or the cart's while riding it).  cart: cart y (default: the
    robot's row).  lift: pinion lift (0 = in mesh with the transfer rack; default lifted unless the robot is on the cart
    away from a row)"""
    rot = rot or [[-90.0, -90.0] for _ in range(N_BAYS_ALL)]
    cart = ry if cart is None else cart
    lift = (0.0 if abs(cx - X_CART) < 1 and abs(ry - round(ry / ROW_P) * ROW_P) > 0.5 else TURN_LIFT) if lift is None else lift
    return dict(cx=cx, cz=cz, h1=h1, h2=h2, rot=rot, arm=arm, f=f, ry=ry, cart=cart, lift=lift)


def basket_M(b, l, y0=BASKET_Y0):
    """basket of (global) bay b, level l (near wall at row-frame y0)"""
    return row_M(bay_row(b)) @ MT(ROW_BAY_X[b % ROW_BAYS], y0, LEVELS[l])


def build(S, baskets=None):
    """S: state() dict.  baskets: (face, level) on a shelf, 'tray', or ('y', face, level, y0) = moving, near wall at y0"""
    items = [(n, s, c, "static") for n, s, c in static_items()]
    for j, rr in enumerate(S["rot"]):
        for k, a in enumerate(rr):
            items += [(n, s, c, "rotor") for n, s, c in crossing_items(j, k, a)]
    moving = abs(S["cart"] - round(S["cart"] / ROW_P) * ROW_P) > 0.5
    items += [(n, s.transform(MT(0, S["cart"], 0)), c, "cart" if moving else "rotor") for n, s, c in cart_local()]
    R = robot_world(S)
    g = mech_angles(S["arm"], S["f"])
    d = g["d"]
    place = lambda lst, M, grp: [(n, s.transform(M), c, grp) for n, s, c in lst]
    items += place(robot_local(), R, "robot")
    items += place(arm_drive_local(), R @ MT(*ARM_DRIVE_AT) @ rot('x', g["drive"]), "robot")
    items += place(idler_local(), R @ MT(*IDLER_AT) @ rot('x', g["drive"]), "robot")
    items += place(servo1_local(), R @ MT(*SERVO1_POS) @ rot('x', g["servo1"]), "robot")
    items += place(fservo_local(), R @ MT(*FSERVO_POS) @ rot('y', g["servo2"]), "robot")
    Mt = R @ MT(*TURN_PIN_AT) @ MT(0, 0, S.get("lift", TURN_LIFT))
    items += place(turn_carriage_local(), Mt, "robot")
    items += place(turn_pinion_local(), Mt @ rot('z', turn_pinion_angle(S["cart"])), "robot")
    Rm, Ri = R @ MT(0, d, 0), R @ MT(0, 2 * d, 0)
    items += place(arm_mid_local(), Rm, "arm_mid")
    items += place(arm_in_local(), Ri, "arm_in")
    for sx in SIDES:
        fa = g["finger_l" if sx < 0 else "finger_r"]
        items += place(fshaft_local(sx), R @ MT(*fshaft_at(sx)) @ rot('y', fa), "robot")
        items += place(ftube_mid_local(sx), Rm @ MT(*fshaft_at(sx)) @ rot('y', fa), "arm_mid")
        items += place(finger_local(sx), Ri @ MT(*fshaft_at(sx)) @ rot('y', fa), "finger")
        for k in (0, 1):
            items += place(dpulley_local(sx, k), Rm @ MT(*dpul_at(sx, k)) @ rot('x', g["dpul"]), "arm_mid")
    for n, sx, k, b in stage2_runs(S["arm"]):
        items.append((n, box(*b).transform(R), C_BELT, "belt"))
    for i, dz in ((1, D_HUB / 2), (2, -D_HUB / 2)):
        Mh = R @ MT(0, 0, dz) @ rot('y', S["h%d" % i])
        items += [(f"{n} (hub {i})", s.transform(Mh), c, f"hub{i}") for n, s, c in hub_local()]
        items += [(f"{n} (hub {i})", s.transform(Mh @ MT(*PINION_AT)), c, f"hub{i}") for n, s, c in hub_pinion_local()]
    bs = basket_solid()
    for k, b in enumerate(baskets or []):
        if b == 'tray':
            items.append(("basket (on tray)", bs.transform(R @ MT(BXR, BASKET_TRAY_Y0, T_V3)), A.BASKET_COLS[k % 10], "robot"))
        elif b[0] == 'y':
            items.append(("basket (moving)", bs.transform(basket_M(b[1], b[2], b[3])), A.BASKET_COLS[k % 10], "robot"))
        else:
            items.append((f"basket bay{b[0]+1}-L{b[1]+1}", bs.transform(basket_M(*b)), A.BASKET_COLS[k % 10], "static"))
    return items


def pose(name):
    """named poses for the CAD export"""
    B_all = [(b, l) for b in range(N_BAYS_ALL) for l in range(len(LEVELS))]
    others = [x for x in B_all if x != (0, 2)]
    vert = [[0.0, 0.0]] + [[-90.0, -90.0]] * (N_BAYS_ALL - 1)
    if name == "pick":       # row 1 bay 1, shelf 3, arms out, both fingers down around the basket
        return build(state(bay_rail_x(0), cz_for_level(2), 0, 0, vert, TIP_SHELF, 90), B_all)
    if name == "swivel":     # at bay 1's crossings: lower hub already vertical, upper hub swivelling
        return build(state(bay_rail_x(0), CZ_HOME, -45, 0, [[0.0, -45.0]] + vert[1:]), others + ['tray'])
    if name == "travel":     # along row 1 on the two horizontal rails
        return build(state(bay_rail_x(1) + 200), others + ['tray'])
    if name == "transfer":   # on the cart, half way from row 1 to row 2, pinion in the transfer rack
        return build(state(X_CART, ry=ROW_P / 2), others + ['tray'])
    raise ValueError(name)


POSES = ("pick", "swivel", "travel", "transfer")
