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


# ------------------------------------------------------------------ printable racks: whole-tooth segments
RACK_SEG_LEN = RACK_SEG_TEETH * P                     # P = rack pitch (4.712 mm)


def rack_pieces(L):
    """[(offset, length)] pieces of a rack of length L: full pieces of RACK_SEG_TEETH teeth (cut through a tooth space, so the
    pitch continues across the joint), the remainder last"""
    out, off = [], 0.0
    while off < L - 0.01:
        sl = min(RACK_SEG_LEN, L - off)
        out.append((off, sl))
        off += sl
    return out


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


# ------------------------------------------------------------------ docking station: motorized scissor jack + raised floor
# Dock frame: origin at (PXC, row y = 0, z = 0): x along the row, y into the rack, z up.  Every part is named "jack: ...".
def jack_angles(zt):
    """scissor arm angle (rad) and the slider's distance from the fixed pivot, for a platform top at height zt"""
    H = zt - PLATE_T - PIN_DROP - JACK_ZB                 # pin height difference (bottom pins -> top pins)
    phi = math.asin(H / JACK_L)
    return phi, JACK_L * math.cos(phi)


def jack_screw_angle(zt):
    """the nut moves JACK_LEAD per screw turn"""
    return 360.0 * (jack_angles(zt)[1] - jack_angles(JACK_DOWN)[1]) / JACK_LEAD


def dock_basket_M(r, zt, y0):
    """a basket on dock r's platform (top at zt) with its near wall at row-frame y0"""
    return row_M(r) @ MT(PXC, y0, zt + BASKET_CLR)


def _bar(x0, x1, y0, y1, half=7.0):
    """flat scissor arm: bar along x with rounded ends, centred on z = 0"""
    return Solid(box(x0, x1, y0, y1, -half, half).polys + cyl(half, y0, y1, 'y', (x0, 0), 20).polys
                 + cyl(half, y0, y1, 'y', (x1, 0), 20).polys)


@lru_cache(None)
def jack_static_local():
    """fixed parts of the jack, dock frame: base plate, guide rails, bottom pivot lugs + shaft, bearing block, limit switches,
    and the NEMA17 on the floor beside the platform"""
    it = []
    add = lambda n, s, c: it.append((n, s, c))
    add("jack: base plate (aluminium)", box(-135, 135, 8, 222, 0, JACK_BASE_T), A.C_PLATE)
    for ys in JACK_YS:
        add("jack: slider guide rail", box(-34, 84, ys + 10, ys + 16, JACK_BASE_T, JACK_RAIL_TOP), A.C_STEEL)
        add("jack: bottom pivot lug", box(-JACK_X0 - 8, -JACK_X0 + 8, ys - 18, ys - 10, JACK_BASE_T, JACK_ZB + 8), A.C_PRINT)
    add("jack: bottom pivot shaft (fixed)", cyl(3, JACK_YS[0] - 16, JACK_YS[1] + 1, 'y', (-JACK_X0, JACK_ZB), 14), A.C_STEEL)
    add("jack: screw bearing block (printed)", box(SCREW_X - 8, SCREW_X, JACK_YC - 12, JACK_YC + 12, JACK_BASE_T, JACK_ZS + 11), A.C_PRINT)
    # limit switches at both ends of the nut's travel (nut block +-8 about x = -13 raised ... +69.8 lowered)
    add("jack: limit switch (platform up)", box(-30, -22, JACK_YC + 12, JACK_YC + 22, JACK_BASE_T, 20), A.C_SW)
    add("jack: limit switch (platform down)", box(78, 86, JACK_YC + 12, JACK_YC + 22, JACK_BASE_T, 20), A.C_SW)
    xm = SCREW_X + SCREW_L + 8                                            # motor front face (its shaft end is at x = 140 + 8)
    add("jack: NEMA17 stepper + 3:1 planetary gearhead", box(xm, xm + MOTOR_L, JACK_YC - 21, JACK_YC + 21, JACK_ZS - 21, JACK_ZS + 21)
        + cyl(11, xm - 6, xm, 'x', (JACK_YC, JACK_ZS), 20), A.C_GB)
    br = box(xm - 6, xm, JACK_YC - 33, JACK_YC + 33, 0, 56.0)
    for yy in (JACK_YC - 27.5, JACK_YC + 27.5):                      # holes for the motor cover (nut pockets on the back face)
        for zz in (6.0, 50.0):
            br = br - cyl(M3_CLR / 2, xm - 7, xm + 1, 'x', (yy, zz), 14) - hexagon(M3_NUT_AF, xm - 7, xm - 6 + M3_NUT_T, (yy, zz), 'x')
    add("jack: motor bracket (printed)", br, A.C_PRINT)
    return it


@lru_cache(None)
def jack_platform_local():
    """platform body, origin at the platform's top centre (z 0 = top face); y is the row-frame y"""
    it = []
    add = lambda n, s, c: it.append((n, s, c))
    add("jack: platform plate (8 mm aluminium)", box(-DOCK_HALF, DOCK_HALF, DOCK_Y[0], DOCK_Y[1], -PLATE_T, 0)
        + box(-DOCK_HALF, DOCK_HALF, DOCK_Y[1], DOCK_Y[1] + 4, -PLATE_T, 9), A.C_PLATE)           # + back stop
    zp = -PLATE_T - PIN_DROP                                                 # pin height under the plate
    for ys in JACK_YS:
        add("jack: roller guide bar", box(-24, 80, ys - 16, ys - 12, zp - 8, -PLATE_T), A.C_STEEL)
        add("jack: top pivot lug", box(-JACK_X0 - 8, -JACK_X0 + 8, ys + 10, ys + 16, zp - 8, -PLATE_T), A.C_PRINT)
    add("jack: top pivot shaft", cyl(3, JACK_YS[0] + 1, JACK_YS[1] + 14, 'y', (-JACK_X0, zp), 14), A.C_STEEL)
    return it


@lru_cache(None)
def jack_arm1_local():
    """both outer scissor arms (bottom-left fixed pivot -> top-right roller), origin at the fixed pivot, along +x;
    with the centre shaft and the roller axle"""
    it = []
    for ys in JACK_YS:
        it.append(("jack: scissor arm (outer)", _bar(0, JACK_L, ys - 9, ys - 1), C_ARM))
        it.append(("jack: roller", cyl(6, ys - 12, ys - 9, 'y', (JACK_L, 0), 20), A.C_WHEEL))
    it.append(("jack: centre pin shaft", cyl(3, JACK_YS[0] - 9, JACK_YS[1] + 9, 'y', (JACK_L / 2, 0), 14), A.C_STEEL))
    it.append(("jack: roller axle", cyl(2, JACK_YS[0] - 16, JACK_YS[1] - 1, 'y', (JACK_L, 0), 12), A.C_STEEL))
    return it


@lru_cache(None)
def jack_arm2_local():
    """both inner scissor arms (top-left platform pivot -> bottom-right slider), origin at the top pivot, along +x"""
    it = []
    for ys in JACK_YS:
        it.append(("jack: scissor arm (inner)", _bar(0, JACK_L, ys + 1, ys + 9), C_ARM))
    return it


@lru_cache(None)
def jack_slider_local():
    """slider: cross tube (the bottom-right pins) + a nut block clamping the tube and the lead nut + shoes on the guide rails;
    origin at the cross tube axis"""
    it = [("jack: slider cross tube", cyl(4, JACK_YS[0] + 1, JACK_YS[1] + 16, 'y', (0, 0), 16), A.C_STEEL),
          ("jack: lead nut block (printed)", box(-8, 8, JACK_YC - 8, JACK_YC + 8, -4, JACK_ZS - JACK_ZB + 8), A.C_PRINT)]
    for ys in JACK_YS:
        it.append(("jack: slider shoe (printed)", box(-12, 12, ys + 10, ys + 16, JACK_RAIL_TOP - JACK_ZB, 7), A.C_PRINT))
    return it


@lru_cache(None)
def jack_screw_local():
    """TR8x2 lead screw + coupling with a visible index flag, origin on its axis (x) at the left end, turns about x"""
    L = SCREW_L
    shaft = cyl(4, 0, L, 'x', (0, 0), 16)
    rings = [cyl(4.8, x, x + 1.2, 'x', (0, 0), 16) for x in np.arange(6.0, 112.0, 5.0)]     # thread crests (so the turn shows)
    flag = box(L - 22, L - 12, -3, 3, -17, 17) + cyl(8, L - 22, L - 12, 'x', (0, 0), 20)
    return [("jack: TR8x2 lead screw", Solid(shaft.polys + sum([r_.polys for r_ in rings], [])), A.C_STEEL),
            ("jack: shaft coupling + index flag", flag, A.C_PRINT2)]




@lru_cache(None)
def deck_local():
    """the raised floor (rectangle, top at JACK_DOWN) round the lowered platform and the motor; dock frame (x relative to PXC)"""
    x0, x1 = DECK_X[0] - PXC, DECK_X[1] - PXC
    y0, y1 = DECK_Y
    hx0, hx1 = -DOCK_HALF - 3.0, SCREW_X + SCREW_L + 8 + MOTOR_L + 6.0   # hole: platform (3 mm clearance) and the motor
    hy1 = DOCK_Y[1] + 4 + 3.0
    z1 = JACK_DOWN
    z0 = z1 - 18.0
    return [("raised floor: top plate (18 mm plywood)", union_all([box(x0, hx0, y0, y1, z0, z1), box(hx1, x1, y0, y1, z0, z1),
                                                                  box(hx0, hx1, hy1, y1, z0, z1)]), A.C_WOOD),
            ("raised floor: battens", union_all([box(x0, hx0, y0, y1, 0, z0), box(hx1, x1, y0, y1, 0, z0),
                                                 box(hx0, hx1, y1 - 24, y1, 0, z0)]), A.C_PRINT)]


def dock_static_local():
    return jack_static_local() + deck_local()


# ------------------------------------------------------------------ covers (printed shells, screwed on)
FRONT_COVER_SCREW_X = (156.6, 204.4)      # robot frame x of the two chipboard screws into the tray's front edge


@lru_cache(None)
def robot_covers_local():
    """covers on the robot, robot frame: (1) front drive (servo 1 + step-up belt + cross-shaft end), (2) turn-drive cap"""
    w = COVER_WALL
    out = []
    # (1) front drive cover: open at the back (towards the tray) and the bottom; ears on the tray's front face, screwed into the
    #     plywood edge with 3 x 16 chipboard screws
    ix0, ix1, iy0, iz1 = 164.0, 197.0, -347.0, 9.6
    shell = box(ix0 - w, ix1 + w, iy0 - w, -294.0, -46.4, iz1 + w) - box(ix0, ix1, iy0, -293.0, -47.0, iz1)    # lower edge 2.2 mm under the pulley: keeps >3 mm over a lowered basket
    shell = shell - cyl(5.0, ix0 - w - 1, ix1 + w + 1, 'x', (-303.0, 0.0), 24)               # the cross shaft passes through both sides
    for xl, xr in ((ix0 - w - 10.0, ix0 - w + 1.5), (ix1 + w - 1.5, ix1 + w + 10.0)):         # ears
        shell = shell + box(xl, xr, -297.0, -294.0, -34.0, -22.0)
    for xh in FRONT_COVER_SCREW_X:
        shell = shell - cyl(M3_CLR / 2, -298.0, -293.0, 'y', (xh, -28.0), 16)
    out.append(("cover: front drive (servo 1 + belt)", shell, COVER_COLOR))
    # (2) turn-drive cap: over the pinion motor, the lift servo and the carriage; sits on the two mast bosses (M3 screw from the top)
    px, py = TURN_PIN_XY
    top = TURN_GEAR_Z[0] + TURN_LIFT + 42
    ix0, ix1, iy0, iy1, iz0, iz1 = 30.0, 72.0, -113.0, -82.0, 204.0, top + 12
    cap = box(ix0 - w, ix1 + w, iy0 - w, iy1 + w, iz0, iz1 + w) - box(ix0, ix1, iy0, iy1, iz0 - 1, iz1)
    cap = cap - box(ix0 - w - 1, ix0 + 1, -112.5, -97.0, top - 1, top + 5)                     # the mast's top arm passes through the left wall
    for xb in MAST_BOSS_X:
        cap = cap - cyl(M3_CLR / 2, iz1 - 1, iz1 + w + 1, 'z', (xb, MAST_BOSS_Y), 14)
    out.append(("cover: turn-drive cap", cap, COVER_COLOR))
    return out


@lru_cache(None)
def dock_cover_local():
    """cover of the dock's motor + gearhead, dock frame: stands on the floor, flange against the (enlarged) motor bracket, 4 M3 screws"""
    w = COVER_WALL
    xm = SCREW_X + SCREW_L + 8
    ix0, ix1, iy0, iy1, iz1 = xm, xm + MOTOR_L + 2.0, JACK_YC - 23.0, JACK_YC + 23.0, 46.0
    shell = box(ix0, ix1 + w, iy0 - w, iy1 + w, 0, iz1 + w) - box(ix0 - 1, ix1, iy0, iy1, -1, iz1)
    flange = box(xm, xm + 3.0, JACK_YC - 33, JACK_YC + 33, 0, 56.0) - box(xm - 1, xm + 4.0, iy0, iy1, -1, iz1)
    cov = shell + flange
    for yy in (JACK_YC - 27.5, JACK_YC + 27.5):
        for zz in (6.0, 50.0):
            cov = cov - cyl(M3_CLR / 2, xm - 1, xm + 4.0, 'x', (yy, zz), 14)
    return [("cover: jack motor", cov, COVER_COLOR)]


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
        for off, sl in rack_pieces(RAIL_TOP_V3 - z0):
            add("printed in-slot rack", P2.rack_strip(0, sl).translate(jx, 0, z0 + off), A.C_PRINT)
        add("V-slot rail (vertical, between crossings)", A.vslot_z(JZ_LO + g, JZ_UP - g).translate(jx), A.C_RAIL)
        for off, sl in rack_pieces(JZ_UP - JZ_LO - 2 * g):
            add("printed in-slot rack", P2.rack_strip(0, sl).translate(jx, 0, JZ_LO + g + off), A.C_PRINT)
        for zb in (RAIL_TOP - 45, 650.0):
            add("printed rail bridge", P2.rail_bridge().translate(jx, 0, zb), A.C_PRINT)
        for zc in CROSS_Z:      # small printed bracket from the bearing to the next rack post
            add("printed crossing bracket", box(jx - 18, jx + 60, -31, -22, zc - 15, zc + 15), A.C_PRINT)
    cuts = [ROW_RAIL_X[0]] + [v for bx in ROW_BAY_X for v in (bx - RAIL_OFF - g, bx - RAIL_OFF + g)] + [ROW_RAIL_X[1]]
    for x1, x2 in zip(cuts[0::2], cuts[1::2]):
        if x2 - x1 < 20.0:
            continue                       # no stubs: the rail simply starts at the first crossing piece
        for zc in CROSS_Z:
            add("V-slot rail (horizontal)", A.vslot_z(0, x2 - x1).rotate('y', 90).translate(x1, 0, zc), A.C_RAIL)
            for off, sl in rack_pieces(x2 - x1):
                add("printed in-slot rack", P2.rack_strip(0, sl).rotate('y', 90).translate(x1 + off, 0, zc), A.C_PRINT)
    # the rails run on ~120 mm past the last rack to meet the cart: short arms from the rack's end post carry their ends,
    # and a third arm carries the socket for the cart's lock pin (nothing stands on the floor here)
    ye0, ye1 = RAIL_Y + 10, RAIL_Y + 40
    for zc in CROSS_Z:
        add("rail end arm (from the rack end post)", box(RACK_END_X, ROW_RAIL_X[1], ye0, ye1, zc - 12, zc + 12), A.C_RAIL)
    sx0, sx1, yl = CART_PIECE[0] - 34, CART_PIECE[0] - 4, RAIL_Y + 25.0           # socket block, its +x face, pin axis y
    sock = box(sx0, sx1, yl - 13, yl + 13, JZ_LO + 6, LOCK_Z + 13) \
        - cyl(PIN_HOLE_R, sx1 - 12, sx1 + 1, 'x', (yl, LOCK_Z), 24, r1=PIN_MOUTH_R) - cyl(PIN_HOLE_R, sx0 - 1, sx1 - 11, 'x', (yl, LOCK_Z), 20)
    add("cart lock socket (printed, funnel lead-in, on the lower rail-end arm)", sock, A.C_JUNC)
    return it


def xfer_rack_pieces():
    """fixed straight gear rack (module 1.5) along the cart track, teeth facing -x toward the robot's pinion, in printable
    pieces of RACK_SEG_TEETH whole teeth (the pitch carries across the joints)"""
    p, m = math.pi * 1.5, 1.5
    x0 = XFER_RACK_X
    y0, y1 = TRACK_Y
    zg0, zg1 = CZ_HOME + TURN_GEAR_Z[0], CZ_HOME + TURN_GEAR_Z[1]
    out, ys = [], y0
    while ys < y1 - 0.01:
        n = min(RACK_SEG_TEETH, int((y1 - ys) // p))
        if n < 1:
            break
        pts = [(x0 + 1.25 * m + 4, ys)]
        for k in range(n):
            y = ys + k * p
            pts += [(x0 + 1.25 * m, y), (x0 - m, y + p * 0.25), (x0 - m, y + p * 0.5), (x0 + 1.25 * m, y + p * 0.75)]
        ye = ys + n * p
        pts += [(x0 + 1.25 * m, ye), (x0 + 1.25 * m + 4, ye)]
        out.append(extrude(pts, zg0, zg1))
        ys = ye
    return out


def xfer_static():
    """the side bar: the transfer beam (carries the hanging cart and the gear rack) on posts with levelling feet beyond
    the robot's tray, and a floor guide rail that only keeps the cart's bottom from swinging"""
    y0, y1 = TRACK_Y
    xb = XFER_RACK_X + 1.25 * 1.5 + 4
    it = [("floor guide rail (guides only, no load)", box(X_CART - 10, X_CART + 10, y0, y1, 0, 20), A.C_RAIL),
          ] + [("printed transfer rack segment (module 1.5)", pc, A.C_PRINT) for pc in xfer_rack_pieces()] + [
          ("transfer beam (20x40 V-slot, carries the cart)", box(xb, xb + 20, y0, y1, BEAM_Z[0], BEAM_Z[1]), A.C_RAIL)]
    for y in [y0 + 20] + [r * ROW_P + RACK_D / 2 for r in range(N_ROWS)] + [y1 - 20]:   # posts in the racks' depth band
        it += [("transfer beam arm", box(xb + 20, XFER_POST_X, y - 10, y + 10, BEAM_Z[0] + 8, BEAM_Z[1] - 8), A.C_RAIL),
               ("transfer beam post (levelling foot)", box(XFER_POST_X, XFER_POST_X + 20, y - 10, y + 10, 0, BEAM_Z[1] - 8)
                + box(XFER_POST_X - 40, XFER_POST_X + 60, y - 30, y + 30, 0, 6), A.C_RAIL)]
    return it


def static_items():
    global _S
    if _S is None:
        loc = row_static_local()
        _S = [(f"{n} (row {r+1})", s.transform(row_M(r)), c) for r in range(N_ROWS) for n, s, c in loc]
        _S += xfer_static()
        for r in range(N_ROWS):
            _S += [(f"{n} (dock {r+1})", s.transform(row_M(r) @ MT(PXC, 0, 0)), c) for n, s, c in dock_static_local()]
    return _S


_CA = None


def cart_local():
    """transfer cart at y = 0 (lined up with row 1), HANGING from the transfer beam: two trolleys (V-wheels above and below
    the beam, ~230 mm apart) carry it, so it cannot lean or tip; a frame behind the robot's rails holds the two short
    rail pieces; guide rollers on the floor rail stop the bottom swinging (no load).  No motor."""
    global _CA
    if _CA is None:
        x0, x1 = CART_PIECE
        xb = XFER_RACK_X + 1.25 * 1.5 + 4                # beam -x face
        xt1 = XFER_RACK_X - 1.5 - 1.0                    # trolley side plate: between the robot (x <= C+68) and the
        xt0 = xt1 - 7.5                                  # rack teeth (from XFER_RACK_X - 1.5); axles bridge over the teeth
        ya, yz = -40.0, CART_TROLLEY_Y[1] + 20           # frame extent in y (the robot stays at y < -40)
        it = []
        for zc in CROSS_Z:
            it += [("rail piece on cart", A.vslot_z(0, x1 - x0).rotate('y', 90).translate(x0, 0, zc), A.C_RAIL),
                   ("printed in-slot rack (cart)", P2.rack_strip(0, x1 - x0).rotate('y', 90).translate(x0, 0, zc), A.C_PRINT)]
        top = BEAM_Z[1] + 24
        frame = [box(x0, x0 + 20, ya, ya + 20, 22, top), box(x1 - 20, x1, ya, ya + 20, 22, top),      # posts behind the pieces
                 box(x0, xt1, ya, ya + 20, top - 12, top),                                            # top cross member
                 box(x0, x1, ya, ya + 20, 22, 40),                                                    # bottom cross member
                 box(xt0 - 8, xt0, ya, yz, top - 12, top),                                            # top member along y
                 box(X_CART - 10, X_CART + 10, ya, yz, 22, 40),                                       # bottom member along y
                 box(X_CART - 10, X_CART + 10, yz - 20, yz, 22, top), box(X_CART - 10, xt0, yz - 20, yz, top - 12, top)]
        it.append(("cart frame (20x20 V-slot)", union_all(frame), A.C_JUNC))
        for yc in CART_TROLLEY_Y:
            plate = box(xt0, xt1, yc - 22, yc + 22, BEAM_Z[0] - 24, top)
            wheels = [cyl(10, xb + 1, xb + 19, 'x', (yc + d, z), 20) for d in (-12, 12) for z in (BEAM_Z[1] + 10, BEAM_Z[0] - 10)]
            wheels += [cyl(3, xt1, xb + 20, 'x', (yc + d, z), 12) for d in (-12, 12) for z in (BEAM_Z[1] + 10, BEAM_Z[0] - 10)]
            it += [("cart trolley plate", plate, A.C_JUNC), ("trolley V-wheels (above + below the beam)", union_all(wheels), A.C_WHEEL)]
        for yc in (-20.0, CART_TROLLEY_Y[1]):
            it.append(("floor guide rollers (sideways only)", box(X_CART - 18, X_CART - 11, yc - 8, yc + 8, 2, 22)
                       + box(X_CART + 11, X_CART + 18, yc - 8, yc + 8, 2, 22), A.C_WHEEL))
        zbolt = CZ_HOME + BOLT_DZ
        bar = box(X_CART - 58, X_CART + 58, BAR_Y[0], BAR_Y[1], BAR_Z[0], BAR_Z[1]) - cyl(BOLT_R + 1.0, BAR_Y[0] - 1, BAR_Y[0] + 10, 'y', (X_CART + BOLT_X, zbolt), 24, r1=BOLT_MOUTH_R) \
            - cyl(BOLT_R + 1.0, BAR_Y[0] + 9, BAR_Y[0] + 15, 'y', (X_CART + BOLT_X, zbolt), 20)
        it.append(("cart docking bar (printed, with the bolt socket)", bar, A.C_PRINT))
        _CA = it
    return _CA


def cart_lock_local():
    """spring pin of the cart (slides along x): engaged (dx 0) its tip sits in the row's socket; the robot's release plunger
    pushes the tab (dx = LOCK_STROKE) and the pin leaves the socket while the cart moves.  Absolute row-frame x."""
    x0 = CART_PIECE[0]
    yl = RAIL_Y + 25.0
    pin = cyl(PIN_R / 2, x0 - 18, x0 - 9, 'x', (yl, LOCK_Z), 16, r1=PIN_R) + cyl(PIN_R, x0 - 9, x0 + 14, 'x', (yl, LOCK_Z), 20)   # bullet nose
    tab = box(X_CART + TAB_X[0], X_CART + TAB_X[1], -92, yl + 7, LOCK_Z - 5, LOCK_Z + 5) \
        + box(X_CART + TAB_X[0], X_CART + TAB_X[1], -92, -78, TAB_Z[0], TAB_Z[1])
    return [("cart lock pin + release tab (spring)", pin + tab, A.C_STEEL)]


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
    s = s - cyl(2.1, x0 - 1, x0 + w + 1, 'x', (0, 0), 16)                        # 4 mm axle bore (0.2 mm clearance)
    for k in range(3):                                                           # index holes (so the turning shows)
        a = 2 * math.pi * k / 3
        s = s - cyl(0.9, x0 - 1, x0 + w + 1, 'x', (3.9 * math.cos(a), 3.9 * math.sin(a)), 10)
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
        f = box(-w, w, y1 - FINGER_T, y1, 0, FINGER_L) + cyl(FINGER_COLLAR_R, y1 - FINGER_T, y1, 'y', (0, 0), 28)   # round collar
        f = f - box(-(h + 0.1), h + 0.1, y1 - FINGER_T - 1, y1 + 1, -(h + 0.1), h + 0.1)                       # square hole for the tube
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
    add("aluminium column bar 28 x 24 (replaces the printed column)", box(-14, 14, COL_Y[0], COL_Y[1], -hz - 20, hz + 20), A.C_PLATE)
    for z in (hz, -hz):
        add("JGY-370 swivel motor", box(-11.25, 11.25, COL_Y[0] - 22.5, COL_Y[0], z - 23, z + 23), A.C_GB)
        add("JGY-370 swivel motor can", cyl(12.2, 11.25, 44.25, 'x', (COL_Y[0] - 11.25, z), 24), A.C_CAN)
    add("battery + electronics box (fixed counterweight)", box(BAT_X[0], BAT_X[1], BAT_Y[0], BAT_Y[1], BAT_Z[0], BAT_Z[1]), C_BAT)
    add("battery bracket", box(BAT_X[1], -14, COL_Y[0], COL_Y[1], -15, 15), A.C_PLATE)
    T, x = T_V3, BXR
    add("tray bracket", box(14, x - ARM_X[1] - 2, COL_Y[0], COL_Y[1], T - TRAY_T, T), A.C_PLATE)
    add("bar load cell 5 kg + HX711", box(40, 76, COL_Y[0] + 2, COL_Y[1] - 2, T - TRAY_T - 13, T - TRAY_T), A.C_PCB)
    ply = box(x - ARM_X[1], x + ARM_X[1], TRAY_Y[0], TRAY_Y[1], T - TRAY_T, T)
    for xh in FRONT_COVER_SCREW_X:                                  # 2.5 mm pilot holes for the cover's chipboard screws
        ply = ply - cyl(1.25, TRAY_Y[0] - 1, TRAY_Y[0] + 15, 'y', (xh, T - TRAY_T / 2), 12)
    add("plywood tray 12 mm", ply, A.C_WOOD)
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
    mast = box(-10, 10, COL_Y[0] + 2, COL_Y[1] - 2, hz + 20, top + 4) \
        + box(-10, px + 8, py - 9, COL_Y[1] - 2, top, top + 4) \
        + box(px - 14, px - 8, py - 9, py + 9, zg0 + 10, top)
    for xb in MAST_BOSS_X:                                           # bosses for the cap: M3 screw from the top into a nut in a side slot
        mast = mast + cyl(5.0, top + 3, top + 12, 'z', (xb, MAST_BOSS_Y), 24)
        mast = mast - cyl(M3_CLR / 2, top - 1, top + 13, 'z', (xb, MAST_BOSS_Y), 14)
        mast = mast - box(xb - M3_NUT_AF / 2, xb + M3_NUT_AF / 2, MAST_BOSS_Y, MAST_BOSS_Y + 8.0, top + 6.0, top + 6.0 + M3_NUT_T + 0.2)
    add("printed turn-drive mast", mast, A.C_PRINT)
    add("SG90 micro servo (turn pinion lift)", box(px - 26, px - 14, py - 6, py + 17, top - 30, top - 8), A.C_SERVO)
    # lock solenoids (push type): docking bolt on the column's rear face, release plunger behind the battery box
    add("lock solenoid (docking bolt, push type)", box(BOLT_X - 9, BOLT_X + 9, -96, -68, BOLT_DZ - 8, BOLT_DZ + 8), A.C_GB)
    add("lock solenoid bracket (docking bolt)", box(13, BOLT_X + 12, -99, -93, BOLT_DZ - 12, BOLT_DZ + 12), A.C_PRINT)
    add("lock solenoid (release plunger, push type)", box(-100, -72, -99, -78, REL_DZ - 8, REL_DZ + 8), A.C_GB)
    add("lock solenoid bracket (release plunger)", box(-104, -68, -100, -94, REL_DZ - 12, REL_DZ + 12), A.C_PRINT)
    _RB = it
    return it


@lru_cache(None)
def dock_bolt_local():
    """docking bolt, robot frame, along y: tip at BOLT_REST_Y retracted, +BOLT_STROKE extended (into the cart's socket)"""
    rod = cyl(BOLT_R, -84.0, BOLT_REST_Y - 6, 'y', (BOLT_X, BOLT_DZ), 14) + cyl(BOLT_R, BOLT_REST_Y - 6, BOLT_REST_Y, 'y', (BOLT_X, BOLT_DZ), 14, r1=2.0)
    return [("lock solenoid bolt (docking, bullet nose)", rod, A.C_STEEL)]


@lru_cache(None)
def release_plunger_local():
    """release plunger, robot frame, along x at y -85: tip at PLUNGER_REST_X retracted (2 mm from the pin's tab), pushes +x"""
    return [("lock solenoid plunger (cart release)", box(-94, PLUNGER_REST_X, -87, -83, REL_DZ - 2, REL_DZ + 2), A.C_STEEL)]


TURN_PIN_AT = (TURN_PIN_XY[0], TURN_PIN_XY[1], TURN_GEAR_Z[0])     # pinion lowered (in mesh)


@lru_cache(None)
def turn_carriage_local():
    """lift carriage of the turn drive (moves up/down with the pinion, does not turn), in the pinion frame"""
    h = TURN_GEAR_Z[1] - TURN_GEAR_Z[0]
    return [("N20 gear motor (turn drive, on the lift carriage)", box(-6, 6, -6, 6, h + 2, h + 11)
             + cyl(6, h + 11, h + 29, 'z', (0, 0), 20), A.C_GB),
            ("printed lift carriage", box(-10, -6, -8, 8, h + 2, h + 29), A.C_PRINT)]


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
def state(cx, cz=CZ_HOME, h1=-90.0, h2=-90.0, rot=None, arm=TIP0, f=0.0, ry=0.0, cart=None, lift=None, jack=None,
          lock=None, bolt=None):
    """robot column at row-frame x = cx, at y = ry (a row's y, or the cart's while riding it).  cart: cart y (default: the
    robot's row).  lift: pinion lift (0 = in mesh with the transfer rack; default lifted unless the robot is on the cart
    away from a row)"""
    rot = rot or [[-90.0, -90.0] for _ in range(N_BAYS_ALL)]
    cart = ry if cart is None else cart
    lift = (0.0 if abs(cx - X_CART) < 1 and abs(ry - round(ry / ROW_P) * ROW_P) > 0.5 else TURN_LIFT) if lift is None else lift
    cart_moving = abs(cart - round(cart / ROW_P) * ROW_P) > 0.5
    lock = (LOCK_STROKE if cart_moving else 0.0) if lock is None else lock                 # pin out of the row's socket
    bolt = (BOLT_STROKE if (cart_moving and abs(cx - X_CART) < 1) else 0.0) if bolt is None else bolt   # robot locked to the cart
    return dict(cx=cx, cz=cz, h1=h1, h2=h2, rot=rot, arm=arm, f=f, ry=ry, cart=cart, lift=lift, lock=lock, bolt=bolt,
                jack=list(jack) if jack is not None else [JACK_DOWN] * N_ROWS)


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
    items += [(n, s.transform(MT(S["lock"], S["cart"], 0)), c, "cart" if moving else "rotor")
              for n, s, c in cart_lock_local()]
    R = robot_world(S)
    g = mech_angles(S["arm"], S["f"])
    d = g["d"]
    place = lambda lst, M, grp: [(n, s.transform(M), c, grp) for n, s, c in lst]
    items += place(robot_local(), R, "robot")
    items += place(robot_covers_local(), R, "cover")                      # printed covers: toggled in the sim, screwed on in reality
    for r in range(N_ROWS):
        items += [(f"{n} (dock {r+1})", s.transform(row_M(r) @ MT(PXC, 0, 0)), c, "static") for n, s, c in dock_cover_local()]
    items += place(arm_drive_local(), R @ MT(*ARM_DRIVE_AT) @ rot('x', g["drive"]), "robot")
    items += place(idler_local(), R @ MT(*IDLER_AT) @ rot('x', g["drive"]), "robot")
    items += place(servo1_local(), R @ MT(*SERVO1_POS) @ rot('x', g["servo1"]), "robot")
    items += place(fservo_local(), R @ MT(*FSERVO_POS) @ rot('y', g["servo2"]), "robot")
    Mt = R @ MT(*TURN_PIN_AT) @ MT(0, 0, S.get("lift", TURN_LIFT))
    items += place(turn_carriage_local(), Mt, "robot")
    items += place(dock_bolt_local(), R @ MT(0, S["bolt"], 0), "robot")
    items += place(release_plunger_local(), R @ MT(S["lock"], 0, 0), "robot")
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
    for r in range(N_ROWS):                       # the docks' moving parts: platform, scissor arms, slider, lead screw
        zt = S["jack"][r]
        phi, sx = jack_angles(zt)
        D, g = row_M(r) @ MT(PXC, 0, 0), f"jack{r}"
        items += place(jack_platform_local(), D @ MT(0, 0, zt), g)
        items += place(jack_arm1_local(), D @ MT(-JACK_X0, 0, JACK_ZB) @ rot('y', -math.degrees(phi)), g)
        items += place(jack_arm2_local(), D @ MT(-JACK_X0, 0, zt - PLATE_T - PIN_DROP) @ rot('y', math.degrees(phi)), g)
        items += place(jack_slider_local(), D @ MT(-JACK_X0 + sx, 0, JACK_ZB), g)
        items += place(jack_screw_local(), D @ MT(SCREW_X, JACK_YC, JACK_ZS) @ rot('x', jack_screw_angle(zt)), g)
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
        elif b[0] == 'dock':         # ('dock', row, platform top, near-wall y): being pushed / pulled on a dock platform
            items.append(("basket (moving)", bs.transform(dock_basket_M(b[1], b[2], b[3])), A.BASKET_COLS[k % 10], "robot"))
        elif b[0] == 'dockrest':     # ('dockrest', row, platform top): resting on the platform
            items.append((f"basket on dock {b[1]+1}", bs.transform(dock_basket_M(b[1], b[2], BASKET_Y0)), A.BASKET_COLS[k % 10], "static"))
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
    if name == "dock":       # at dock 1: the basket is on the raised platform (rear fingers just pushed it), jack up
        return build(state(DOCK_X_C, arm=TIP_PUSHED, f=90, jack=[JACK_UP, JACK_DOWN, JACK_DOWN]),
                     B_all + [('dock', 0, JACK_UP, BASKET_Y0)])
    raise ValueError(name)


POSES = ("pick", "swivel", "travel", "transfer", "dock")
