"""Printable parts.  Rail-side parts are drawn for the RIGHT rail in a local frame:
   u = x - XR (positive = outward, away from the rack centre), y = global y, w = z - reference.
Left-side parts are mirror images (mirror in x)."""
import math, numpy as np
from csg import *
from params import *

H = lambda r, a0, a1, axis, c, seg=24: cyl(r, a0, a1, axis, c, seg)


# ------------------------------------------------------------------ rack segment
def rack_segment():
    L = SEG_LEN
    body = box(-10, TEETH_U1, -40, RACK_ROOT_Y, 0, L)
    tongue = box(-2.8, 2.8, -41.4, -39.9, 0, L)
    s = PITCH / 2 - 2 * BACKLASH
    tip_y = RACK_PITCH_Y + 0.8 * MOD          # shortened addendum: no undercut interference with 14T pinion
    tn = math.tan(math.radians(PA))
    hw = lambda y: s / 2 + (RACK_PITCH_Y - y) * tn
    pts = [(RACK_ROOT_Y - 0.5, 0.0)]  # (y, z) for axis 'x'
    for i in range(SEG_TEETH):
        zc = PITCH * (i + 0.5)
        pts += [(RACK_ROOT_Y, zc - hw(RACK_ROOT_Y)), (tip_y, zc - hw(tip_y)), (tip_y, zc + hw(tip_y)), (RACK_ROOT_Y, zc + hw(RACK_ROOT_Y))]
    pts += [(RACK_ROOT_Y - 0.5, L)]
    teeth = extrude(pts, TEETH_U0, TEETH_U1, 'x')
    part = union_all([body, tongue, teeth])
    holes = union_all([H(M5_CLR / 2, -45, -25, 'y', (0, zh)) for zh in (30.0, L - 30.0)])
    return part - holes


def rack_print(p):  # lay on outer face (u = TEETH_U1) -> teeth flat on the bed
    return p.rotate('y', 90)


# ------------------------------------------------------------------ pinion
def involute_profile(z=PIN_T, m=MOD, pa=PA, n_inv=10, n_arc=4):
    rp = m * z / 2; rb = rp * math.cos(math.radians(pa)); ra = rp + m; rf = rp - 1.25 * m
    inv = lambda a: math.tan(a) - a
    a0 = math.radians(pa)
    def th(r):
        ar = math.acos(min(1.0, rb / r))
        return math.pi / (2 * z) + inv(a0) - inv(ar)
    pts = []
    radii = [rb + (ra - rb) * (i / n_inv) ** 1.3 for i in range(n_inv + 1)]
    for k in range(z):
        ph = 2 * math.pi * k / z
        tb = th(rb)
        pts.append((rf, ph - tb - 0.02))
        for r in radii: pts.append((r, ph - th(r)))
        ta = th(ra)
        for j in range(1, n_arc): pts.append((ra, ph - ta + 2 * ta * j / n_arc))
        for r in radii[::-1]: pts.append((r, ph + th(r)))
        pts.append((rf, ph + tb + 0.02))
        nxt = 2 * math.pi * (k + 1) / z - tb - 0.02
        for j in range(1, n_arc):
            pts.append((rf, ph + tb + 0.02 + (nxt - (ph + tb + 0.02)) * j / n_arc))
    return [(r * math.cos(a), r * math.sin(a)) for r, a in pts]


def pinion():
    width = PIN_U1 - PIN_U0
    r = SHAFT_D / 2 + 0.12
    t = np.linspace(0, 2 * math.pi, 40, endpoint=False)
    flat = SHAFT_FLAT - SHAFT_D / 2 + 0.12   # distance centre->flat
    bore = [(r * math.cos(a), min(r * math.sin(a), flat)) for a in t]
    g = extrude_ring(involute_profile(), bore, 0, width)
    # grub screw: after printing, drill Ø2.5 radially through the flat side at mid-width and self-tap an M3
    return g


# ------------------------------------------------------------------ rail mount (bottom/top)
MOUNT_H = 55.0
SHELF_Z = 44.0


def rail_mount():
    Hh = MOUNT_H
    rplate = box(-10, 36, -40, -34, 0, Hh)
    fplate = box(-19, 19, -6, 0, 0, Hh)
    webs = box(-10, -6, -34, -6, 0, Hh) + box(6, 10, -34, -6, 0, Hh)
    shelf = box(10, 36, -68, -40, SHELF_Z, SHELF_Z + 6)
    gusset = extrude([(-68, SHELF_Z), (-40, SHELF_Z), (-40, 12)], 32, 36, 'x')
    tab = box(14, 34, -59.6, -57.6, SHELF_Z + 6, SHELF_Z + 18)
    part = union_all([rplate, fplate, webs, shelf, gusset, tab])
    cut = []
    for zh in (12.0, 30.0):
        cut.append(H(M5_CLR / 2, -41, -33, 'y', (0, zh)))   # rail screws (M5x10 + hammer nut)
        cut.append(H(5.6, -7, 1, 'y', (0, zh)))             # driver access
    cut.append(slot((0, 38.0), (0, 50.0), 3.3, -7, 1, 'y'))   # centre slot for angle-iron bolt
    for su in (-14.8, 14.8):
        cut.append(slot((su, 8.0), (su, 46.0), 3.3, -7, 1, 'y'))
    for su in (19.25, 28.75):
        cut.append(H(1.1, -61, -56, 'y', (su, SHELF_Z + 6 + 2.9), 12))  # KW11 switch screws (M2 self-tap)
    return part - union_all(cut)


# ------------------------------------------------------------------ carriage bracket (motor + sensor)
def carriage_bracket():
    pad = box(-32.5, 58, -77, PLATE_Y0, -32.5, 32.5)
    wall = box(52, 58, -77, -2, -20, 32.5)
    mpad = box(21, 58, -38, -2, -20, -12)
    wall2 = box(52, 58, -8, 30, -72, -12)
    bar = box(28, 58, 8, 30, -72, -66)
    splate = box(28, 34, 8, 30, -72, -46)
    rib1 = extrude([(-38, -20), (-2, -20), (-2, -32)], 52, 58, 'x')
    part = union_all([pad, wall, mpad, wall2, bar, splate, rib1])
    cut = []
    for (u, w) in PLATE_HOLES:
        cut.append(H(M5_CLR / 2, -78, -66, 'y', (u, w)))
    for sx in (-1, 1):
        for sz in (-1, 1):
            cut.append(H(6.2, -78, -66, 'y', (sx * WHEEL_DX, sz * WHEEL_DZ)))   # axle-screw head / eccentric access
    for u in (29.0, 37.0, 45.0):
        cut.append(slot((u, -32.0), (u, -8.0), M3_CLR / 2, -21, -11, 'z'))  # L-bracket screws (adjust mesh in y)
    cut.append(slot((19.0, -62.0), (19.0, -56.0), 6.3, 27, 35, 'x', 32))  # LJ12A3 Ø12 sensor, +-3 mm height adjust
    return part - union_all(cut)


def bracket_print(p):  # outer wall face (u=58) on the bed
    return p.rotate('y', 90)


def deck_hanger():
    vp = box(-34, -21, -83, -77, -38, 16)
    leg = box(-27, -21, -83, -77, -55, -38)
    fl = box(-48, -21, -140, -77, -55, -49)
    rib = extrude([(-140, -49), (-83, -49), (-83, -32.5)], -27, -21, 'x')
    part = union_all([vp, leg, fl, rib])
    cut = [H(M5_CLR / 2, -84, -76, 'y', (-28.5, w)) for w in (-8.0, 8.0)]
    cut += [H(M5_CLR / 2, -56, -48, 'z', (-38.0, y)) for y in (-92.0, -128.0)]
    return part - union_all(cut)


def hanger_print(p):
    return p.rotate('y', 90)


# ------------------------------------------------------------------ small parts
def wheel_spacer():
    return cyl(5.0, 0, 6, 'z', (0, 0), 40) - cyl(2.65, -1, 7, 'z', (0, 0), 24)


def drill_jig():
    j = box(-32.5, 32.5, -32.5, 32.5, 0, 5)
    j = j + union_all([cyl(2.4, -4, 2.5, 'z', (sx * WHEEL_DX, sz * WHEEL_DZ), 24) for sx in (-1, 1) for sz in (-1, 1)])
    j = j - union_all([cyl(2.6, -1, 6, 'z', (u, w), 20) for (u, w) in PLATE_HOLES])
    return j


def _servo_tab_holes(cx, cy, along, z0, z1, r=1.4):
    out = []
    for d in (-SV_HOLE_DX / 2, SV_HOLE_DX / 2):
        for e in (-SV_HOLE_DY / 2, SV_HOLE_DY / 2):
            x, y = (cx + d, cy + e) if along == 'x' else (cx + e, cy + d)
            out.append(cyl(r, z0, z1, 'z', (x, y), 12))
    return out


def shoulder_tower():
    """frame: x,y global, z = height above tray top"""
    sy = SHOULDER[1]
    bc = sy + SV_L / 2 - SV_SPLINE_OFF       # body centre y (body extends toward +y)
    top = UPPER_Z - (SV_HORN_TOP - SV_TAB_Z)  # tab underside height (127)
    y0, y1 = bc - 30, bc + 30
    base = box(-40, 40, y0 - 16, y1 + 16, 0, 6)
    walls = box(-17, 17, y0, y1, 0, top)
    part = base + walls
    cut = [box(-10.6, 10.6, bc - SV_L / 2 - 0.4, bc + SV_L / 2 + 0.4, 8, top + 1)]
    cut.append(box(16, 18, bc - 12, bc + 12, 20, 100))        # cable window (side)
    cut += _servo_tab_holes(0, bc, 'y', top - 12, top + 1)
    cut += [cyl(M5_CLR / 2, -1, 7, 'z', (sx * 32, yy), 16) for sx in (-1, 1) for yy in (y0 - 8, y1 + 8)]
    # bracket screws on front face (y1)
    cut += [cyl(1.4, y1 - 8, y1 + 1, 'y', (0, zz), 12) for zz in (30.0, 110.0)]
    return part - union_all(cut)


def upper_arm():
    t = ARM_T
    sh = cyl(14, 0, t, 'z', (0, 0), 40)
    bar = box(0, 150, -12, 12, 0, t)
    el = box(140, 200, -16, 16, 0, t)
    part = union_all([sh, bar, el])
    cut = [cyl(4.2, -1, t + 1, 'z', (0, 0), 24)]
    cut += [cyl(M3_CLR / 2, -1, t + 1, 'z', (HORN_PCD / 2 * math.cos(a), HORN_PCD / 2 * math.sin(a)), 12)
            for a in np.radians([45, 135, 225, 315])]
    bc = L1 + SV_SPLINE_OFF - SV_L / 2
    cut.append(box(bc - SV_L / 2 - 0.3, bc + SV_L / 2 + 0.3, -10.4, 10.4, -1, t + 1))
    cut += _servo_tab_holes(bc, 0, 'x', -1, t + 1)
    cut += [box(30, 120, -6, 6, -1, t + 1)]                       # lightening / cable slot
    return part - union_all(cut)


def forearm():
    t = ARM_T
    el = cyl(16, 0, t, 'z', (0, 0), 40)
    bar = box(0, L2, -10, 10, 0, t)
    tip = cyl(11, 0, t, 'z', (L2, 0), 32)
    pin = cyl(PIN_DIA / 2, -PIN_LEN + 3, 0.5, 'z', (L2, 0), 24) + cyl(PIN_DIA / 2 - 2.5, -PIN_LEN, -PIN_LEN + 3, 'z', (L2, 0), 24, r1=PIN_DIA / 2)
    fil = cyl(PIN_DIA / 2 + 3, -3, 0.5, 'z', (L2, 0), 24, r1=PIN_DIA / 2 + 6)
    part = union_all([el, bar, tip, pin, fil])
    cut = [cyl(4.2, -1, t + 1, 'z', (0, 0), 24)]
    cut += [cyl(M3_CLR / 2, -1, t + 1, 'z', (HORN_PCD / 2 * math.cos(a), HORN_PCD / 2 * math.sin(a)), 12)
            for a in np.radians([45, 135, 225, 315])]
    cut += [box(30, 140, -5, 5, -1, t + 1)]
    return part - union_all(cut)


def forearm_print(p):
    return p.rotate('x', 180)


def basket_stop():
    base = box(-20, 20, -24, 0, 0, 5)
    up = box(-20, 20, -6, 0, 0, 26)
    gus = extrude([(-24, 5), (-6, 5), (-6, 20)], -3, 3, 'x')
    part = union_all([base, up, gus])
    cut = [cyl(M5_CLR / 2, -1, 6, 'z', (sx * 13, -15), 16) for sx in (-1, 1)]
    cut += [cyl(1.1, -7, 1, 'y', (sx, 18), 12) for sx in (-4.75, 4.75)]   # KW11 switch (M2) on the front face
    return part - union_all(cut)


def cam_tof_bracket():
    """plate on the tower front face: ESP32-CAM cradle (top) + VL53L0X pocket (bottom). frame: x, y(out = +y), z"""
    plate = box(-24, 24, 0, 4, 20, 128)
    # ESP32-CAM 27 x 40.5 board, cradle lips
    cx0, cx1, cz0, cz1 = -14.2, 14.2, 72, 113.5
    cr = box(cx0 - 2, cx1 + 2, 4, 11, cz0 - 2, cz1) - box(cx0, cx1, 3, 12, cz0, cz1 + 1)
    lip = box(cx0 - 2, cx0 + 1.5, 9, 11, cz0 - 2, cz1) + box(cx1 - 1.5, cx1 + 2, 9, 11, cz0 - 2, cz1)
    # VL53L0X (GY-530) ~ 25 x 13 board: 2 holes + pocket
    tof = box(-14, 14, 4, 7, 32, 50) - box(-12.7, 12.7, 3, 8, 34, 48)
    part = union_all([plate, cr, lip, tof])
    cut = [cyl(1.7, -1, 5, 'y', (0, zz), 12) for zz in (30.0, 110.0)]
    cut += [box(-3, 3, -1, 5, 55, 66)]   # cable pass
    cut += [cyl(1.0, -1, 8, 'y', (sx * 10, 41), 10) for sx in (-1, 1)]
    return part - union_all(cut)


def cam_print(p):
    return p.rotate('x', 90)


def tof_down_mount():
    """spacer block bolted under the deck; VL53L0X board screws to its bottom face, looking at the floor"""
    part = box(-15, 15, -10, 10, 0, 12)
    cut = [cyl(M5_CLR / 2, -1, 13, 'z', (0, 0), 16), cyl(4.6, -1, 4, 'z', (0, 0), 6)]   # M5 + nut trap
    cut += [cyl(1.0, -1, 7, 'z', (sx * 10, 6), 10) for sx in (-1, 1)]
    return part - union_all(cut)
