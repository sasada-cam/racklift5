"""v2 printable parts: in-slot gear racks, climbing robot, tray arm, junction turntables.
Robot BODY frame (robot standing on a vertical rail): x = s (across the rail, + toward its bay),
z = a (along the rail, + up), y = global y (rack face at y = 0, rail front face at y = -60).
Junction frame: same axes, origin at the junction centre."""
import math, numpy as np
from csg import *
from params import *
from parts import involute_profile, _servo_tab_holes

H = lambda r, a0, a1, axis, c, seg=24: cyl(r, a0, a1, axis, c, seg)
TN = math.tan(math.radians(PA))


# ------------------------------------------------------------------ in-slot rack strip
def rack_strip(a0, a1):
    """rack strip that slides into the FRONT slot of a V-slot; teeth point toward -y.
    a0..a1 measured from a tooth-SPACE centre (tooth centres at (k+.5)P). Returned with a0 at z=0."""
    s = P / 2 - 2 * BACKLASH
    hw = lambda y: s / 2 + (y - RK_PITCH_Y) * TN
    pts = [(RK_ROOT_Y + 0.6, a0)]
    k0 = int(math.floor(a0 / P)) - 1
    k1 = int(math.ceil(a1 / P)) + 1
    for k in range(k0, k1):
        zc = (k + 0.5) * P
        lo_r, hi_r = zc - hw(RK_ROOT_Y), zc + hw(RK_ROOT_Y)
        if hi_r <= a0 + 0.05 or lo_r >= a1 - 0.05:
            continue
        seq = [(RK_ROOT_Y, zc - hw(RK_ROOT_Y)), (RK_TIP_Y, zc - hw(RK_TIP_Y)), (RK_TIP_Y, zc + hw(RK_TIP_Y)), (RK_ROOT_Y, zc + hw(RK_ROOT_Y))]
        seq = [(y, min(max(z, a0), a1)) for y, z in seq]
        pts += seq
    pts += [(RK_ROOT_Y + 0.6, a1)]
    # remove consecutive duplicates
    clean = []
    for p_ in pts:
        if not clean or abs(clean[-1][0] - p_[0]) > 1e-6 or abs(clean[-1][1] - p_[1]) > 1e-6:
            clean.append(p_)
    teeth = extrude(clean, -RK_TEETH_W / 2, RK_TEETH_W / 2, 'x')
    flange = box(-RK_FLANGE_W / 2, RK_FLANGE_W / 2, RK_ROOT_Y, RK_FLANGE_Y1 - 0.2, a0, a1)
    neck = box(-RK_TEETH_W / 2, RK_TEETH_W / 2, RK_ROOT_Y - 0.01, RK_ROOT_Y + 0.6, a0, a1)
    return union_all([flange, neck, teeth]).translate(0, 0, -a0)


RACK_STRIPS = {  # name: (a0, a1, qty)
    "rack_turntable_half": (0.0, SEG_HALF, 4),       # 2 per turntable (one each side of centre)
    "rack_end": (STUB0, 65 * P, 2),                  # first strip of each vertical rail
    "rack_33T": (0.0, 33 * P, 6),
    "rack_18T_top": (0.0, 18 * P, 2),
    "rack_horizontal": (STUB0, 61 * P - 1.0, 1),     # the short rail between the junctions
}


# ------------------------------------------------------------------ gears
def gear_part(z, width, bore_flat=True, hub=None):
    r = SHAFT_D / 2 + 0.12
    t = np.linspace(0, 2 * math.pi, 40, endpoint=False)
    flat = SHAFT_FLAT - SHAFT_D / 2 + 0.12
    bore = [(r * math.cos(a), min(r * math.sin(a), flat) if bore_flat else r * math.sin(a)) for a in t]
    g = extrude_ring(involute_profile(z=z), bore, 0, width)
    if hub:
        hl, hr = hub
        ring = [(hr * math.cos(a), hr * math.sin(a)) for a in np.linspace(0, 2 * math.pi, 40, endpoint=False)]
        bore2 = [(x * 1.004, y * 1.004) for x, y in bore[::-1]][::-1]
        g = Solid(g.polys + extrude_ring(ring, bore2, width - 0.6, width + hl).polys)   # overlapping shell (slicers merge)
    return g


def climber_pinion():   # 14T, 5 mm face (runs in the 6.2 mm slot opening) + 6 mm hub
    return gear_part(14, CP_W, hub=(6.0, 7.0))


def turn_pinion():      # 12T, 8 mm face, on the turn motor
    return gear_part(TURN_PIN_T, GEAR_T)


# ------------------------------------------------------------------ robot chassis
def chassis():
    base = box(-37, 37, -77, -67, -72, 65)
    ext = box(-37, -4, -77, -67, 65, 100)           # tab for the climber yoke
    hub = cyl(14, -112, -76.5, 'y', (0, 0), 40)
    pil = box(-10, 36, -110.5, -76.5, TURN_A - 16, TURN_A + 16)   # turn-motor pillar
    rib = box(-37, -31, -110.5, -76.5, TURN_A - 16, -20)
    part = union_all([base, ext, hub, pil, rib])
    cut = []
    for ac in PLATE_AC:
        for ss in (-1, 1):
            for dz in (-8, 8):
                cut.append(H(M5_CLR / 2, -78, -66, 'y', (ss * 28.5, ac + dz)))
        for ss in (-1, 1):
            for wz in (-20, 20):
                cut.append(H(6.2, -78, -66, 'y', (ss * WHEEL_DX, ac + wz)))
    cut.append(H(4.15, -114, -66, 'y', (0, 0)))                       # M8 axle
    cut.append(hexagon(13.3, -73.5, -66, (0, 0), 'y'))                 # M8 nut trap (behind)
    cut += [H(1.6, -112, -100, 'y', (ss, TURN_A), 12) for ss in (-4.0, 30.0)]   # L-bracket screws (self-tap)
    cut += [H(M5_CLR / 2, -78, -66, 'y', (-20.5, aa)) for aa in (75.0, 92.0)]  # yoke screws
    return part - union_all(cut)


def climber_yoke():
    side = box(-37, -31, -110, -77, 66, 100)
    front = box(-37, 36, -110, -104, 66, 100)
    foot = box(-37, -4, -84, -77, 66, 100)
    part = union_all([side, front, foot])
    cut = [H(13.0, -111, -103, 'y', (22.25, CP_A), 32)]                  # motor can passes through
    cut += [H(M5_CLR / 2, -85, -76, 'y', (-20.5, aa)) for aa in (75.0, 92.0)]
    cut += [slot((ss, 70.0), (ss, 96.0), 1.7, -111, -103, 'y') for ss in (12.0, 33.0)]   # L-bracket base (adjust mesh)
    return part - union_all(cut)


# ------------------------------------------------------------------ tray arm (world-level frame, origin = robot centre)
def tray_arm():
    y_g0, y_g1 = HUB_Y0, HUB_Y0 - GEAR_T     # gear -135..-147
    y_p0, y_p1 = y_g1, HUB_Y1                 # plate -143..-151
    gear = extrude_ring(involute_profile(z=48), [(11.05 * math.cos(a), 11.05 * math.sin(a)) for a in np.linspace(0, 2 * math.pi, 48, endpoint=False)], 0, 8)
    gear = extrude_ring(involute_profile(z=HAND_GEAR_T), [(11.05 * math.cos(a), 11.05 * math.sin(a)) for a in np.linspace(0, 2 * math.pi, 48, endpoint=False)], 0, GEAR_T + 0.6)
    gear = gear.rotate('x', 90).translate(0, y_g0, 0)          # y -135 .. -143.6 (overlaps the plate)
    # plate outline (x, z)
    outline = [(-36, 30), (34, 34), (82, 22), (118, 22), (118, -22), (82, -22), (40, -36),
               (-10, -40), (-62, -60), (-76, -72), (-91, -57), (-42, -14)]
    plate = extrude([(x, z) for x, z in outline], 0, y_p0 - y_p1, 'y').translate(0, y_p1, 0)
    tube = cyl(16, y_p1, y_g0, 'y', (0, 0), 40)
    tube = cyl(16, y_p1, y_g1 + 0.6, 'y', (0, 0), 40)
    part = union_all([plate, tube])
    cut = [cyl(11.05, y_p1 - 1, y_p1 + 7.2, 'y', (0, 0), 40), cyl(7.2, y_p1 - 1, y_g1 + 1, 'y', (0, 0), 24)]
    cut += [H(M5_CLR / 2, y_p1 - 1, y_p0 + 1, 'y', (x, z)) for x, z in ((-70, -62), (-86, -60))]   # crown screws
    cut += [H(M5_CLR / 2, y_p1 - 1, y_p0 + 1, 'y', (x, z)) for x, z in ((100, 10), (100, -10))]   # tray-foot screws
    return Solid((part - union_all(cut)).polys + gear.polys)


def crown():
    """two 4-way receivers that slide over the junction torque pins; bolts to the tray-arm."""
    y0, y1 = HUB_Y1 - 6, HUB_Y1               # plate -157..-151
    yb = -127.0                               # prongs reach back to here
    discs = [cyl(21, y0, y1, 'y', c, 40) for c in RECV]
    link = box(-96, -60, y0, y1, -70, -52)
    link2 = box(-104, -88, y0, y1, -86, -60)
    plate = union_all(discs + [link, link2])
    prongs = []
    g = RECV_GAP / 2
    for (cx, cz) in RECV:
        for sx in (-1, 1):
            for sz in (-1, 1):
                x0, x1 = sorted((cx + sx * g, cx + sx * 17))
                z0, z1 = sorted((cz + sz * g, cz + sz * 17))
                prongs.append(box(x0, x1, yb, y1, z0, z1))
    part = union_all([plate] + prongs)
    cut = [H(M5_CLR / 2, y0 - 1, y1 + 1, 'y', c) for c in ((-70, -62), (-86, -60))]
    cut += [H(6.15, y0 - 1, y1 + 1, 'y', rc, 32) for rc in RECV]   # 2x LJ12A3 look at the pin nuts
    return part - union_all(cut)


def tray_foot():
    zt = TRAY_TOP_REL - TRAY_T                 # tray underside
    vp = box(82, 118, HUB_Y1 - 6, HUB_Y1, -22, zt)
    hp = box(100, 230, -205, HUB_Y1, zt - 6, zt)
    hp2 = box(100, 160, HUB_Y1, -95, zt - 6, zt)
    gus = extrude([(HUB_Y1 - 6, -22), (HUB_Y1 - 6, zt - 6), (-200, zt - 6)], 100, 106, 'x')
    part = union_all([vp, hp, hp2, gus])
    cut = [H(M5_CLR / 2, HUB_Y1 - 7, HUB_Y1 + 1, 'y', (x, z)) for x, z in ((100, 10), (100, -10))]
    cut += [H(M5_CLR / 2, zt - 10, zt + 5, 'z', (x, y)) for x, y in ((130, -110), (200, -170), (130, -195), (215, -185))]
    return part - union_all(cut)


# ------------------------------------------------------------------ junction parts (junction frame, rotor drawn vertical)
ROTOR_HALF = 105.0


def rotor():
    bar = box(-20, 20, -40, -32, -ROTOR_HALF, ROTOR_HALF)
    hubd = cyl(22, -40, -32, 'y', (0, 0), 40)
    part = bar + hubd
    cut = [hexagon(13.3, -41, -34.6, (0, 0), 'y'), H(4.15, -41, -31, 'y', (0, 0))]
    for aa in (-62.0, 62.0):
        cut += [H(M5_CLR / 2, -41, -31, 'y', (0, aa)), H(5.2, -35, -31, 'y', (0, aa))]
    cut += [H(4.0, -41, -31, 'y', (0, e * ROTOR_HALF), 20) for e in (-1, 1)]   # detent notches
    cut += [box(-12, 12, -41, -31, s * 26, s * 50) for s in (-1, 1)]               # lightening
    return part - union_all(cut)


def detent():
    """leaf-spring detent, screwed to the base plate; bump drops into the rotor end notch"""
    block = box(18, 32, -40, -31, ROTOR_HALF + 3, ROTOR_HALF + 16)
    arm = box(-2, 18, -39.5, -32.5, ROTOR_HALF + 4.0, ROTOR_HALF + 6.5)
    bump = cyl(3.6, -39.5, -32.5, 'y', (0, ROTOR_HALF + 4.0), 24)
    part = union_all([block, arm, bump])
    cut = [H(2.2, -41, -30, 'y', (25, ROTOR_HALF + 9.5), 12)]
    return part - union_all(cut)


def pin_sleeve():
    L = -31.0 - (PIN_TIP_Y + 6.5)            # from base plate to the tip nut (~111.5)
    s = cyl(PIN_D_FIX / 2, 0, L - 2, 'z', (0, 0), 32).union(cyl(PIN_D_FIX / 2, L - 2, L, 'z', (0, 0), 32, r1=PIN_D_FIX / 2 - 1.2))
    return s - cyl(4.2, -1, L + 1, 'z', (0, 0), 24)


def stub_block():
    b = box(-10, 10, -40, -31, -20, 20)
    return b - union_all([H(M5_CLR / 2, -41, -30, 'y', (0, a)) for a in (-12.0, 12.0)])


def base_spacer():
    return cyl(8, 0, 22, 'z', (0, 0), 32) - cyl(M5_CLR / 2, -1, 23, 'z', (0, 0), 16)


def rail_bridge():
    """holds a vertical rail 40 mm in front of the rack beside a post. Frame: x = global x - rail x."""
    rp = box(-10, 10, -40, -33, 0, 40)
    fp = box(-50, 72, -6, 0, 0, 40)
    w1 = box(-10, -6, -33, -6, 0, 40)
    w2 = box(6, 10, -33, -6, 0, 40)
    gus = [extrude([(10, -33), (40, -6), (10, -6)], z0, z0 + 5, 'z') for z0 in (0.0, 35.0)]
    part = union_all([rp, fp, w1, w2] + gus)
    cut = [H(M5_CLR / 2, -41, -32, 'y', (0, z)) for z in (12.0, 28.0)]
    cut += [H(5.6, -7, 1, 'y', (0, z)) for z in (12.0, 28.0)]
    for x0 in (-42.0, -26.0, 50.0, 64.0):
        cut.append(slot((x0, 10.0), (x0, 30.0), 3.3, -7, 1, 'y'))
    return part - union_all(cut)


def axle_spacer():
    """tube between the chassis boss (y -112) and the hand's rear bearing (y -135)"""
    return cyl(7, -134.9, -112.1, 'y', (0, 0), 32) - cyl(4.2, -136, -111, 'y', (0, 0), 24)
