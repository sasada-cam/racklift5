"""v2 assembly: 2 bays, rail grid with turntable junctions, climbing robot with level tray + SCARA picker."""
import math, numpy as np
from csg import *
from params import *
import parts as P1
import parts_v2 as P2

C_ANGLE = (125, 135, 150); C_RAIL = (205, 210, 215); C_PLATE = (60, 65, 75); C_WHEEL = (35, 35, 38)
C_PRINT = (245, 130, 35); C_PRINT2 = (40, 170, 160); C_JUNC = (150, 95, 200); C_STEEL = (150, 150, 158)
C_GB = (205, 175, 80); C_CAN = (190, 192, 198); C_SERVO = (25, 55, 140); C_PCB = (20, 120, 60)
C_TOF = (120, 60, 160); C_IND = (210, 190, 90); C_WOOD = (220, 185, 135); C_BOLT = (80, 80, 85); C_SW = (30, 30, 30)
BASKET_COLS = [(90, 160, 225), (235, 95, 110), (120, 200, 120), (250, 200, 70), (170, 130, 220),
               (80, 190, 190), (240, 150, 60), (160, 160, 90), (220, 120, 190), (110, 130, 230)]

_cache = {}


def part(mod, name, *a):
    k = (mod.__name__, name, a)
    if k not in _cache:
        _cache[k] = getattr(mod, name)(*a)
    return _cache[k]


def MT(x=0, y=0, z=0):
    M = np.eye(4); M[:3, 3] = [x, y, z]; return M


def MX():
    M = np.eye(4); M[0, 0] = -1; return M


def merge(solids):
    return Solid(sum([s.polys for s in solids], []))


def vslot_z(z0, z1):
    h, s, d = 10, 3.1, 6.0
    pts = [(-h, -h), (-s, -h), (-s, -h + d), (s, -h + d), (s, -h), (h, -h), (h, -s), (h - d, -s), (h - d, s), (h, s),
           (h, h), (s, h), (s, h - d), (-s, h - d), (-s, h), (-h, h), (-h, s), (-h + d, s), (-h + d, -s), (-h, -s)]
    return extrude([(a, b - 50) for a, b in pts], z0, z1, 'z')     # rail centre at y=-50


def basket_solid():
    x = BASKET_W / 2; t = 3
    walls = [box(-x, x, 0, BASKET_D, 0, t), box(-x, -x + t, 0, BASKET_D, 0, BASKET_H), box(x - t, x, 0, BASKET_D, 0, BASKET_H),
             box(-x, x, 0, t, 0, BASKET_H), box(-x, x, BASKET_D - t, BASKET_D, 0, BASKET_H),
             box(-25, 25, -0.5, 0, 35, 85)]  # tag
    return merge(walls)


# ------------------------------------------------------------------ static world
def rack_bay(bx):
    ang = []
    for sx in (-1, 1):
        S = MT(bx) @ (MX() if sx < 0 else np.eye(4))
        fr = [box(XI, XI + ANG_T, 0, ANG, 0, RACK_H), box(XI, XI + ANG, 0, ANG_T, 0, RACK_H),
              box(XI, XI + ANG_T, RACK_D - ANG, RACK_D, 0, RACK_H), box(XI + ANG_T - ANG, XI + ANG_T, RACK_D - ANG_T, RACK_D, 0, RACK_H)]
        for z in LEVELS + [RACK_H]:
            fr += [box(XI - ANG_T, XI, 0, RACK_D, z - ANG, z), box(XI - ANG, XI, 0, RACK_D, z - ANG_T, z)]
        ang += [f.transform(S) for f in fr]
    ang += [box(-XI - ANG_T, XI + ANG_T, RACK_D, RACK_D + ANG_T, 0, ANG).translate(bx), box(-XI - ANG_T, XI + ANG_T, RACK_D, RACK_D + ANG_T, RACK_H - ANG, RACK_H).translate(bx),
            box(-XI - ANG, XI + ANG, -ANG_T, 0, 0, ANG).translate(bx)]
    return merge(ang)


def base_plate_outline():
    """junction base plate (9 mm plywood), junction-relative (x, z)"""
    return [(-200, -130), (200, -130), (200, 20), (-200, 20)]


_STATIC = None


def static_items():
    global _STATIC
    if _STATIC is not None:
        return _STATIC
    it = []
    add = lambda n, s, c: it.append((n, s, c))
    for bx in BAY_X:
        add("slotted angle rack (bay)", rack_bay(bx), C_ANGLE)
    for i, (jx, jz) in enumerate(JUNCTIONS):
        # vertical fixed rail + its rack strips
        z0 = jz + STUB0
        add("V-slot rail (vertical)", vslot_z(z0, RAIL_TOP).translate(jx), C_RAIL)
        add("printed in-slot rack", P2.rack_strip(STUB0, RAIL_TOP - jz).translate(jx, 0, z0), C_PRINT)
        # top bridge
        add("printed rail bridge", P2.rail_bridge().translate(jx, 0, RAIL_TOP - 45), C_PRINT)
        # junction base plate
        bp = extrude(base_plate_outline(), -31, -22, 'y').translate(jx, 0, jz)
        add("junction base plate (9 mm plywood)", bp, C_WOOD)
        for (rx, rz) in RECV:
            add("torque pin sleeve", P2.pin_sleeve().rotate('x', 90).translate(jx + rx, -31, jz + rz), C_JUNC)
            add("M8 nut (pin tip)", hexagon(13, PIN_TIP_Y, PIN_TIP_Y + 6.5, (jx + rx, jz + rz), 'y'), C_BOLT)
        add("printed rail bridge", P2.rail_bridge().translate(jx, 0, jz + 165), C_PRINT)
        add("printed detent", P2.detent().rotate('y', 180).translate(jx, 0, jz), C_JUNC)          # holds turntable vertical
        add("printed detent", P2.detent().transform(MX()).rotate('y', -90).translate(jx, 0, jz), C_JUNC)  # holds it horizontal
    # horizontal rail between J1 and J2
    (x1, jz), (x2, _) = JUNCTIONS[0], JUNCTIONS[1]
    L = (x2 - STUB0) - (x1 + STUB0)
    hr = vslot_z(0, L).rotate('y', 90).translate(x1 + STUB0, 0, jz)
    add("V-slot rail (horizontal)", hr, C_RAIL)
    add("printed in-slot rack", P2.rack_strip(STUB0, x2 - x1 - STUB0).rotate('y', 90).translate(x1 + STUB0, 0, jz), C_PRINT)
    for xx in (x1 + STUB0 + 25, x2 - STUB0 - 25):
        add("printed stub block", P2.stub_block().rotate('y', 90).translate(xx, 0, jz), C_JUNC)
    _STATIC = it
    return it


def rotor_items(j, ang):
    """rotating part of junction j at angle ang (deg, 0 = vertical, -90 = horizontal)"""
    jx, jz = JUNCTIONS[j]
    M = MT(jx, 0, jz) @ rot('y', ang)
    it = [("junction rotor", P2.rotor().transform(M), C_JUNC),
          ("rail piece on turntable", vslot_z(-SEG_HALF, SEG_HALF).transform(M), C_RAIL),
          ("printed in-slot rack (turntable)", P2.rack_strip(-SEG_HALF, SEG_HALF).translate(0, 0, -SEG_HALF).transform(M), C_PRINT)]
    return it


# ------------------------------------------------------------------ robot body (body frame)
_BODY = None


def body_items():
    global _BODY
    if _BODY is not None:
        return _BODY
    it = []
    add = lambda n, s, c: it.append((n, s, c))
    for ac in PLATE_AC:
        add("gantry plate", box(-PLATE_W / 2, PLATE_W / 2, PLATE_Y0, PLATE_Y1, ac - PLATE_W / 2, ac + PLATE_W / 2), C_PLATE)
    for a in WHEEL_A:
        for sx in (-1, 1):
            add("V wheel", cyl(WHEEL_R, WHEEL_YC - WHEEL_W / 2, WHEEL_YC + WHEEL_W / 2, 'y', (sx * WHEEL_DX, a), 28), C_WHEEL)
            add("spacer", cyl(5, PLATE_Y1, WHEEL_YC - WHEEL_W / 2, 'y', (sx * WHEEL_DX, a), 12), C_STEEL)
    add("printed chassis", P2.chassis(), C_PRINT)
    add("printed climber yoke", P2.climber_yoke(), C_PRINT)
    # climber: pinion in the slot, shaft along +s, JGY-370 beside it
    add("printed climber pinion", P2.climber_pinion().translate(0, 0, -CP_W / 2).rotate('y', 90).translate(0, CP_Y, CP_A), C_PRINT2)
    add("climber shaft", cyl(3, 0, 13, 'x', (CP_Y, CP_A), 12), C_STEEL)
    add("steel L-bracket (climber)", merge([box(9, 11, CP_Y - 16, -61, CP_A - 16, CP_A + 16), box(9, 34, -105.8, -103.8, CP_A - 16, CP_A + 16)]), C_STEEL)
    add("JGY-370 climb motor", box(11, 33.5, CP_Y - 36, CP_Y + 10, CP_A - 16, CP_A + 16), C_GB)
    add("JGY-370 climb motor can", cyl(12.2, CP_Y - 36 - 33, CP_Y - 36, 'y', (22.25, CP_A), 24), C_CAN)
    # turn drive
    add("JGY-370 turn motor", box(-10, 36, -133, -110.5, TURN_A - 16, TURN_A + 16), C_GB)
    add("JGY-370 turn motor can", cyl(12.2, 36, 69, 'x', (-121.75, TURN_A), 24), C_CAN)
    add("steel L-bracket (turn)", merge([box(-10, 36, -135, -133, TURN_A - 16, TURN_A + 16), box(-12, -10, -135, -100, TURN_A - 16, TURN_A + 16)]), C_STEEL)
    add("printed turn pinion", P2.turn_pinion().rotate('x', -90).translate(0, -147.0, TURN_A), C_PRINT2)
    add("M8 axle", cyl(4, -158, -66, 'y', (0, 0), 16), C_STEEL)
    add("printed axle spacer", P2.axle_spacer(), C_PRINT)
    add("limit switches (hand angle)", merge([box(-30, -20, -122, -110, 20, 26), box(20, 30, -122, -110, 20, 26)]), C_SW)
    _BODY = it
    return it


# ------------------------------------------------------------------ tray + hand (world-level frame, origin = robot centre)
_TRAY = None


def tray_items(pin_y):
    global _TRAY
    if _TRAY is not None:
        return _TRAY
    it = []
    add = lambda n, s, c: it.append((n, s, c))
    bxr = RAIL_OFF                                   # bay centre relative to robot x
    T = TRAY_TOP_REL
    add("printed hand (tray arm + 48T gear)", P2.tray_arm(), C_PRINT)
    add("printed fork crown", P2.crown(), C_PRINT)
    for rc in RECV:
        add("LJ12A3 (junction pin)", cyl(6, HUB_Y1 - 50, HUB_Y1 + 4, 'y', rc, 20), C_IND)
    add("printed tray foot", P2.tray_foot(), C_PRINT)
    add("plywood tray 12 mm", box(TRAY_X0, TRAY_X1, DECK_Y1, DECK_Y0, T - TRAY_T, T), C_WOOD)
    M = MT(bxr, 0, T)
    add("printed shoulder tower", P1.shoulder_tower().transform(M), C_PRINT)
    bc = SHOULDER[1] + SV_L / 2 - SV_SPLINE_OFF
    add("printed cam/ToF bracket", P1.cam_tof_bracket().transform(MT(bxr, bc + 30, T)), C_PRINT)
    add("ESP32-CAM", box(-13.5, 13.5, bc + 35, bc + 39, 72.5, 113).translate(bxr, 0, T), C_PCB)
    add("VL53L0X (basket)", box(-12.5, 12.5, bc + 35, bc + 36.5, 34.5, 47.5).translate(bxr, 0, T), C_TOF)
    for sx in (-80, 80):
        add("printed basket stop", P1.basket_stop().transform(MT(bxr + sx, -193, T)), C_PRINT)
    add("limit switch (basket on tray)", box(-10, 10, -193, -186.6, 13, 23).translate(bxr - 80, 0, T), C_SW)
    add("VL53L0X (height)", box(-12, 12, -318, -305, T - TRAY_T - 1.5, T - TRAY_T).translate(bxr + 100, 0, 0), C_TOF)
    add("MPU-6050", box(-10, 10, -260, -244, T - TRAY_T - 1.5, T - TRAY_T).translate(bxr, 0, 0), C_PCB)
    _TRAY = it
    return it


def arm_local():
    """SCARA pieces in their own joint frames (relative to tray top) for the viewer"""
    up = [("printed upper arm", P1.upper_arm().translate(0, 0, UPPER_Z), C_PRINT)]
    from assembly_servo import servo_model
    b, h = servo_model((L1, 0), 180, UPPER_Z + SV_TAB_Z, inverted=True)
    up += [("20kg servo (elbow)", b, C_SERVO), ("servo horn", h, C_STEEL)]
    fo = [("printed forearm + hook", P1.forearm().translate(0, 0, UPPER_Z - (SV_HORN_TOP - SV_TAB_Z) - ARM_T), C_PRINT)]
    b2, h2 = servo_model((0, 0), 90, UPPER_Z - SV_HORN_TOP)
    sh = [("20kg servo (shoulder)", b2, C_SERVO), ("servo horn", h2, C_STEEL)]
    return up, fo, sh


def ik(target_y):
    sx, sy = SHOULDER
    d = abs(target_y - sy)
    beta = math.acos(max(-1, min(1, d / (2 * L1))))
    q1 = math.pi / 2 - beta
    E = (sx + L1 * math.cos(q1), sy + L1 * math.sin(q1))
    q2 = math.atan2(target_y - E[1], 0 - E[0])
    return q1, E, q2


# ------------------------------------------------------------------ full pose
def build(C, theta, rotor_angles, pin_y=-178.0, baskets=None):
    """C = (x, z) robot centre; theta = body angle (0 = vertical, -90 = horizontal);
    baskets: list of (bay, level) or 'tray'"""
    items = []
    for n, s, c in static_items():
        items.append((n, s, c, "static"))
    for j, a in enumerate(rotor_angles):
        for n, s, c in rotor_items(j, a):
            items.append((n, s, c, "rotor"))
    Mb = MT(C[0], 0, C[1]) @ rot('y', theta)
    for n, s, c in body_items():
        items.append((n, s.transform(Mb), c, "body"))
    Mt = MT(C[0], 0, C[1])
    for n, s, c in tray_items(pin_y):
        items.append((n, s.transform(Mt), c, "tray"))
    # arm
    up, fo, sh = arm_local()
    T = C[1] + TRAY_TOP_REL
    bxw = C[0] + RAIL_OFF
    q1, E, q2 = ik(pin_y)
    for n, s, c in sh:
        items.append((n, s.transform(MT(bxw + SHOULDER[0], SHOULDER[1], T)), c, "arm"))
    for n, s, c in up:
        items.append((n, s.transform(MT(bxw + SHOULDER[0], SHOULDER[1], T) @ rot('z', math.degrees(q1))), c, "arm"))
    for n, s, c in fo:
        items.append((n, s.transform(MT(bxw + E[0], E[1], T) @ rot('z', math.degrees(q2))), c, "arm"))
    # baskets
    bs = basket_solid()
    for k, b in enumerate(baskets or []):
        if b == 'tray':
            items.append((f"basket (on tray)", bs.translate(bxw, -193, T), BASKET_COLS[k % 10], "tray"))
        else:
            bay, lv = b
            items.append((f"basket bay{bay+1}-L{lv+1}", bs.translate(BAY_X[bay], BASKET_Y0, LEVELS[lv]), BASKET_COLS[k % 10], "static"))
    return items
