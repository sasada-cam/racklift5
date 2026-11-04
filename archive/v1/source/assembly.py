"""Full assembly: rack + rails + lift + SCARA picker + sensors."""
import math, numpy as np
from csg import *
from params import *
import parts as P

C_ANGLE = (125, 135, 150); C_RAIL = (205, 210, 215); C_PLATE = (60, 65, 75); C_WHEEL = (35, 35, 38)
C_PRINT = (245, 130, 35); C_PRINT2 = (40, 170, 160); C_STEEL = (150, 150, 158); C_GB = (205, 175, 80)
C_CAN = (190, 192, 198); C_SERVO = (25, 55, 140); C_PCB = (20, 120, 60); C_TOF = (120, 60, 160)
C_IND = (210, 190, 90); C_WOOD = (220, 185, 135); C_BOLT = (80, 80, 85); C_SW = (30, 30, 30)
BASKET_COLS = [(90, 160, 225), (235, 95, 110), (120, 200, 120), (250, 200, 70), (170, 130, 220)]

_cache = {}


def part(name):
    if name not in _cache:
        _cache[name] = getattr(P, name)()
    return _cache[name]


def M_T(x=0, y=0, z=0):
    M = np.eye(4); M[:3, 3] = [x, y, z]; return M


def M_mirror_x():
    M = np.eye(4); M[0, 0] = -1; return M


def M_mirror_z():
    M = np.eye(4); M[2, 2] = -1; return M


def side_M(side, pc_or_z):
    """right-rail local (u,y,w) -> global for side +1 (right) / -1 (left)"""
    M = M_T(XR, 0, pc_or_z)
    if side < 0:
        M = M_mirror_x() @ M
    return M


def vslot(x0, x1, y0, y1, z0, z1):
    """2020 V-slot, simplified profile with 4 slot openings"""
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    h = 10; s = 3.1; d = 1.8
    pts = []
    # square with notch on each side, CCW starting bottom-left
    pts = [(-h, -h), (-s, -h), (-s, -h + d), (s, -h + d), (s, -h), (h, -h), (h, -s), (h - d, -s), (h - d, s), (h, s),
           (h, h), (s, h), (s, h - d), (-s, h - d), (-s, h), (-h, h), (-h, s), (-h + d, s), (-h + d, -s), (-h, -s)]
    pts = [(cx + a, cy + b) for a, b in pts]
    return extrude(pts, z0, z1, 'z')


def servo_model(spline_xy, ang_deg, bottom_z, inverted=False):
    b = box(-SV_SPLINE_OFF, SV_L - SV_SPLINE_OFF, -SV_W / 2, SV_W / 2, 0, SV_H)
    tabs = box(-SV_SPLINE_OFF - 7, SV_L - SV_SPLINE_OFF + 7, -SV_W / 2, SV_W / 2, SV_TAB_Z, SV_TAB_Z + SV_TAB_T)
    sp = cyl(6.5, SV_H, SV_H + 4, 'z', (0, 0), 20)
    horn = cyl(HORN_D / 2, SV_HORN_TOP - 4, SV_HORN_TOP, 'z', (0, 0), 28)
    body = Solid(b.polys + tabs.polys)
    hs = Solid(sp.polys + horn.polys)
    M = rot('z', ang_deg)
    if inverted:
        M = M @ rot('x', 180)
    M = M_T(spline_xy[0], spline_xy[1], bottom_z) @ M
    return body.transform(M), hs.transform(M)


def ik(target_y):
    sx, sy = SHOULDER
    d = abs(target_y - sy)
    beta = math.acos(max(-1, min(1, (L1 ** 2 + d ** 2 - L2 ** 2) / (2 * L1 * d))))
    q1 = math.pi / 2 - beta          # elbow on +x side
    E = (sx + L1 * math.cos(q1), sy + L1 * math.sin(q1))
    q2 = math.atan2(target_y - E[1], 0 - E[0])
    return q1, E, q2


def build(level_idx=2, state="reach", raise_=0.0, T=None, pin_y=None, skip_static=False):
    """state: 'reach' (pin hooked in the basket at shelf), 'pulled' (basket on tray), 'empty' (no basket on tray,
    basket still on shelf).  raise_ lifts the tray above the shelf (approach); T overrides tray height."""
    items = []  # (name, Solid, color)
    add = lambda n, s, c: items.append((n, s, c))
    zl = LEVELS[level_idx]
    if T is None:
        T = zl + raise_
    if pin_y is None:
        pin_y = {"reach": BASKET_Y0 + 15, "pulled": -178.0, "loaded": -207.0, "pushed": BASKET_Y0 - 5}.get(state, -178.0)
    Pc = T + T_OFF

    # ---------------- rack (slotted angle) ----------------
    ang = []
    for sx in (-1, 1):
        S = M_mirror_x() if sx < 0 else np.eye(4)
        frame = [box(XI, XI + ANG_T, 0, ANG, 0, RACK_H), box(XI, XI + ANG, 0, ANG_T, 0, RACK_H),          # front post
                 box(XI, XI + ANG_T, RACK_D - ANG, RACK_D, 0, RACK_H), box(XI + ANG_T - ANG, XI + ANG_T, RACK_D - ANG_T, RACK_D, 0, RACK_H)]
        for z in LEVELS + [RACK_H]:
            frame += [box(XI - ANG_T, XI, 0, RACK_D, z - ANG, z), box(XI - ANG, XI, 0, RACK_D, z - ANG_T, z)]
        for s_ in frame: ang.append(s_.transform(S))
        # ledge bolts at front post = inductive targets
        for z in LEVELS:
            bolt = [cyl(4.3, XI - 5.5, XI - ANG_T, 'x', (19, z - 19), 6), cyl(2.5, XI - 2, XI + 9, 'x', (19, z - 19), 10),
                    hexagon(8, XI + ANG_T, XI + 6.5, (19, z - 19), 'x')]
            for b in bolt: add("ledge bolt (sensor target)", b.transform(S), C_BOLT)
    ang += [box(-XI - ANG_T, XI + ANG_T, RACK_D, RACK_D + ANG_T, 0, ANG), box(-XI - ANG_T, XI + ANG_T, RACK_D, RACK_D + ANG_T, RACK_H - ANG, RACK_H),
            box(-XI - ANG, XI + ANG, -ANG_T, 0, 0, ANG)]
    add("slotted angle rack", Solid(sum([a.polys for a in ang], [])), C_ANGLE)

    # baskets
    for i, z in enumerate(LEVELS):
        if i == level_idx and state in ("pulled", "loaded"):
            continue
        add(f"basket {i+1}", basket(BASKET_Y0, z), BASKET_COLS[i])
    if state in ("pulled", "loaded"):
        add(f"basket {level_idx+1} (on tray)", basket(-193.0 + 0.0, T, front_at=-193.0 + 0), BASKET_COLS[level_idx])

    # ---------------- rails ----------------
    for sx in (-1, 1):
        S = side_M(sx, 0)
        add("V-slot 2020 rail", vslot(-10, 10, RAIL_Y0, RAIL_Y1, 0, RAIL_LEN).transform(S), C_RAIL)
        for k in range(N_SEG):
            add("printed rack segment", part("rack_segment").transform(S @ M_T(0, 0, RACK_Z0 + k * SEG_LEN)), C_PRINT)
        # mounts: bottom (as drawn) and top (mirrored in z)
        add("printed rail mount", part("rail_mount").transform(S @ M_T(0, 0, 40)), C_PRINT)
        add("printed rail mount", part("rail_mount").transform(S @ M_T(0, 0, 999.5) @ M_mirror_z()), C_PRINT)
        # limit switches (bottom both sides, top on left)
        add("limit switch", microswitch().transform(S @ M_T(0, 0, 40)), C_SW)
        if sx < 0:
            add("limit switch", microswitch().transform(S @ M_T(0, 0, 999.5) @ M_mirror_z()), C_SW)

        # ---------------- carriage ----------------
        C = side_M(sx, Pc)
        add("gantry plate", box(-PLATE_W / 2, PLATE_W / 2, PLATE_Y0, PLATE_Y1, -PLATE_W / 2, PLATE_W / 2).transform(C), C_PLATE)
        for ux in (-1, 1):
            for wz in (-1, 1):
                c = (ux * WHEEL_DX, wz * WHEEL_DZ)
                add("V wheel", cyl(WHEEL_R, WHEEL_YC - WHEEL_W / 2, WHEEL_YC + WHEEL_W / 2, 'y', c, 28).transform(C), C_WHEEL)
                add("spacer", cyl(5, PLATE_Y1, WHEEL_YC - WHEEL_W / 2, 'y', c, 12).transform(C), C_STEEL)
                add("axle screw", cyl(4.5, PLATE_Y0 - 3, PLATE_Y0, 'y', c, 12).transform(C), C_BOLT)
        add("printed carriage bracket", part("carriage_bracket").transform(C), C_PRINT)
        add("printed deck hanger", part("deck_hanger").transform(C), C_PRINT)
        add("printed pinion", part("pinion").transform(C @ M_T(PIN_U0, PIN_Y, 0) @ rot('y', 90)), C_PRINT2)
        # JGY-370 + steel L bracket (approximate)
        y0, y1 = PIN_Y - GB_Y / 2, PIN_Y + GB_Y / 2
        gb_w0 = -GB_SHAFT_FROM_BOTTOM
        add("steel L-bracket", Solid(box(20, 22, y0, y1, BRACKET_BASE_W, 18).polys + box(20, 50, y0, y1, BRACKET_BASE_W, BRACKET_BASE_W + 2).polys).transform(C), C_STEEL)
        add("JGY-370 gearbox", box(GB_U0, GB_U0 + GB_T, y0, y1, gb_w0, gb_w0 + GB_Z).transform(C), C_GB)
        add("JGY-370 motor", cyl(MOTOR_D / 2, gb_w0 + GB_Z, gb_w0 + GB_Z + MOTOR_L, 'z', (GB_U0 + GB_T / 2, PIN_Y), 24).transform(C), C_CAN)
        add("motor shaft", cyl(3, PIN_U0 + 1, 22, 'x', (PIN_Y, 0), 12).transform(C), C_STEEL)
        # inductive sensor LJ12A3-4
        add("LJ12A3-4 inductive sensor", cyl(6, -10, 46, 'x', (19, -59), 20).transform(C), C_IND)
        for uu in (24.0, 34.5):
            add("M12 nut", hexagon(17, uu, uu + 3.5, (19, -59), 'x').transform(C), C_STEEL)

    # ---------------- tray ----------------
    add("plywood deck 9 mm", box(-DECK_X, DECK_X, DECK_Y1, DECK_Y0, T - DECK_T, T), C_WOOD)
    st = []
    for sx in (-1, 1):
        S = M_mirror_x() if sx < 0 else np.eye(4)
        st += [box(60, 98, DECK_Y1 + 5, DECK_Y0 - 5, T - DECK_T - ANG_T, T - DECK_T).transform(S),
               box(96, 98, DECK_Y1 + 5, DECK_Y0 - 5, T - DECK_T - ANG, T - DECK_T).transform(S)]
    add("angle-iron deck stiffeners", Solid(sum([s.polys for s in st], [])), C_ANGLE)
    add("printed shoulder tower", part("shoulder_tower").transform(M_T(0, 0, T)), C_PRINT)
    bc = SHOULDER[1] + SV_L / 2 - SV_SPLINE_OFF
    add("printed cam/ToF bracket", part("cam_tof_bracket").transform(M_T(0, bc + 30, T)), C_PRINT)
    add("ESP32-CAM", box(-13.5, 13.5, bc + 35, bc + 39, T + 72.5, T + 113).union(box(-4, 4, bc + 39, bc + 43, T + 99, T + 107)), C_PCB)
    add("camera lens", cyl(3.5, bc + 43, bc + 47, 'y', (0, T + 103), 16), C_SW)
    add("VL53L0X (basket)", box(-12.5, 12.5, bc + 35, bc + 36.5, T + 34.5, T + 47.5), C_TOF)
    add("printed basket stop", part("basket_stop").transform(M_T(-80, -193, T)), C_PRINT)
    add("printed basket stop", part("basket_stop").transform(M_T(80, -193, T)), C_PRINT)
    add("limit switch (basket on tray)", box(-10, 10, -193, -186.6, T + 13, T + 23), C_SW)
    add("printed ToF mount", part("tof_down_mount").transform(M_T(-120, -312, T - DECK_T) @ rot('x', 180)), C_PRINT)
    add("VL53L0X (height)", box(-132.5, -107.5, -318, -305, T - DECK_T - 13.5, T - DECK_T - 12), C_TOF)
    add("MPU-6050", box(-10, 10, -260, -244, T - DECK_T - 4, T - DECK_T - 2.5), C_PCB)

    # ---------------- SCARA arm ----------------
    q1, E, q2 = ik(pin_y)
    b, h = servo_model(SHOULDER, 90, T + UPPER_Z - SV_HORN_TOP)
    add("20kg servo (shoulder)", b, C_SERVO); add("servo horn (shoulder)", h, C_STEEL)
    add("printed upper arm", part("upper_arm").transform(M_T(SHOULDER[0], SHOULDER[1], T + UPPER_Z) @ rot('z', math.degrees(q1))), C_PRINT)
    back = math.degrees(math.atan2(SHOULDER[1] - E[1], SHOULDER[0] - E[0]))
    b, h = servo_model(E, back, T + UPPER_Z + SV_TAB_Z, inverted=True)
    add("20kg servo (elbow)", b, C_SERVO); add("servo horn (elbow)", h, C_STEEL)
    add("printed forearm + hook pin", part("forearm").transform(M_T(E[0], E[1], T + UPPER_Z - (SV_HORN_TOP - SV_TAB_Z) - ARM_T) @ rot('z', math.degrees(q2))), C_PRINT)
    return items, dict(Pc=Pc, T=T, E=E, q1=q1, q2=q2, pin_y=pin_y)


def basket(front_y, z, front_at=None):
    y0 = front_y; y1 = y0 + BASKET_D; x = BASKET_W / 2; t = 3
    walls = [box(-x, x, y0, y1, z, z + t), box(-x, -x + t, y0, y1, z, z + BASKET_H), box(x - t, x, y0, y1, z, z + BASKET_H),
             box(-x, x, y0, y0 + t, z, z + BASKET_H), box(-x, x, y1 - t, y1, z, z + BASKET_H)]
    return Solid(sum([w.polys for w in walls], []))


def microswitch():
    """KW11 roller switch on a rail-mount shelf (rail-mount local frame)"""
    zb = P.SHELF_Z + 6
    body = box(14, 34, -66, -59.6, zb, zb + 10)
    lever = box(16, 30, -63.5, -62, zb + 10, zb + 12)
    roller = cyl(2.4, 28, 32, 'x', (-62.8, zb + 13.5), 10)
    return Solid(body.polys + lever.polys + roller.polys)
