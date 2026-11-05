"""RackLift5 v3 (concept): Squid-style robot on the v2 rack and rails.

Robot (front view):
                 [hub 1]           each hub = wheels + climb pinion + climb motor, swivels 90 deg on its leg end
                    | leg 1        (vertical rail <-> horizontal rail)
   [counterweight]==O==========[ tray ]    O = centre joint: both legs turn about it, each with its own motor;
                    | leg 2                    the beam (tray + counterweight + slide) always stays level
                 [hub 2]
Junction: one hub is always clamped on a fixed rail while the other hub swivels together with the v2 turntable piece.
Picker: a linear slide with a spring-loaded hook pin (rides over basket walls) and a lock servo -> pull AND push back.

Rails: v2, plus (1) vertical rails 90 mm longer at the top (upper hub at the top shelf), (2) a 300 mm horizontal stub
past junction 2 (the robot's centre must stay right of a junction, so the right hub needs rail there), (3) a small
servo per junction to set the empty turntable piece.  Same global frame as v2 (x right, y into the rack, z up, mm).
See docs/design/001-load-carrying-single-vs-two-rail.md, section 8e.
"""
import math
from params import *  # noqa: F401,F403

# ---------------- legs, hubs
L_LEG = 150.0                   # centre joint -> hub centre
HUB_WHEEL_A = 32.5              # wheels at hub +/-32.5 along the rail
HUB_PLATE_L = 90.0
CP_OUT = -10.0                  # climb pinion along the rail from the hub centre
HUB_R0, HUB_R1 = 52.0, 56.0     # printed hub cover (tube) radii, axis = y
HUB_Y = (-102.0, -70.0)         # hub cover y range (in front of the plates)
LEG_Y = {1: (-114.0, -106.0), 2: (-126.0, -118.0)}   # each leg in its own y layer (they cross at the centre joint)
GEAR_R = 46.5                   # 60T gear on each leg at the centre joint, driven by a 12T pinion (5:1)
PINION_ANG = {1: 30.0, 2: 120.0}    # pinion positions (deg) around the centre joint, outside each leg's swing range
SWING = {1: (88.0, 224.0), 2: (-92.0, -40.0)}         # leg angles used (deg); pinions sit outside these

# ---------------- level beam: tray on the right (bay side), counterweight on the left
T_V3 = -10.0                    # tray top relative to the centre joint (tray hangs just below it)
BEAM_Y = (-175.0, -155.0)           # in front of the swivel motors at the leg ends
CW_MASS = 1.5                   # kg: battery + electronics + ballast (ESTIMATE; set from the real battery)
CW_X = (-320.0, -200.0)         # counterweight box x range relative to the centre joint (centre 260 mm out)
CW_Y = (-200.0, -130.0)
CW_Z = (-25.0, 45.0)

# ---------------- linear slide + spring-loaded hook pin (replaces the SCARA arm)
SLIDE_Y = (-320.0, -10.0)       # 20x20 slide rail along y, above the tray
SLIDE_Z = 150.0                 # rail underside above the tray top
PIN_ARM = 70.0                  # pin axis this far in front of the carriage's front face
CARR_L = 40.0                   # carriage length along y
PIN_DOWN = 80.0                 # pin tip above the tray top when down (basket rim at 110)
PIN_LIFT = 35.0                 # lift needed to ride over a basket wall
PIN_STOW = -178.0               # inside a basket on the tray, 15 mm behind its near wall
PIN_PULL = BASKET_Y0 + 15.0     # inside a basket on the shelf
PIN_OUT = -205.0                # outside (robot side of) a basket on the tray
PIN_PUSH = BASKET_Y0 - 12.0     # touching the near wall from outside, basket fully on the shelf

# ---------------- rails (v2 + additions)
RAIL_TOP_V3 = 1090.0
X_STUB_END = JUNCTIONS[-1][0] + 300.0

# ---------------- poses
TRAVEL_HALF = 112.0             # travel pose: hubs at centre +/-112 on the horizontal rail ...
TRAVEL_H = math.sqrt(L_LEG ** 2 - TRAVEL_HALF ** 2)   # ... centre joint 100 mm above the rail
WALK = [(0.0, 2 * TRAVEL_HALF), (210.0, 200.0), (2 * L_LEG, 0.0)]   # junction walk keyframes (a = hub 1 up the vertical
#   rail, b = hub 2 along the horizontal rail, from the junction centre); between 2 and 3 the junction servo sets the piece


def cz_for_level(l):
    """centre-joint height to pick from shelf l"""
    return LEVELS[l] - T_V3


def walk_state(jx, jz, a, b):
    """hub 1 at (jx, jz + a) on the vertical rail, hub 2 at (jx + b, jz) on the horizontal rail ->
    centre joint (cx, cz) and leg angles (deg).  Centre on the bay side (+x) of the hub line."""
    d = math.hypot(a, b)
    k = math.sqrt(max(0.0, L_LEG ** 2 - d * d / 4))
    nx, nz = (a / d, b / d) if d > 1e-9 else (1.0, 0.0)
    cx, cz = b / 2 + nx * k, a / 2 + nz * k
    a1 = math.degrees(math.atan2(a - cz, 0.0 - cx)) % 360
    a2 = math.degrees(math.atan2(0.0 - cz, b - cx))
    return dict(cx=jx + cx, cz=jz + cz, a1=a1, a2=a2)


def vertical_state(jx, cz):
    return dict(cx=jx, cz=cz, a1=90.0, a2=-90.0)


def travel_state(cx):
    return walk_state(cx - TRAVEL_HALF, JZ, 0.0, 2 * TRAVEL_HALF)


def hub_pos(s, i):
    a = math.radians(s["a%d" % i])
    return s["cx"] + L_LEG * math.cos(a), s["cz"] + L_LEG * math.sin(a)
