"""RackLift5 v3 (concept): compact Squid layout on the v2 rack.

Front view of one bay:                          Top view:
        ||  vertical rail (one per bay)           ======+=============  rails (y -60..-40)
   [H1]=++=============  upper horizontal rail      [bat]|col| +--------------+
    |   ||   [tray]                                          | ||  basket  || |   || = telescopic arms
 [bat]=col=                                                  +--------------+       with flip fingers
   [H2]=++=============  lower horizontal rail
* Travel on the two horizontal rails (one hub on each), climb on the single vertical rail (both hubs).
* Each rail crossing has a small turntable piece on a 50 mm bearing; a hub swivels 90 deg with its piece, one at a time.
* The body never rotates: the tray is always level.
* Layout: rows of racks (floor plan), rails on each row's front; a transfer cart at the rows' right ends carries the
  robot from row to row, pushed by the robot's own pinion along a fixed gear rack.
* Picker: two 2-stage telescopic arms (stacked bars, belt-doubled like warehouse shuttle load handlers) in the 23 mm gaps
  beside the basket, each with a front and a rear flip finger that capture the basket -> pulls it and pushes it back.
  Two servos drive everything: servo 1 the arm stroke, servo 2 all four fingers.
* Battery + electronics: a fixed box beside the column, opposite the tray, close to the rails (passive counterweight).
* A load cell under the tray bracket weighs the payload (overload check).
Same global frame as v2 (x right, y into the rack, z up, mm).  See docs/design/001-load-carrying-single-vs-two-rail.md, 8h.
"""
from params import *  # noqa: F401,F403

# ---------------- rails
JZ_LO = 70.0                   # lower horizontal rail (v2 had it at 140)
D_HUB = 260.0                  # hub spacing = spacing of the two horizontal rails
JZ_UP = JZ_LO + D_HUB          # upper horizontal rail z 330 (below shelf 2 at 353)
CROSS_Z = (JZ_LO, JZ_UP)
SEG_V3 = 58.0                  # crossing turntable piece +/-58 (the hub's wheels span +/-45)
GAP_V3 = 2.0
RAIL_TOP_V3 = 1080.0           # vertical rails 80 mm above the rack (upper hub at the top shelf)
BEARING_R = 25.0               # 50 mm lazy-susan bearing behind each crossing piece

# ---------------- robot (C = centre between the hubs)
HUB_WHEEL_A = 32.5
HUB_PLATE_L = 90.0
CP_OUT = -20.0
HUB_R0, HUB_R1 = 44.0, 47.0
HUB_Y = (-82.0, -70.0)
COL_Y = (-120.0, -96.0)        # column in front of the hubs (hubs swivel behind it, y > -94, radius ~56)
CZ_HOME = JZ_LO + D_HUB / 2    # 200: travel height; shelf 1 is picked here
T_V3 = LEVELS[0] - CZ_HOME     # tray top relative to C (-22)

# ---------------- tray + telescopic arms (2 servos drive everything)
BXR = RAIL_OFF                 # bay centre relative to the column
TRAY_Y = (-294.0, -70.0)       # tray stays in front of the rails (y < -60); front strip carries the drive posts
BASKET_TRAY_Y0 = -252.0        # near wall of a basket on the tray (-252..-72)
ARM_X = (133.0, 146.0)         # arm in the gap between the basket side (130) and the rack post (153), from the bay centre
ARM_W = ARM_X[1] - ARM_X[0]    # 13; lanes across an arm are measured with u from its inner face (basket side)
# three 13 x 8 mm bars stacked on top of each other, sliding on PTFE pads (all moving parts stay visible):
OUT_Z, MID_Z, IN_Z = (35.0, 43.0), (47.0, 55.0), (59.0, 67.0)   # above the tray top / the shelf
ARM_Z = (OUT_Z[0], IN_Z[1])
ARM_L = 180.0                  # each bar 180 mm (= basket depth); outer bar fixed on the tray at y -255..-75
TIP0 = TRAY_Y[1] - 5.0         # inner bar tip when stowed (-75)
ARM_Y0 = TIP0 - ARM_L
# stage 1: servo 1 -> GT2 step-up belt (25T -> 8T) -> cross shaft -> a 33T pulley + belt loop on the outer side of each
#          arm; the middle bar is clamped to the loop's top run
# stage 2: two pulleys on the middle bar; the 'extend' belt is anchored to the outer bar's rear and the inner bar's rear,
#          the 'retract' belt to their fronts -> the inner bar moves twice as far as the middle bar (2:1)
LANE_EXT, LANE_RET, LANE_S1 = (2.5, 6.0), (7.0, 10.5), (13.5, 17.5)   # u ranges: stage-2 belts, stage-1 belt (outside)
DPUL_R = 6.75                  # stage-2 pulley pitch radius (belt runs at MID centre +- this, in the 4 mm gaps)
DPUL_Y = (TIP0 - 9.0, ARM_Y0 + 9.0)   # extend pulley (front end of the middle bar), retract pulley (rear end), stowed
S1_Y = (-303.0, -78.0)         # stage-1 drive pulley (on the cross shaft) and idler, zt 22
S1_ZT = 22.0
STEP_T = (25, 8)               # servo pulley / cross-shaft pulley teeth (3.1:1 step-up)
SERVO1_AT = (-318.0, -36.0)    # servo 1 output (y, zt), under the tray front
SERVO1_X = -60.0               # step-up belt plane (x from the bay centre)
# fingers: each arm carries a telescoping square shaft (fixed rod / middle tube / inner tube) above the bars; servo 2 turns
# the left rod through a spur-gear pair and the right rod through a GT2 belt, so the two sides turn opposite ways (inward)
FS_ZT = 77.0                   # finger shaft axis above the tray top
FS_SQ = (5.0, 8.0, 11.0)       # rod / middle tube / inner tube (square, mm)
FINGER_L, FINGER_T, FINGER_W = 32.0, 4.0, 8.0
REAR_OFF = BASKET_D + FINGER_T + 2.0            # 186: rear finger's face is this far behind the tip -> 2 mm capture play
FRONT_Y = (-294.0, -288.0)     # front posts (rod bearing + cross-shaft bearing)
FG_Y, FP_Y = (-281.0, -277.0), (-287.0, -283.0) # finger gear plane, finger belt plane
FT_MID_Y = (-275.0, TIP0 - 2.0)   # middle square tube, stowed
FSERVO_DX = 21.0               # servo 2 output: this far inward of the left rod (gear centre distance)
# pick sequence: both fingers drop together and capture the basket between them (front behind the far wall, rear in
# front of the near wall); retracting pulls it, extending pushes it.
TIP_SHELF = BASKET_Y0 + BASKET_D + 1.0 + FINGER_T   # 225: front finger 1 mm behind the far wall (rack flange at 228)
TIP_PUSHED = BASKET_Y0 + REAR_OFF                   # 226: rear finger pushed the basket back to y 40
TIP_PULLED = BASKET_TRAY_Y0 + BASKET_D + FINGER_T   # -68: basket pulled against the tray stops
TIP_TRAY = TIP_PULLED + 1.0                         # -67: fingers drop around a basket on the tray
TIP_PULL = TIP_SHELF                                # (old name)

# ---------------- battery box (fixed counterweight) + load cell
BAT_MASS = 1.0                 # kg (ESTIMATE)
BAT_X = (-170.0, -70.0)
BAT_Y = (-160.0, -100.0)
BAT_Z = (-45.0, 45.0)


def cz_for_level(l):
    return LEVELS[l] - T_V3


# ---------------- the robot's pinion drive (moves the transfer cart; no motor off the robot)
RAIL_Y = -50.0                         # horizontal rail centre line (y)
TURN_PIN_T = 16                        # 16T, module 1.5 -> pitch radius 12
TURN_PIN_R = 1.5 * TURN_PIN_T / 2
TURN_GEAR_Z = (182.0, 190.0)           # pinion plane above C (above the upper hub's cover, C+177)
TURN_PIN_XY = (60.0, -102.5)           # pinion axis in the robot frame (on a mast above the column)


# ---------------- floor of racks (top view): rows of racks, rails along each row's front, side rails between rows
# All rows face the same way (front at the row's y = 0, rack behind it), ROW_P apart.  The robot snakes through them:
# row 1 -> right side rail -> row 2 -> left side rail -> row 3 (a single line, so every passive turret always waits on
# the robot's side, and every joint is an L corner the robot turns itself at - none it has to drive straight past).
N_ROWS = 3
ROW_BAYS = 4                                   # racks (bays) per row
BAY_P = BAY_X[1] - BAY_X[0]                    # 438: bay pitch along a row (as v2)
ROW_BAY_X = [i * BAY_P for i in range(ROW_BAYS)]
DX_LAST = ROW_BAY_X[-1]                        # last bay of a row (its right corner turret is relative to it)
ROW_P = 650.0                                  # row pitch: 230 rack + 420 aisle (robot 340 deep + turning room)
N_BAYS_ALL = N_ROWS * ROW_BAYS
# The rows are joined by a TRANSFER CART at their right ends (the red joints of the sketch): the robot drives off the
# end of its row onto the cart's two short rail pieces, the cart slides along the side track (the xy-plane side bar)
# to the next row's rail ends, and the robot drives on.  No turning, all rows face the same way.  The cart has no motor:
# the robot's own pinion (the turn drive) drops into a fixed straight gear rack along the track and pushes robot + cart.
# The cart is always at the robot's row (it is the only way between rows), so it never waits on the wrong side.
X_CART = DX_LAST + 371.0                       # robot column on the cart (its battery then clears the last rack, x 191)
CART_PIECE = (X_CART - SEG_V3, X_CART + SEG_V3)
ROW_RAIL_X = (-300.0, CART_PIECE[0] - 2.0)     # fixed horizontal rails of a row (row frame x)
TRACK_Y = (-420.0, (N_ROWS - 1) * ROW_P + 120.0)   # cart track / transfer rack extent (world y)
TURN_LIFT = 12.0                               # an SG90 lifts the robot's pinion this much (clear of the transfer rack it
#                                                drives over); it drops into mesh only on the cart
XFER_RACK_X = X_CART + TURN_PIN_XY[0] + TURN_PIN_R   # pitch line of the fixed transfer rack (teeth face -x)
XFER_POST_X = X_CART + BXR + ARM_X[1] + 44.0     # rack-beam posts stand beyond the robot's tray end (x 376 from C)
