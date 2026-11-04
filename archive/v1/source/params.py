"""All dimensions in mm.  GLOBAL FRAME: x = right, y = into the rack (front face of rack at y=0),
z = up from the floor.  Edit these values to match the parts you actually bought, then re-run build.py.
Values marked  # MEASURE  are typical catalogue sizes - check them with a caliper.
"""
import math

# ---------------- slotted angle iron rack ----------------
ANG = 38.0            # angle iron leg width            # MEASURE
ANG_T = 2.0           # angle iron thickness            # MEASURE
XI = 153.0            # inner face of post side-flange (clear opening = 2*XI = 306)
RACK_D = 230.0        # rack depth (front y=0 .. back)
RACK_H = 1000.0       # post height (1 m pieces)
LEVELS = [100.0, 280.0, 460.0, 640.0, 820.0]   # top surface of each shelf ledge (5 steps)

# ---------------- basket (Daraz small foldable basket) ----------------
BASKET_W, BASKET_D, BASKET_H = 260.0, 180.0, 110.0   # MEASURE yours
BASKET_Y0 = 40.0      # basket front wall y when stored on a shelf

# ---------------- V-slot rail ----------------
XR = XI + ANG / 2     # rail centre x (= centre of the post front flange) -> 172
RAIL_Y0, RAIL_Y1 = -60.0, -40.0   # rail front / back face y
RAIL_LEN = 1000.0
SLOT_W = 6.2

# gantry plate / wheels
PLATE_W, PLATE_T = 65.0, 6.0      # MEASURE (OpenBuilds mini V gantry plate 20mm)
PLATE_Y1 = -61.0                  # back face of plate
PLATE_Y0 = PLATE_Y1 - PLATE_T     # front face of plate (-67)
WHEEL_DX = 19.8                   # wheel centre offset from rail centre (x)
WHEEL_DZ = 20.0                   # wheel centre offset from plate centre (z)
WHEEL_R, WHEEL_W = 12.2, 10.2
WHEEL_YC = (RAIL_Y0 + RAIL_Y1) / 2
PLATE_HOLES = [(sx * 28.5, sz * 8.0) for sx in (-1, 1) for sz in (-1, 1)]  # 4 x Ø5.2 to drill (u, w)

# ---------------- rack & pinion ----------------
MOD = 1.5
PIN_T = 14
PA = 20.0
PITCH = math.pi * MOD                 # 4.712
PIN_R = MOD * PIN_T / 2               # 10.5 pitch radius
ADD, DED = 1.0 * MOD, 1.25 * MOD
BACKLASH = 0.15                       # printed-gear clearance (each side)
SEG_TEETH = 34
SEG_LEN = SEG_TEETH * PITCH           # 160.22
N_SEG = 5
RACK_Z0 = 120.0                       # bottom of first rack segment
RACK_ROOT_Y = -32.0                   # tooth root line (teeth point toward +y)
RACK_PITCH_Y = RACK_ROOT_Y + DED      # -30.125
PIN_Y = RACK_PITCH_Y + PIN_R          # pinion axis y (-19.625)
TEETH_U0, TEETH_U1 = 6.0, 19.0        # tooth band across the rack (u = outward from rail centre)
PIN_U0, PIN_U1 = 6.5, 18.5            # pinion face (12 mm)

# ---------------- lift / tray ----------------
T_OFF = 40.0                          # tray top = plate centre - T_OFF
APPROACH = 40.0                       # lift raise used to hook the basket
DECK_T = 9.0                          # plywood deck thickness
DECK_X = 145.0                        # deck half width
DECK_Y0, DECK_Y1 = -3.0, -330.0
PC_HOME = 135.0                       # plate-centre height at homing switch

# ---------------- JGY-370 + steel L bracket (approx) ----------------
GB_U0 = 22.5                          # gearbox face (u) behind the L-bracket plate
GB_T, GB_Y, GB_Z = 22.5, 32.0, 46.0   # gearbox: thickness along shaft, width (y), height (z)   # MEASURE
GB_SHAFT_FROM_BOTTOM = 10.0           # shaft distance from bottom of gearbox           # MEASURE
MOTOR_D, MOTOR_L = 24.4, 33.0
LBR_T = 2.0
SHAFT_D, SHAFT_FLAT = 6.0, 5.5        # D-shaft                                          # MEASURE
BRACKET_BASE_W = -12.0                # w of underside of the steel L-bracket base (rel. plate centre)

# ---------------- servos (20 kg, 25T) ----------------
SV_L, SV_W, SV_H = 40.5, 20.2, 38.0   # body                                             # MEASURE
SV_TAB_L, SV_TAB_T, SV_TAB_Z = 54.5, 2.5, 26.0   # tab span, thickness, tab underside height from body bottom
SV_HOLE_DX, SV_HOLE_DY = 49.5, 10.0
SV_SPLINE_OFF = 10.25                 # spline axis from the near end of the body
SV_HORN_TOP = 47.0                    # top of metal disc horn above body bottom
HORN_PCD, HORN_D = 14.0, 25.0         # horn bolt circle (4 x M3) and disc diameter      # MEASURE

# ---------------- arm (2-link SCARA) ----------------
L1 = L2 = 180.0
SHOULDER = (0.0, -290.0)
UPPER_Z = 148.0                       # upper-arm underside above tray top
ARM_T = 8.0
PIN_LEN = 39.0
PIN_DIA = 8.0

# hardware
M5_CLR, M3_CLR = 5.4, 3.3
M5_HEAD_D = 10.0
