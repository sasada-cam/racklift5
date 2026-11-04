"""Load the ORIGINAL RackLift5 v2 part / assembly code on top of the B-rep csg backend.
import racklift_brep as R  ->  R.P1 (parts.py), R.P2 (parts_v2.py), R.A (assembly_v2.py), R.params
"""
import os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.abspath(os.path.join(HERE, "..", "..", "generator"))
for p in (SRC, HERE):                 # HERE first -> our csg.py shadows the mesh one
    if p in sys.path:
        sys.path.remove(p)
sys.path.insert(0, SRC)
sys.path.insert(0, HERE)

import csg                            # noqa: E402  (B-rep backend)
assert os.path.dirname(os.path.abspath(csg.__file__)) == HERE, "wrong csg module picked up"
import params                         # noqa: E402
import parts as P1                    # noqa: E402
import gears                          # noqa: E402
P1.involute_profile = gears.tagged_involute_profile(P1.involute_profile)
import parts_v2 as P2                 # noqa: E402  (imports the patched involute_profile)
import assembly_v2 as A               # noqa: E402

# printable parts: name -> (builder, print-orientation matrix, qty, group, note)  (mirrors export_v2.py)
import numpy as np                    # noqa: E402
from csg import rot                   # noqa: E402
I = np.eye(4); RX = lambda d: rot('x', d); RY = lambda d: rot('y', d)
PRINTED = {
    "R01_chassis": (P2.chassis, RX(-90), 1, "Robot"),
    "R02_climber_yoke": (P2.climber_yoke, RY(-90), 1, "Robot"),
    "R03_climber_pinion_14T": (P2.climber_pinion, I, 2, "Robot"),
    "R04_turn_pinion_12T": (P2.turn_pinion, I, 1, "Robot"),
    "R05_hand_tray_arm_48T": (P2.tray_arm, RX(90), 1, "Robot"),
    "R06_fork_crown": (P2.crown, RX(90), 1, "Robot"),
    "R07_tray_foot": (P2.tray_foot, RX(180), 1, "Robot"),
    "R08_wheel_spacer_6mm": (P1.wheel_spacer, I, 4, "Robot"),
    "R09_gantry_drill_jig": (P1.drill_jig, RX(180), 1, "Robot"),
    "R10_hand_axle_spacer": (P2.axle_spacer, RX(90), 1, "Robot"),
    "T01_shoulder_tower": (P1.shoulder_tower, I, 1, "Tray"),
    "T02_upper_arm": (P1.upper_arm, I, 1, "Tray"),
    "T03_forearm_hook": (P1.forearm, RX(180), 1, "Tray"),
    "T04_basket_stop": (P1.basket_stop, I, 2, "Tray"),
    "T05_cam_tof_bracket": (P1.cam_tof_bracket, RX(90), 1, "Tray"),
    "T06_tof_down_mount": (P1.tof_down_mount, I, 1, "Tray"),
    "J01_turntable_rotor": (P2.rotor, RX(90), 2, "Junction"),
    "J02_detent_spring": (P2.detent, RX(90), 4, "Junction"),
    "J03_torque_pin_sleeve": (P2.pin_sleeve, I, 4, "Junction"),
    "J04_stub_block": (P2.stub_block, I, 2, "Junction"),
    "J05_base_spacer_22": (P2.base_spacer, I, 10, "Junction"),
    "J06_rail_bridge": (P2.rail_bridge, RX(-90), 4, "Junction"),
}
for k, (a0, a1, q) in P2.RACK_STRIPS.items():
    PRINTED["K_" + k] = ((lambda a0=a0, a1=a1: P2.rack_strip(a0, a1)), RX(-90), q, "Rack")
