import sys, os, json; sys.path.insert(0,'.')
import numpy as np
import parts as P
from csg import to_mesh, mesh_stats, write_stl_mesh, rot, Solid
OUT = "out/stl"; os.makedirs(OUT, exist_ok=True)
MX = np.diag([-1.0,1,1,1])
I = np.eye(4)
# name, builder, print transform, qty, mirror-variant name (or None), note
PARTS = [
 ("01_rack_segment_R", "rack_segment", rot("y",90), 5, "01_rack_segment_L", "Gear rack m1.5, 160 mm. 5 per rail. Print teeth-face down."),
 ("02_pinion_m1.5_14T", "pinion", I, 2, None, "14-tooth pinion, 6 mm D-bore for JGY-370."),
 ("03_rail_mount_A", "rail_mount", I, 2, "03_rail_mount_B", "Rail stand-off + limit-switch shelf. A: bottom-right & top-left (rotated). B: bottom-left & top-right."),
 ("04_carriage_bracket_R", "carriage_bracket", rot("y",90), 1, "04_carriage_bracket_L", "Bolts to gantry plate; holds motor L-bracket and LJ12A3 sensor."),
 ("05_deck_hanger_R", "deck_hanger", rot("y",90), 1, "05_deck_hanger_L", "Hangs the plywood deck under the carriage."),
 ("06_wheel_spacer_6mm", "wheel_spacer", I, 4, None, "For the 4 concentric wheels (buy aluminium ones if you can)."),
 ("07_gantry_drill_jig", "drill_jig", rot("x",180), 1, None, "Locates on wheel holes; drill 4 x 5.2 mm holes in each gantry plate."),
 ("08_shoulder_tower", "shoulder_tower", I, 1, None, "Holds the shoulder servo on the deck."),
 ("09_upper_arm", "upper_arm", I, 1, None, "SCARA link 1 (180 mm), carries the elbow servo upside-down."),
 ("10_forearm_hook", "forearm", rot("x",180), 1, None, "SCARA link 2 (180 mm) with hook pin."),
 ("11_basket_stop", "basket_stop", I, 2, None, "Tray end stops; one carries the basket-present switch."),
 ("12_cam_tof_bracket", "cam_tof_bracket", rot("x",90), 1, None, "ESP32-CAM cradle + VL53L0X pocket on the tower front."),
 ("13_tof_down_mount", "tof_down_mount", I, 1, None, "VL53L0X looking at the floor (absolute height)."),
]
info = []
for name, fn, tf, qty, mname, note in PARTS:
    V0, F0 = to_mesh(getattr(P, fn)())
    def place(M):
        V = V0 @ M[:3,:3].T
        F = F0 if np.linalg.det(M[:3,:3]) > 0 else F0[:, ::-1]
        lo, hi = V.min(0), V.max(0)
        return V - np.array([(lo[0]+hi[0])/2, (lo[1]+hi[1])/2, lo[2]]), F
    variants = [(name, place(tf))]
    if mname: variants.append((mname, place(MX[:4,:4] @ tf)))
    for nm, (V, F) in variants:
        st = mesh_stats(V, F)
        write_stl_mesh(V, F, f"{OUT}/{nm}.stl", nm.encode())
        lo, hi = V.min(0), V.max(0); size = (hi-lo)
        vol = np.sum(np.einsum('ij,ij->i', V[F[:,0]], np.cross(V[F[:,1]], V[F[:,2]])))/6
        info.append(dict(file=nm+".stl", qty=qty, size=[round(x,1) for x in size], vol_cm3=round(vol/1000,1),
                         grams_100pct=round(vol/1000*1.24,1), fits_220=bool(size[0]<=218 and size[1]<=218 and size[2]<=245), note=note, **st))
        print(nm, info[-1])
json.dump(info, open("out/parts_info.json","w"), indent=1)
