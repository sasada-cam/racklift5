import sys, os, json; sys.path.insert(0,'.')
import numpy as np
import parts as P1, parts_v2 as P2
from csg import to_mesh, mesh_stats, write_stl_mesh, rot
OUT="out/stl"; os.makedirs(OUT, exist_ok=True)
for f in os.listdir(OUT): os.remove(os.path.join(OUT,f))
I=np.eye(4)
RX=lambda d: rot('x',d); RY=lambda d: rot('y',d)
PARTS=[
 # group, file, builder, print-transform, qty, note
 ("Robot","R01_chassis", lambda: P2.chassis(), RX(-90), 1, "Back face on the bed. Bolts to both gantry plates; holds the M8 hand axle and the turn motor."),
 ("Robot","R02_climber_yoke", lambda: P2.climber_yoke(), RY(-90), 1, "Side wall on the bed. Holds the climb motor's L-bracket; slots set the gear mesh."),
 ("Robot","R03_climber_pinion_14T", lambda: P2.climber_pinion(), I, 2, "5 mm wide, runs inside the rail slot. 100% infill, 0.12 mm layers. 1 spare."),
 ("Robot","R04_turn_pinion_12T", lambda: P2.turn_pinion(), I, 1, "On the turn motor; drives the 48T gear on the hand."),
 ("Robot","R05_hand_tray_arm_48T", lambda: P2.tray_arm(), RX(90), 1, "The 'hand': 48T gear, 2x 608 bearings, arms to the tray and the fork crown."),
 ("Robot","R06_fork_crown", lambda: P2.crown(), RX(90), 1, "Two 4-way forks that slide onto the junction pins; holds 2x LJ12A3."),
 ("Robot","R07_tray_foot", lambda: P2.tray_foot(), RX(180), 1, "Carries the plywood tray on the hand."),
 ("Robot","R10_hand_axle_spacer", lambda: P2.axle_spacer(), RX(90), 1, "On the M8 axle between chassis and hand."),
 ("Robot","R08_wheel_spacer_6mm", lambda: P1.wheel_spacer(), I, 4, "For the concentric wheels (or buy aluminium)."),
 ("Robot","R09_gantry_drill_jig", lambda: P1.drill_jig(), RX(180), 1, "Drill 4 x 5.2 mm in each gantry plate."),
 ("Tray","T01_shoulder_tower", lambda: P1.shoulder_tower(), I, 1, "Shoulder servo holder."),
 ("Tray","T02_upper_arm", lambda: P1.upper_arm(), I, 1, "SCARA link 1, carries the elbow servo upside-down."),
 ("Tray","T03_forearm_hook", lambda: P1.forearm(), RX(180), 1, "SCARA link 2 with the hook pin."),
 ("Tray","T04_basket_stop", lambda: P1.basket_stop(), I, 2, "Tray stops; one holds the basket switch."),
 ("Tray","T05_cam_tof_bracket", lambda: P1.cam_tof_bracket(), RX(90), 1, "ESP32-CAM + VL53L0X on the tower."),
 ("Tray","T06_tof_down_mount", lambda: P1.tof_down_mount(), I, 1, "VL53L0X looking down."),
 ("Junction","J01_turntable_rotor", lambda: P2.rotor(), RX(90), 2, "Carries the 300 mm rail piece; spins on a 608 bearing in the base plate."),
 ("Junction","J02_detent_spring", lambda: P2.detent(), RX(90), 4, "Clicks the turntable at 0 deg and 90 deg. PETG."),
 ("Junction","J03_torque_pin_sleeve", lambda: P2.pin_sleeve(), I, 4, "Over an M8 x 130 bolt: the fixed pins the hand pushes against."),
 ("Junction","J04_stub_block", lambda: P2.stub_block(), I, 2, "Holds the short horizontal rail to the base plates."),
 ("Junction","J05_base_spacer_22", lambda: P2.base_spacer(), I, 10, "Spaces the base plates 22 mm off the rack posts."),
 ("Junction","J06_rail_bridge", lambda: P2.rail_bridge(), RX(-90), 4, "Holds each vertical rail to the rack (top + just above the junction)."),
]
for k,(a0,a1,q) in P2.RACK_STRIPS.items():
    PARTS.append(("Rack", "K_"+k, (lambda a0=a0,a1=a1: P2.rack_strip(a0,a1)), RX(-90), q, f"In-slot gear rack, {a1-a0:.1f} mm. Flange down, 0.12 mm layers."))
info=[]
for grp,name,fn,tf,qty,note in PARTS:
    s=fn(); best=None
    for tol,pt in [(1e-4,2e-3),(0.05,0.01)]:
        V,F=to_mesh(s,tol,pt); st=mesh_stats(V,F)
        if best is None or st['edges_not_2']<best[2]['edges_not_2']: best=(V,F,st)
    V,F,st=best
    V=V@tf[:3,:3].T
    if np.linalg.det(tf[:3,:3])<0: F=F[:,::-1]
    lo,hi=V.min(0),V.max(0); V=V-np.array([(lo[0]+hi[0])/2,(lo[1]+hi[1])/2,lo[2]])
    write_stl_mesh(V,F,f"{OUT}/{name}.stl",name.encode())
    size=V.max(0)-V.min(0)
    vol=np.sum(np.einsum('ij,ij->i',V[F[:,0]],np.cross(V[F[:,1]],V[F[:,2]])))/6
    info.append(dict(group=grp,file=name+".stl",qty=qty,size=[round(float(x),1) for x in size],grams=round(vol/1000*1.27,1),note=note,
                     fits=bool(max(size[0],size[1])<=218 and size[2]<=245), **st))
    print(name, qty, info[-1]['size'], st, info[-1]['fits'])
json.dump(info,open("out/parts_info.json","w"),indent=1)
print("filament g (solid):", sum(x['grams']*x['qty'] for x in info))
