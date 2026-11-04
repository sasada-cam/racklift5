import sys, time; sys.path.insert(0,'.')
import numpy as np, assembly_v2 as A
from params import *
from interfere import interference, bb
SKIP = [("V wheel","V-slot"),("V wheel","rail piece"),("printed climber pinion","printed in-slot rack"),
        ("printed turn pinion","printed hand"),("M8 axle","printed hand"),("spacer","V-slot"),("spacer","rail piece")]
def skip(a,b):
    return any((a.startswith(x) and b.startswith(y)) or (b.startswith(x) and a.startswith(y)) for x,y in SKIP)
_tc = {}
def tris(n, s):
    return s.triangles()
def check(C, theta, rot, pin_y=-178.0, baskets=None, h=1.0, label=""):
    items = A.build(C, theta, rot, pin_y, baskets)
    T = [(n, s.triangles(), g) for n,s,c,g in items]
    fixed = [x for x in T if x[2] in ("static","rotor")]
    mov = [x for x in T if x[2] in ("body","tray","arm")]
    hits=[]
    boxes = {id(t): bb(t) for _,t,_ in T}
    def pair(a, b):
        (na,ta,_),(nb,tb,_) = a,b
        if skip(na,nb): return
        a0,a1 = boxes[id(ta)]; b0,b1 = boxes[id(tb)]
        if np.any(a1 < b0) or np.any(b1 < a0): return
        v,p = interference(ta,tb,h)
        if v>0.5: hits.append((na,nb,round(v,1),np.round(p,0)))
    for m in mov:
        for f in fixed: pair(m,f)
    body=[x for x in mov if x[2]=="body"]; rest=[x for x in mov if x[2]!="body"]
    for b in body:
        for r in rest: pair(b,r)
    arm=[x for x in mov if x[2]=="arm"]; tray=[x for x in mov if x[2]=="tray"]
    for a in arm:
        for t in tray:
            if t[0].startswith("basket"): continue
            pair(a,t)
    return hits
if __name__=="__main__":
    B_all=[(b,l) for b in range(2) for l in range(5)]
    cases=[]
    for b in range(2):
        for l in range(5):
            others=[x for x in B_all if x!=(b,l)]
            z=LEVELS[l]-TRAY_TOP_REL
            cases.append((f"bay{b+1} L{l+1} approach", (RAIL_X[b], z+APPROACH), 0, [0,0], BASKET_Y0+15, B_all))
            cases.append((f"bay{b+1} L{l+1} hook", (RAIL_X[b], z), 0, [0,0], BASKET_Y0+15, B_all))
            cases.append((f"bay{b+1} L{l+1} pulled", (RAIL_X[b], z), 0, [0,0], -178.0, others+['tray']))
            cases.append((f"bay{b+1} L{l+1} push", (RAIL_X[b], z), 0, [0,0], BASKET_Y0-5, B_all))
    others=[x for x in B_all if x!=(0,2)]
    for j in range(2):
        for th in (0,-15,-30,-45,-60,-75,-90):
            rot=[0,0]; rot[j]=th
            if j==1: rot[0]=-90
            cases.append((f"turn J{j+1} {th}", JUNCTIONS[j], th, rot, -178.0, others+['tray']))
    for x in np.linspace(RAIL_X[0], RAIL_X[1], 9):
        cases.append((f"H travel x={x:.0f}", (x, JZ), -90, [-90,-90], -178.0, others+['tray']))
    for z in (JZ, JZ+60, JZ+120, LEVELS[1]-TRAY_TOP_REL):
        cases.append((f"climb bay2 z={z:.0f}", (RAIL_X[1], z), 0, [-90,0], -178.0, others+['tray']))
    sel = sys.argv[1] if len(sys.argv)>1 else ""
    for c in cases:
        if sel and sel not in c[0]: continue
        t=time.time(); hits=check(*c[1:])
        print(f"{c[0]:28s}", "OK" if not hits else "HIT", f"{time.time()-t:.0f}s", flush=True)
        for hh in hits: print("     ",hh)
