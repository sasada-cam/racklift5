import sys, time; sys.path.insert(0,'.')
import numpy as np
import assembly as A
from params import *
from interfere import interference
STATIC = ("slotted angle rack","ledge bolt","V-slot","printed rack segment","printed rail mount","limit switch")
ARM = ("20kg servo","servo horn","printed upper arm","printed forearm")
def is_static(n): return n.startswith(STATIC) and not n.startswith("limit switch (basket") or (n.startswith("basket") and "tray" not in n)
SKIP = {("V wheel","V-slot 2020 rail"),("printed pinion","printed rack segment")}
def check(level, state, raise_=0.0, T=None, pin_y=None, h=1.0):
    items, info = A.build(level, state, raise_, T, pin_y)
    tris = [(n, s.triangles()) for n,s,c in items]
    stat = [(n,t) for n,t in tris if is_static(n)]
    mov = [(n,t) for n,t in tris if not is_static(n)]
    hits = []
    for nm, tm in mov:
        for ns, ts in stat:
            if (nm, ns) in SKIP: continue
            v, p = interference(tm, ts, h)
            if v > 0.5: hits.append((nm, ns, round(v,1), np.round(p,0)))
    arm = [(n,t) for n,t in mov if n.startswith(ARM)]
    rest = [(n,t) for n,t in mov if not n.startswith(ARM)]
    for na, ta in arm:
        for nr, tr in rest:
            v, p = interference(ta, tr, h)
            if v > 0.5: hits.append((na, nr, round(v,1), np.round(p,0)))
    return hits, info
if __name__ == "__main__":
    cases = []
    for i in range(5):
        cases += [(i,"empty",APPROACH,None,BASKET_Y0+15), (i,"reach",0,None,None), (i,"pulled",0,None,None), (i,"loaded",0,None,None), (i,"pushed",0,None,None)]
        if i < 4: cases += [(i,"pulled",90,None,None), (i,"loaded",90,None,None)]
    cases += [(0,"pulled",0,PC_HOME-T_OFF,None), (4,"pulled",0,905-T_OFF,None)]
    for c in cases:
        t=time.time(); hits, info = check(*c)
        print(c, "T=%.0f"%info["T"], "OK" if not hits else "", f"{time.time()-t:.0f}s", flush=True)
        for hh in hits: print("    ", hh)
