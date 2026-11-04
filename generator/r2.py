import sys, time; sys.path.insert(0,'.')
import numpy as np, assembly_v2 as A
from params import *
from render import render
B_all=[(b,l) for b in range(2) for l in range(5)]
others=[x for x in B_all if x!=(0,2)]
def objs(items, lo=None, hi=None):
    out=[]
    for n,s,c,g in items:
        T=s.triangles()
        if lo is not None:
            cen=T.mean(1); m=np.all((cen>=lo)&(cen<=hi),axis=1); T=T[m]
            if not len(T): continue
        out.append((T,c))
    return out
w=sys.argv[1:] or ['a','b','c','d']
if 'a' in w:
    it=A.build((RAIL_X[0], LEVELS[2]-TRAY_TOP_REL), 0, [0,-90], BASKET_Y0+15, B_all)
    render(objs(it),'out/renders/v2_01_reach.png',az=28,el=16,W=1600,H=1300,title="Robot on bay-1 rail at shelf 3, hook in the basket")
if 'b' in w:
    it=A.build(((RAIL_X[0]+RAIL_X[1])/2, JZ), -90, [-90,-90], -178.0, others+['tray'])
    render(objs(it),'out/renders/v2_02_travel.png',az=22,el=14,W=1600,H=1300,title="Driving sideways on the bottom rail, basket level on the tray")
if 'c' in w:
    it=A.build(JUNCTIONS[0], -45, [-45,-90], -178.0, others+['tray'])
    lo=np.array([RAIL_X[0]-260,-400,-10]); hi=np.array([RAIL_X[0]+420,60,420])
    render(objs(it,lo,hi),'out/renders/v2_03_turning.png',az=-20,el=12,W=1500,H=1100,title="Junction J1 mid-turn (45°): robot + rail piece rotate, hand and tray stay level")
if 'd' in w:
    it=A.build((RAIL_X[0], JZ+260), 0, [0,-90], -178.0, others)
    lo=np.array([RAIL_X[0]-230,-200,-10]); hi=np.array([RAIL_X[0]+210,60,470])
    render(objs(it,lo,hi),'out/renders/v2_04_junction_empty.png',az=-18,el=10,W=1500,H=1200,title="Junction J1 (turntable vertical) with the robot just above it")
