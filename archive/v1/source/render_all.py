import sys, time; sys.path.insert(0,'.')
import numpy as np, os
os.makedirs("out/renders", exist_ok=True)
import assembly as A
from render import render
from params import *
def objs(items, filt=None):
    out=[]
    for n,s,c in items:
        T=s.triangles()
        if filt is not None:
            lo,hi=filt; v=T.reshape(-1,3)
            if np.any(v.max(0)<lo) or np.any(v.min(0)>hi): continue
            # clip triangles fully outside
            cen=T.mean(1); m=np.all((cen>=lo-5)&(cen<=hi+5),axis=1); T=T[m]
        out.append((T,c))
    return out
which = sys.argv[1:] or ['iso','pulled','carriage','side','top']
items,_ = A.build(2,"reach")
if 'iso' in which:
    render(objs(items),"out/renders/01_assembly_reach.png",az=32,el=20,W=1400,H=1500,title="Level 3: lift at shelf height, hook pin inside the basket")
if 'pulled' in which:
    it2,_=A.build(2,"pulled",raise_=90)
    render(objs(it2),"out/renders/02_assembly_carrying.png",az=-35,el=24,W=1400,H=1500,title="Carrying: basket pulled onto the tray, lift moving")
if 'carriage' in which:
    lo=np.array([140,-90,400]); hi=np.array([240,40,580])
    render(objs(items,(lo,hi)),"out/renders/03_carriage_closeup.png",az=150,el=25,W=1400,H=1100,title="Right carriage (seen from behind/outside): rack, pinion, JGY-370, LJ12A3 sensor")
if 'side' in which:
    lo=np.array([-400,-400,380]); hi=np.array([400,260,720])
    render(objs(items,(lo,hi)),"out/renders/04_side_section.png",az=90,el=0,W=1500,H=900,title="Side view at level 3: forearm passes over the basket rim, under the next shelf")
if 'top' in which:
    it3,_=A.build(2,"pulled")
    lo=np.array([-260,-360,455]); hi=np.array([260,260,640])
    render(objs(items,(lo,hi)),"out/renders/05_top_reach.png",az=0,el=89.9,W=1100,H=1300,title="Top view: arm reaching into shelf")
    render(objs(it3,(lo,hi)),"out/renders/06_top_pulled.png",az=0,el=89.9,W=1100,H=1300,title="Top view: basket pulled onto tray")
