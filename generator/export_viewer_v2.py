import sys, json, base64; sys.path.insert(0,'.')
import numpy as np
import assembly_v2 as A
from assembly_servo import servo_model
from params import *
import parts as P1
from csg import Solid
meshes=[]
def q(T):
    a=np.round(T.reshape(-1,3)/0.05).astype(np.int16); return base64.b64encode(a.tobytes()).decode()
def addm(n,g,s,c):
    T=s.triangles() if isinstance(s,Solid) else s
    if len(T)==0: return
    meshes.append(dict(n=n,g=g,c='#%02x%02x%02x'%tuple(c),p=q(T)))
def merged(items, g):
    d={}
    for n,s,c in items: d.setdefault((n,c),[]).append(s.triangles())
    for (n,c),L in d.items(): addm(n,g,np.concatenate(L),c)
merged(A.static_items(),"static")
for j,(jx,jz) in enumerate(JUNCTIONS):
    it=[(n,s.translate(-jx,0,-jz),c) for n,s,c in A.rotor_items(j,0)]
    merged(it,f"rotor{j}")
merged(A.body_items(),"body")
tr=[x for x in A.tray_items(-178.0)]
up,fo,sh=A.arm_local()
tr+= [(n,s.translate(RAIL_OFF+SHOULDER[0],SHOULDER[1],TRAY_TOP_REL),c) for n,s,c in sh]
merged(tr,"tray")
merged(up,"upper"); merged(fo,"fore")
bs=A.basket_solid()
for i in range(10): addm(f"basket {i+1}", f"basket{i}", bs, A.BASKET_COLS[i])
meta=dict(LEVELS=LEVELS,BAY_X=BAY_X,RAIL_X=RAIL_X,JZ=JZ,JUNCTIONS=JUNCTIONS,TRAY_TOP=TRAY_TOP_REL,RAIL_OFF=RAIL_OFF,
          APPROACH=APPROACH,SHOULDER=SHOULDER,L1=L1,L2=L2,BASKET_Y0=BASKET_Y0,PIN_REACH=BASKET_Y0+15,PIN_PULLED=-178.0,
          PIN_OUT=-207.0,PIN_PUSH=BASKET_Y0-5,TRAY_FRONT=-193.0)
json.dump(dict(meta=meta,meshes=meshes),open("out/viewer_v2.json","w"),separators=(',',':'))
print(len(meshes), sum(len(m['p']) for m in meshes)/1e6,"MB")
