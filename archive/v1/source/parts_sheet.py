import sys, json, struct; sys.path.insert(0,'.')
import numpy as np, os
os.makedirs("out/renders", exist_ok=True)
from render import render
info=json.load(open('out/parts_info.json'))
def read_stl(p):
    b=open(p,'rb').read(); n=struct.unpack('<I',b[80:84])[0]
    a=np.frombuffer(b[84:84+n*50],dtype=[('n','<f4',3),('v','<f4',9),('a','<u2')]); return a['v'].reshape(-1,3,3).astype(float)
sel=[x for x in info if not x['file'].endswith(('_L.stl','_B.stl'))]
objs=[]; labels=[]; x=0; y=0; rowh=0; W=520
cols=[(245,130,35),(40,170,160)]
for i,p in enumerate(sel):
    T=read_stl('out/stl/'+p['file']); lo=T.reshape(-1,3).min(0); hi=T.reshape(-1,3).max(0); sz=hi-lo
    if x+sz[0]>W: x=0; y-=rowh+45; rowh=0
    T=T-lo+np.array([x,y-sz[1],0]); objs.append((T,cols[i%2]))
    labels.append(((x,y+8,0), f"{p['file'][:-4]}  x{p['qty']}"))
    x+=sz[0]+40; rowh=max(rowh,sz[1])
render(objs,'out/renders/07_printed_parts.png',az=0,el=89.9,W=1500,H=1300,title="Printed parts in print orientation (as exported); L/B mirror variants not shown",labels=labels)
render(objs,'out/renders/07b_printed_parts_iso.png',az=-25,el=40,W=1500,H=1100,title="Printed parts (print orientation)")
