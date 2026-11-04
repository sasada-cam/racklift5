import sys, json, struct; sys.path.insert(0,'.')
import numpy as np
from render import render
info=json.load(open('out/parts_info.json'))
def read_stl(p):
    b=open(p,'rb').read(); n=struct.unpack('<I',b[80:84])[0]
    a=np.frombuffer(b[84:84+n*50],dtype=[('n','<f4',3),('v','<f4',9),('a','<u2')]); return a['v'].reshape(-1,3,3).astype(float)
cols={'Robot':(245,130,35),'Tray':(40,170,160),'Junction':(150,95,200),'Rack':(120,120,130)}
objs=[]; x=0; y=0; rowh=0; W=640
for p in info:
    if p['file'].startswith('K_') and p['file']!='K_rack_33T.stl': continue
    T=read_stl('out/stl/'+p['file']); lo=T.reshape(-1,3).min(0); hi=T.reshape(-1,3).max(0); sz=hi-lo
    if x+sz[0]>W: x=0; y-=rowh+30; rowh=0
    objs.append((T-lo+np.array([x,y-sz[1],0]),cols[p['group']])); x+=sz[0]+30; rowh=max(rowh,sz[1])
render(objs,'out/renders/v2_parts.png',az=-25,el=42,W=1500,H=1150,title="Printed parts (orange robot, teal tray, purple junction, grey rack)")
