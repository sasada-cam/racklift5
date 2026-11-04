import sys, json, base64; sys.path.insert(0,'.')
import numpy as np
import assembly as A
from params import *
from clearance import is_static
from csg import to_mesh, Solid
T0 = LEVELS[0]
items, info = A.build(0, "empty", T=T0, pin_y=-178.0)
ARMN = ("printed upper arm","20kg servo (elbow)","servo horn (elbow)","printed forearm + hook pin")
meshes = []
def q(T):
    a = np.round(T.reshape(-1,3)/0.05).astype(np.int16)
    return base64.b64encode(a.tobytes()).decode()
def addm(name, group, s, col):
    T = s.triangles() if isinstance(s, Solid) else s
    meshes.append(dict(n=name, g=group, c='#%02x%02x%02x'%tuple(col), p=q(T)))
# merge same name+group+color into one mesh to cut draw calls
merged = {}
for n,s,c in items:
    if n.startswith(ARMN) or n.startswith("basket"): continue
    g = "static" if is_static(n) else "lift"
    merged.setdefault((n,g,c), []).append(s.triangles())
for (n,g,c),L in merged.items():
    addm(n, g, np.concatenate(L), c)
# arm groups in local frames
addm("printed upper arm", "upper", A.part("upper_arm").transform(A.M_T(0,0,T0+UPPER_Z)), A.C_PRINT)
b,h = A.servo_model((L1,0), 180, T0+UPPER_Z+SV_TAB_Z, inverted=True)
addm("20kg servo (elbow)", "upper", b, A.C_SERVO); addm("servo horn (elbow)", "upper", h, A.C_STEEL)
addm("printed forearm + hook pin", "fore", A.part("forearm").transform(A.M_T(0,0,T0+UPPER_Z-(SV_HORN_TOP-SV_TAB_Z)-ARM_T)), A.C_PRINT)
for i in range(5):
    addm(f"basket {i+1}", f"basket{i}", A.basket(0,0), A.BASKET_COLS[i])
meta = dict(LEVELS=LEVELS, T0=T0, T_OFF=T_OFF, APPROACH=APPROACH, SHOULDER=SHOULDER, L1=L1, L2=L2, BASKET_Y0=BASKET_Y0,
            PIN_REACH=BASKET_Y0+15, PIN_PULLED=-178.0, PIN_OUT=-207.0, PIN_PUSH=BASKET_Y0-5, TRAY_FRONT=-193.0, HOME_T=PC_HOME-T_OFF)
json.dump(dict(meta=meta, meshes=meshes), open("out/viewer_data.json","w"), separators=(',',':'))
print(len(meshes), sum(len(m['p']) for m in meshes)/1e6, "MB b64")
