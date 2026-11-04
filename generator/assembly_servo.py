import numpy as np
from csg import *
from params import *
def MT(x=0, y=0, z=0):
    M = np.eye(4); M[:3, 3] = [x, y, z]; return M
def servo_model(spline_xy, ang_deg, bottom_z, inverted=False):
    b = box(-SV_SPLINE_OFF, SV_L - SV_SPLINE_OFF, -SV_W / 2, SV_W / 2, 0, SV_H)
    tabs = box(-SV_SPLINE_OFF - 7, SV_L - SV_SPLINE_OFF + 7, -SV_W / 2, SV_W / 2, SV_TAB_Z, SV_TAB_Z + SV_TAB_T)
    sp = cyl(6.5, SV_H, SV_H + 4, 'z', (0, 0), 20)
    horn = cyl(HORN_D / 2, SV_HORN_TOP - 4, SV_HORN_TOP, 'z', (0, 0), 28)
    body = Solid(b.polys + tabs.polys); hs = Solid(sp.polys + horn.polys)
    M = rot('z', ang_deg)
    if inverted: M = M @ rot('x', 180)
    M = MT(spline_xy[0], spline_xy[1], bottom_z) @ M
    return body.transform(M), hs.transform(M)
