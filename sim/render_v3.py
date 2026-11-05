"""Render the v3 README images -> docs/renders/v3_*.png (MuJoCo, headless).
    pixi run -e sim render-v3
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, mujoco, PIL.Image as I
import run_v3 as R
P3, A3 = R.P3, R.A3
out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "docs", "renders")
os.makedirs(out, exist_ok=True)
s = R.Sim(); s.show_balance = False
r = mujoco.Renderer(s.m, 900, 1400); cam = mujoco.MjvCamera()
MM = 1e-3
def shot(name, look, az, el, dist, balance=False):
    s.apply()
    cam.azimuth, cam.elevation, cam.distance = az, el, dist; cam.lookat[:] = look
    r.update_scene(s.d, cam)
    if balance:
        s.show_balance = True; s.draw_balance(r.scene); s.show_balance = False
    I.fromarray(r.render()).save(os.path.join(out, name + ".png"))
# 1-2: the whole floor
shot("v3_01_floor", (0.75, 0.55, 0.3), 135, -40, 4.8)
shot("v3_02_top_view", (0.9, 0.5, 0.2), 90, -89, 3.3)
# 3: at a shelf, both fingers down round the basket (row 1, bay 1, shelf 3)
b, l = 0, 2
s.cmd.update(x=A3.bay_rail_x(b), z=P3.cz_for_level(l), h1=0, h2=0, r00=0, r01=0, arm=P3.TIP_SHELF, f=90.0)
s.basket[(b, l)] = "held"; s.ctx[(b, l)] = ("shelf", (b, l)); s.hold[(b, l)] = P3.BASKET_Y0
s.apply(); rp = s.d.xpos[s.m.body("robot").id]
shot("v3_03_pick_captured", rp + np.array([P3.BXR, 60, P3.T_V3 + 90]) * MM, 100, -50, 0.8)
# 4: pulling the basket half way onto the tray (picker mechanism visible)
s.cmd.update(arm=(P3.TIP_SHELF + P3.TIP_PULLED) / 2); s.apply()
shot("v3_04_pulling", rp + np.array([P3.BXR + 140, -60, P3.T_V3 + 50]) * MM, 125, -28, 0.75)
# 5: picker drive close up (servos, belts, cross shaft), arms half out
s.basket[(b, l)] = "shelf"
s.cmd.update(arm=(P3.TIP0 + P3.TIP_SHELF) / 2, f=0.0); s.apply()
shot("v3_05_picker_drive", rp + np.array([P3.BXR, -300, P3.T_V3 + 30]) * MM, 70, -18, 0.6)
# 6-7: on the transfer cart, half way across an aisle; pinion in the transfer rack
s.cmd.update(x=P3.X_CART, z=P3.CZ_HOME, h1=-90, h2=-90, r00=-90, r01=-90, arm=P3.TIP0, f=0.0,
             ry=P3.ROW_P / 2, cart=P3.ROW_P / 2, lift=0.0)
shot("v3_06_transfer_cart", (P3.X_CART * MM + 0.1, P3.ROW_P / 2 * MM, 0.3), 160, -30, 1.6)
# 8: weight balance arrows with a basket on the tray
s.cmd.update(x=A3.bay_rail_x(0) + 130, ry=0.0, cart=0.0, lift=P3.TURN_LIFT)
s.basket[(0, 4)] = "tray"; s.apply(); rp = s.d.xpos[s.m.body("robot").id]
shot("v3_08_weight_balance", rp + np.array([120, 0, 0]) * MM, 90, -3, 1.1, balance=True)
# 9-12: the docking station (row 1): pushing onto the raised platform, jack half way down, lowered on the raised floor, jack
k, xc, pxc = (1, 1), P3.DOCK_X_C, P3.PXC
s.cmd.update(x=xc, z=P3.CZ_HOME, h1=-90, h2=-90, r00=-90, r01=-90, arm=(P3.TIP_TRAY + P3.TIP_PUSHED) / 2, f=90.0,
             ry=0.0, cart=0.0, lift=P3.TURN_LIFT, j0=P3.JACK_UP)
s.basket[(0, 4)] = "shelf"
s.basket[k] = "held"; s.ctx[k] = ("dock", 0); s.hold[k] = 0.0
look = np.array([pxc, 100, 150]) * MM
shot("v3_09_dock_push", look + np.array([0, -0.1, 0.05]), 130, -22, 1.8)
s.cmd.update(x=s.x_home, arm=P3.TIP0, f=0.0, j0=120.0); s.basket[k] = "dock"; s.dock_of[k] = 0
shot("v3_10_dock_jack_lowering", look, 160, -15, 1.3)
s.cmd.update(j0=P3.JACK_DOWN)
shot("v3_11_dock_lowered", look, 150, -25, 1.5)
s.basket[k] = "away"; s.away_pos[k] = (P3.X_CART + 700.0, 80.0, 0.0); s.dock_of.pop(k)
s.cmd.update(j0=140.0)
for kk in list(s.away_pos):                                   # parked parcels out of the picture
    s.away_pos[kk] = (P3.X_CART + 3000.0, 80.0, 0.0)
shot("v3_12_dock_jack", np.array([pxc, 100, 90]) * MM, 205, -18, 0.95)
print("ok")
