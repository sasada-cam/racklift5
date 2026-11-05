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
s.basket[(b, l)] = "held"; s.hold[(b, l)] = P3.BASKET_Y0
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
print("ok")
