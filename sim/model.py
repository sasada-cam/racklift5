"""Build the MuJoCo model of RackLift5 v2 from generator/params.py + the generator meshes.

Frames: metres, global frame of params.py (x right, y into the rack, z up).
Chain:  world -> carriage (slide x, slide z) -> chassis (hinge y = body angle)
                                              -> hand (hinge y, kept level) -> upper arm (hinge z) -> forearm (hinge z)
Turntable rotors are hinges at the junctions; baskets are free bodies.
"""
import os, sys, struct, math
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
GEN = os.path.abspath(os.path.join(HERE, "..", "generator"))
sys.path.insert(0, GEN)
import params as P                      # noqa: E402
import assembly_v2 as A                 # noqa: E402

BUILD = os.path.join(HERE, "build")
MM = 1e-3
G = 9.81

# ---- masses (kg): ESTIMATES - edit to match the real build -------------------------------------
M_CARRIAGE, M_CHASSIS = 0.2, 1.0
M_HAND = 1.3                       # tray + plywood + shoulder tower + servos + electronics on the tray
M_UPPER, M_FORE = 0.12, 0.05
M_BASKET = 0.25
HAND_COM = (0.23, -0.17, 0.03)     # relative to the hand axle (tray sits ~230 mm to the side)


def write_stl(path, tris):
    tris = np.asarray(tris, dtype=np.float32)
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    with open(path, "wb") as f:
        f.write(b"racklift5".ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        rec = np.zeros(len(tris), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
        rec["n"], rec["v"] = n, tris
        f.write(rec.tobytes())


class Meshes:
    def __init__(self):
        self.dir = os.path.join(BUILD, "meshes"); os.makedirs(self.dir, exist_ok=True)
        self.k = 0; self.assets = []

    def add(self, solid, offset=(0, 0, 0)):
        tris = solid.triangles() - np.array(offset, float)
        name = f"m{self.k:03d}"; self.k += 1
        write_stl(os.path.join(self.dir, name + ".stl"), tris)
        self.assets.append(f'<mesh name="{name}" file="meshes/{name}.stl" scale="{MM} {MM} {MM}" inertia="convex"/>')
        return name


def rgba(c, a=1.0):
    return f"{c[0]/255:.3f} {c[1]/255:.3f} {c[2]/255:.3f} {a}"


def vis(M, items, offset=(0, 0, 0), alpha=1.0):
    return "\n".join(f'<geom type="mesh" mesh="{M.add(s, offset)}" rgba="{rgba(c, alpha)}" contype="0" conaffinity="0" group="1" density="0"/>'
                     for _, s, c in items)


def v3(x, y, z):
    return f"{x*MM:.6f} {y*MM:.6f} {z*MM:.6f}"


def box_geom(x0, x1, y0, y1, z0, z1, ctype, caff, extra=""):
    c = ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2); h = ((x1 - x0) / 2, (y1 - y0) / 2, (z1 - z0) / 2)
    return (f'<geom type="box" pos="{v3(*c)}" size="{v3(*h)}" contype="{ctype}" conaffinity="{caff}" '
            f'rgba="0 0 0 0" group="3" {extra}/>')


def com_inertial(parts, base_diag):
    """parts = [(mass, (x,y,z))] about the body origin -> <inertial> at the combined COM"""
    m = sum(p[0] for p in parts)
    com = np.array(sum(np.array(p[1]) * p[0] for p in parts) / m)
    d = np.array(base_diag, float)
    for pm, pp in parts:
        r = np.array(pp) - com
        d = d + pm * np.array([r[1]**2 + r[2]**2, r[0]**2 + r[2]**2, r[0]**2 + r[1]**2]) * 0.0  # base_diag already covers spread
    return f'<inertial pos="{com[0]:.5f} {com[1]:.5f} {com[2]:.5f}" mass="{m:.4f}" diaginertia="{d[0]:.6f} {d[1]:.6f} {d[2]:.6f}"/>', m, com


def build(payload_kg=0.5, counterweight_kg=0.0, counter_x=-0.23, out=None):
    os.makedirs(BUILD, exist_ok=True)
    M = Meshes()
    T = P.TRAY_TOP_REL
    bxr = P.RAIL_OFF
    x_home = (P.RAIL_X[0] + P.RAIL_X[1]) / 2

    # ------------------------------------------------------------------ static world (visual) + collision
    world = [vis(M, A.static_items())]
    for bx in P.BAY_X:
        for L in P.LEVELS:
            for sx in (-1, 1):
                xa, xb = sorted((bx + sx * (P.XI - P.ANG), bx + sx * P.XI))
                world.append(box_geom(xa, xb, 0, P.RACK_D, L - 6, L, 1, 2, 'friction="0.5 0.005 0.0001"'))
    world.append('<geom type="plane" size="3 3 0.1" rgba="0.92 0.92 0.94 1" contype="0" conaffinity="0"/>')

    # rotors (turntable + rail piece), local frame = junction centre
    jx0, jz0 = P.JUNCTIONS[0]
    rot_items = A.rotor_items(0, 0)
    rot_mesh = vis(M, rot_items, offset=(jx0, 0, jz0))
    rotors = ""
    for j, (jx, jz) in enumerate(P.JUNCTIONS):
        rotors += (f'<body name="rotor{j}" pos="{v3(jx, 0, jz)}"><joint name="rotor{j}" type="hinge" axis="0 1 0" range="-1.8 0.2" armature="0.01"/>'
                   f'<inertial pos="0 0 0" mass="0.1" diaginertia="0.002 0.002 0.002"/>{rot_mesh}</body>\n')

    # ------------------------------------------------------------------ robot
    body_vis = vis(M, A.body_items())
    tray_vis = vis(M, A.tray_items(-178.0))
    up, fo, sh = A.arm_local()
    sh_vis = vis(M, sh); up_vis = vis(M, up); fo_vis = vis(M, fo)

    hand_parts = [(M_HAND, HAND_COM)]
    cw = ""
    if counterweight_kg > 0:
        cpos = (counter_x, -0.17, 0.03)
        hand_parts.append((counterweight_kg, cpos))
        cw = (f'<geom type="box" pos="{cpos[0]} {cpos[1]} {cpos[2]}" size="0.04 0.05 0.03" rgba="0.2 0.2 0.25 1" '
              f'contype="0" conaffinity="0" group="1"/>')
    hand_inertial, hand_m, hand_com = com_inertial(hand_parts, (0.03, 0.02, 0.04))

    deck = box_geom(100, 380, -330, -3, T - 12, T, 1, 2, 'friction="0.5 0.005 0.0001"')
    stops = "".join(box_geom(bxr + sx - 20, bxr + sx + 20, -217, -193, T, T + 26, 1, 2) for sx in (-80, 80))
    pin_z0 = (P.UPPER_Z - (P.SV_HORN_TOP - P.SV_TAB_Z) - P.ARM_T)       # forearm underside above tray top
    pin = (f'<geom type="cylinder" fromto="{v3(P.L2, 0, pin_z0 - P.PIN_LEN)} {v3(P.L2, 0, pin_z0)}" size="{P.PIN_DIA/2*MM}" '
           f'contype="4" conaffinity="2" rgba="0.9 0.2 0.2 1" group="1" friction="0.3 0.005 0.0001"/>')

    arm = (f'<body name="upper" pos="{v3(bxr, P.SHOULDER[1], T)}">'
           f'<joint name="q1" type="hinge" axis="0 0 1" range="-0.4 1.8" armature="0.005"/>'
           f'<inertial pos="{0.09:.3f} 0 0.15" mass="{M_UPPER}" diaginertia="0.0001 0.0004 0.0004"/>{up_vis}'
           f'<body name="fore" pos="{v3(P.L1, 0, 0)}"><joint name="q2" type="hinge" axis="0 0 1" range="0.1 3.2" armature="0.005"/>'
           f'<inertial pos="{0.09:.3f} 0 0.1" mass="{M_FORE}" diaginertia="0.00005 0.00017 0.00017"/>{fo_vis}{pin}</body></body>')

    robot = f'''
<body name="carriage" pos="0 0 0" gravcomp="1">
  <joint name="x" type="slide" axis="1 0 0" range="{(P.RAIL_X[0]-80)*MM} {(P.RAIL_X[1]+80)*MM}" armature="0.5"/>
  <joint name="z" type="slide" axis="0 0 1" range="{(P.JZ-10)*MM} {(P.RAIL_TOP-120)*MM}" armature="0.5"/>
  <inertial pos="0 0 0" mass="{M_CARRIAGE}" diaginertia="0.001 0.001 0.001"/>
  <body name="chassis" pos="0 0 0" gravcomp="1">
    <joint name="theta" type="hinge" axis="0 1 0" range="-1.75 0.2" armature="0.02"/>
    <inertial pos="0 -0.1 0" mass="{M_CHASSIS}" diaginertia="0.006 0.006 0.006"/>
    {body_vis}
    <body name="hand" pos="0 0 0">
      <joint name="hand" type="hinge" axis="0 1 0" range="-0.2 1.75" armature="0.02"/>
      {hand_inertial}
      {tray_vis}{sh_vis.replace('<geom', f'<geom pos="{v3(bxr, P.SHOULDER[1], T)}"')}
      {deck}{stops}{cw}
      {arm}
    </body>
  </body>
</body>'''

    # ------------------------------------------------------------------ baskets
    bs = A.basket_solid(); bmesh = M.add(bs)
    bw, bd_, bh, t = P.BASKET_W, P.BASKET_D, P.BASKET_H, 3.0
    bm = M_BASKET + payload_kg
    I = (bm / 12 * (bd_**2 + bh**2) * 1e-6, bm / 12 * (P.BASKET_W**2 + bh**2) * 1e-6, bm / 12 * (P.BASKET_W**2 + bd_**2) * 1e-6)
    baskets, keyf = "", []
    k = 0
    for b in range(P.N_BAYS):
        for l in range(len(P.LEVELS)):
            col = A.BASKET_COLS[k % 10]
            geoms = "".join([
                box_geom(-bw/2, bw/2, 0, bd_, 0, t, 2, 1 | 4, 'friction="0.5 0.005 0.0001"'),
                box_geom(-bw/2, -bw/2 + t, 0, bd_, 0, bh, 2, 1 | 4), box_geom(bw/2 - t, bw/2, 0, bd_, 0, bh, 2, 1 | 4),
                box_geom(-bw/2, bw/2, 0, t, 0, bh, 2, 1 | 4), box_geom(-bw/2, bw/2, bd_ - t, bd_, 0, bh, 2, 1 | 4)])
            baskets += (f'<body name="basket_{b}_{l}" pos="{v3(P.BAY_X[b], P.BASKET_Y0, P.LEVELS[l] + 0.2)}"><freejoint name="bj_{b}_{l}"/>'
                        f'<inertial pos="{v3(0, bd_/2, 30)}" mass="{bm}" diaginertia="{I[0]:.6f} {I[1]:.6f} {I[2]:.6f}"/>'
                        f'<geom type="mesh" mesh="{bmesh}" rgba="{rgba(col)}" contype="0" conaffinity="0" group="1" density="0"/>{geoms}</body>\n')
            k += 1

    # ------------------------------------------------------------------ actuators (these become the sliders in the viewer's Control panel)
    act = f'''
<position name="x" joint="x" kp="20000" kv="800" ctrlrange="{(P.RAIL_X[0]-80)*MM} {(P.RAIL_X[1]+80)*MM}" forcerange="-300 300"/>
<position name="z" joint="z" kp="20000" kv="800" ctrlrange="{(P.JZ-10)*MM} {(P.RAIL_TOP-120)*MM}" forcerange="-300 300"/>
<position name="turn_body" joint="theta" kp="400" kv="20" ctrlrange="-1.7 0.1" forcerange="-60 60"/>
<position name="turntable_1" joint="rotor0" kp="200" kv="3" ctrlrange="-1.7 0.1" forcerange="-30 30"/>
<position name="turntable_2" joint="rotor1" kp="200" kv="3" ctrlrange="-1.7 0.1" forcerange="-30 30"/>
<position name="hand_level" joint="hand" kp="400" kv="20" ctrlrange="-0.1 1.7" forcerange="-60 60"/>
<position name="arm_shoulder" joint="q1" kp="60" kv="1.5" ctrlrange="-0.3 1.7" forcerange="-8 8"/>
<position name="arm_elbow" joint="q2" kp="60" kv="1.5" ctrlrange="0.2 3.1" forcerange="-8 8"/>'''

    xml = f'''<mujoco model="racklift5_v2">
<compiler angle="radian" meshdir="{BUILD}" autolimits="true"/>
<option timestep="0.001" integrator="implicitfast" gravity="0 0 -9.81"/>
<visual><global offwidth="1600" offheight="900"/><quality shadowsize="2048"/><headlight ambient="0.45 0.45 0.45" diffuse="0.6 0.6 0.6"/></visual>
<asset>{chr(10).join(M.assets)}</asset>
<worldbody>
<light pos="0 -2 3" dir="0 0.5 -1" diffuse="0.5 0.5 0.5"/>
{chr(10).join(world)}
{rotors}
{robot}
{baskets}
</worldbody>
<actuator>{act}</actuator>
</mujoco>'''
    path = out or os.path.join(BUILD, "racklift5.xml")
    open(path, "w").write(xml)
    return path, dict(hand_mass=hand_m, hand_com=hand_com, x_home=x_home)


if __name__ == "__main__":
    import mujoco
    path, info = build()
    m = mujoco.MjModel.from_xml_path(path)
    print("compiled:", m.nbody, "bodies", m.ngeom, "geoms", m.nu, "actuators", info)
