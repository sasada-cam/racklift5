"""MuJoCo model of RackLift5 v3 (kinematic: poses are set directly, no physics).
Floor of racks: rows of racks (all facing the same way), rails on each row's front, a transfer cart at the rows'
right ends that the robot pushes from row to row with its own pinion.
Compact Squid layout: two horizontal rails, one vertical rail per bay, small crossing turntables; the robot's hubs swivel
90 deg while the body never rotates; fixed battery box; tray with 2-stage telescopic arms (belt-doubled stacked bars) and
flip fingers on telescoping square shafts; servo 1 drives the arms, servo 2 all four fingers.  Every drive part is a body that turns.
Geometry from generator/assembly_v3.py, the same source as the v3 CAD export."""
import os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import model as MD                       # mesh writer + helpers (also puts generator/ on sys.path)
import params_v3 as P3                   # noqa: E402
import assembly_v3 as A3                 # noqa: E402

MM = MD.MM
v3 = MD.v3
INERT = '<inertial pos="0 0 0" mass="0.1" diaginertia="0.0001 0.0001 0.0001"/>'
TURNING = ["arm_drive", "arm_idler", "servo1", "servo2", "fshaft_l", "fshaft_r", "ftube_l", "ftube_r", "finger_l", "finger_r",
           "dpul_ext_l", "dpul_ext_r", "dpul_ret_l", "dpul_ret_r"]


def belt_name(sx, k):
    return f"belt2_{'l' if sx < 0 else 'r'}{k}"


def build(out=None):
    os.makedirs(MD.BUILD, exist_ok=True)
    M = MD.Meshes()
    M.dir = os.path.join(MD.BUILD, "meshes_v3"); os.makedirs(M.dir, exist_ok=True)
    _add = M.add

    def add(solid, offset=(0, 0, 0)):                       # keep v3 meshes in their own folder
        name = _add(solid, offset)
        M.assets[-1] = M.assets[-1].replace('file="meshes/', 'file="meshes_v3/')
        return name
    M.add = add
    vis = lambda items, off=(0, 0, 0): MD.vis(M, items, off)

    world = [vis(A3.static_items()),
             '<geom type="plane" size="3 3 0.1" rgba="0.92 0.92 0.94 1" contype="0" conaffinity="0"/>']
    rotors = ""
    cross = vis(A3.crossing_local())
    for j in range(P3.N_BAYS_ALL):          # crossing pieces at every bay's vertical rail
        for k, zc in enumerate(P3.CROSS_Z):
            ry = A3.row_M(A3.bay_row(j))[1, 3]
            rotors += (f'<body name="cross{j}{k}" pos="{v3(A3.bay_rail_x(j), ry, zc)}">'
                       f'<joint name="cross{j}{k}" type="hinge" axis="0 1 0"/>{INERT}{cross}</body>')
    # transfer cart (no motor): slides along y, pushed by the robot's pinion in the fixed transfer rack
    rotors += f'<body name="cart"><joint name="cart" type="slide" axis="0 1 0"/>{INERT}{vis(A3.cart_local())}</body>'
    def hub(tag):              # hub body contents: the hub plus its climb pinion on its own axle
        return (f'{vis(A3.hub_local())}<body name="pinion_{tag}" pos="{v3(*A3.PINION_AT)}">'
                f'<joint name="pinion_{tag}" type="hinge" axis="1 0 0"/>{INERT}{vis(A3.hub_pinion_local())}</body>')
    def hinge(name, pos, axis, items, inner=""):
        return (f'<body name="{name}" pos="{v3(*pos)}"><joint name="{name}" type="hinge" axis="{axis}"/>{INERT}'
                f'{vis(items)}{inner}</body>')
    side = lambda sx: "l" if sx < 0 else "r"
    belts = ""                                   # stage-2 belt runs change length: box geoms resized every step
    for n, sx, k, bx in A3.stage2_runs(P3.TIP0):
        x0, x1, y0, y1, z0, z1 = bx
        belts += (f'<geom name="{belt_name(sx, k)}" type="box" pos="{v3((x0+x1)/2, (y0+y1)/2, (z0+z1)/2)}" '
                  f'size="{v3((x1-x0)/2, max(y1-y0, 0.2)/2, (z1-z0)/2)}" rgba="{MD.rgba(A3.C_BELT)}" contype="0" conaffinity="0" group="1"/>')
    mid_kids = "".join(hinge(f"ftube_{side(sx)}", A3.fshaft_at(sx), "0 1 0", A3.ftube_mid_local(sx)) for sx in A3.SIDES) \
        + "".join(hinge(f"dpul_{'ext' if k == 0 else 'ret'}_{side(sx)}", A3.dpul_at(sx, k), "1 0 0", A3.dpulley_local(sx, k))
                  for sx in A3.SIDES for k in (0, 1))
    in_kids = "".join(hinge(f"finger_{side(sx)}", A3.fshaft_at(sx), "0 1 0", A3.finger_local(sx)) for sx in A3.SIDES)
    robot = f'''
<body name="robot">
  <joint name="x" type="slide" axis="1 0 0"/><joint name="y" type="slide" axis="0 1 0"/><joint name="z" type="slide" axis="0 0 1"/>
  {INERT}{vis(A3.robot_local())}
  <body name="hub_upper" pos="{v3(0, 0, P3.D_HUB / 2)}"><joint name="hub_upper" type="hinge" axis="0 1 0"/>{INERT}{hub("upper")}</body>
  <body name="hub_lower" pos="{v3(0, 0, -P3.D_HUB / 2)}"><joint name="hub_lower" type="hinge" axis="0 1 0"/>{INERT}{hub("lower")}</body>
  {hinge("arm_drive", A3.ARM_DRIVE_AT, "1 0 0", A3.arm_drive_local())}
  {hinge("arm_idler", A3.IDLER_AT, "1 0 0", A3.idler_local())}
  {hinge("servo1", A3.SERVO1_POS, "1 0 0", A3.servo1_local())}
  {hinge("servo2", A3.FSERVO_POS, "0 1 0", A3.fservo_local())}
  <body name="turn_lift" pos="{v3(*A3.TURN_PIN_AT)}"><joint name="turn_lift" type="slide" axis="0 0 1"/>{INERT}{vis(A3.turn_carriage_local())}
    {hinge("turn_pinion", (0, 0, 0), "0 0 1", A3.turn_pinion_local())}</body>
  {"".join(hinge(f"fshaft_{side(sx)}", A3.fshaft_at(sx), "0 1 0", A3.fshaft_local(sx)) for sx in A3.SIDES)}
  {belts}
  <body name="arm_mid"><joint name="arm_mid" type="slide" axis="0 1 0"/>{INERT}{vis(A3.arm_mid_local())}{mid_kids}
    <body name="arm_in"><joint name="arm_in" type="slide" axis="0 1 0"/>{INERT}{vis(A3.arm_in_local())}{in_kids}</body>
  </body>
</body>'''
    bmesh = add(A3.basket_solid())
    baskets = ""
    for k, (b, l) in enumerate([(b, l) for b in range(P3.N_BAYS_ALL) for l in range(len(P3.LEVELS))]):
        col = A3.A.BASKET_COLS[k % 10]
        baskets += (f'<body name="basket_{b}_{l}"><freejoint name="bj_{b}_{l}"/>{INERT}'
                    f'<geom type="mesh" mesh="{bmesh}" rgba="{MD.rgba(col)}" contype="0" conaffinity="0" group="1" density="0"/></body>')
    z_top = P3.cz_for_level(len(P3.LEVELS) - 1)
    crossa = "".join(f'<position name="crossing_{j+1}_{"lower" if k == 0 else "upper"}" joint="cross{j}{k}" ctrlrange="-1.5708 0"/>'
                     for j in range(P3.N_BAYS_ALL) for k in range(2))
    act = f'''
<position name="z" joint="z" ctrlrange="{P3.CZ_HOME*MM} {z_top*MM}"/>
<position name="hub_upper_swivel" joint="hub_upper" ctrlrange="-1.5708 0"/>
<position name="hub_lower_swivel" joint="hub_lower" ctrlrange="-1.5708 0"/>{crossa}
<position name="servo1_arms" joint="arm_mid" ctrlrange="0 {(P3.TIP_PUSHED - P3.TIP0) / 2 * MM:.4f}"/>
<position name="servo2_fingers" joint="servo2" ctrlrange="-1.5708 0"/>'''
    xml = f'''<mujoco model="racklift5_v3_floor">
<compiler angle="radian" meshdir="{MD.BUILD}" autolimits="true"/>
<option gravity="0 0 0"/>
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
    path = out or os.path.join(MD.BUILD, "racklift5_v3.xml")
    open(path, "w").write(xml)
    return path


if __name__ == "__main__":
    import mujoco
    m = mujoco.MjModel.from_xml_path(build())
    print("v3 compiled:", m.nbody, "bodies", m.ngeom, "geoms", m.nu, "sliders")
