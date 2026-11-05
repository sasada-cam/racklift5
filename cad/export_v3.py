"""RackLift5 v3 (v2 machine, Squid-style two-hub body) -> exact B-rep CAD (build123d / OpenCascade).  v2 files in models/ are left untouched.

    python cad/export_v3.py            # new printed parts + 4 assembly poses (pick, swivel, travel, transfer, dock)
    python cad/export_v3.py parts      # only the new printed parts
    python cad/export_v3.py pick       # one pose: pick | swivel | travel | transfer | dock

Output: models/v3/print/*.step|.stl, models/v3/design/*.step, models/v3/assembly/RackLift5_v3_<pose>.step
Geometry source: generator/params_v3.py + generator/assembly_v3.py (the same source the v3 MuJoCo model uses).
"""
import os, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "brep"))
sys.path.insert(0, HERE)
import racklift_brep as R          # loads the B-rep csg backend + v2 modules
import assembly_v3 as A3           # noqa: E402  (picks up the B-rep csg)
import export_brep as E            # noqa: E402  (helpers: to_bed, colours, grouping)
import build123d as bd             # noqa: E402

OUT = os.path.abspath(os.path.join(HERE, "..", "models", "v3"))
GROUP_LABEL = {"static": "Rack + rails (fixed)", "rotor": "Crossing turntables + transfer cart", "cart": "Transfer cart (moving)", "jack0": "Dock 1 jack (moving)", "jack1": "Dock 2 jack (moving)",
               "jack2": "Dock 3 jack (moving)", "robot": "Robot: column, battery, tray",
               "hub1": "Upper hub", "hub2": "Lower hub", "arm_mid": "Telescopic arms (middle)",
               "arm_in": "Telescopic arms (inner)", "finger": "Flip fingers + inner square tubes",
               "belt": "Stage-2 belt runs"}
I = R.I


def _pick(items, name):
    return [s for n, s, c in items if n == name][0]


NEW_PARTS = {   # name: (builder, print orientation, qty)
    "V01_hub_cover": (lambda: _pick(A3.hub_local(), "printed hub cover"), R.RX(90), 2),
    "V02_column": (lambda: _pick(A3.robot_local(), "printed column"), R.RX(90), 1),
    "V03_flip_finger": (lambda: _pick(A3.finger_local(1, "front"), "printed flip finger (front)"), I, 4),
    "V04_arm_post": (lambda: _pick(A3.robot_local(), "printed arm post"), I, 2),
    "V05_crossing_bracket": (lambda: _pick(A3.row_static_local(), "printed crossing bracket"), R.RX(90), 2 * A3.N_BAYS_ALL),
    "V06_front_post": (lambda: _pick(A3.robot_local(), "printed front post (rod + cross-shaft bearings)"), R.RX(90), 2),
    "V07_rear_arm_post": (lambda: _pick(A3.robot_local(), "printed arm post (rear, idler axle)"), I, 2),
    "V08_stage1_belt_clamp": (lambda: _pick(A3.arm_mid_local(), "printed stage-1 belt clamp (middle bar)"), R.RX(90), 2),
    "V09_inner_bar_bearing": (lambda: _pick(A3.arm_in_local(), "finger shaft bearing (inner bar)"), R.RX(90), 4),
    "V13_transfer_pinion_16T": (lambda: _pick(A3.turn_pinion_local(), "turn drive pinion 16T (robot)"), I, 1),
    "V15_cart_trolley_plate": (lambda: _pick(A3.cart_local(), "cart trolley plate"), R.RY(90), 2),
    "V16_lift_carriage": (lambda: _pick(A3.turn_carriage_local(), "printed lift carriage"), I, 1),
    "V14_turn_drive_mast": (lambda: _pick(A3.robot_local(), "printed turn-drive mast"), R.RX(90), 1),
    "V10_stage2_pulley": (lambda: _pick(A3.dpulley_local(1, 0), "stage-2 pulley (extend)"), R.RY(90), 4),
    # locks (cart: one docking bar; robot: two solenoid brackets)
    "V24_cart_docking_bar": (lambda: _pick(A3.cart_local(), "cart docking bar (printed, with the bolt socket)"), I, 1),
    "V27_cart_lock_socket_funnel": (lambda: _pick(A3.row_static_local(), "cart lock socket (printed, funnel lead-in, on the lower rail-end arm)"), I, A3.N_ROWS),
    "V25_bolt_solenoid_bracket": (lambda: _pick(A3.robot_local(), "lock solenoid bracket (docking bolt)"), I, 1),
    "V26_release_solenoid_bracket": (lambda: _pick(A3.robot_local(), "lock solenoid bracket (release plunger)"), I, 1),
    # dock jack (per dock; x3 docks): printed parts of the scissor lift - plates, arms, shafts and the stepper are bought
    "V17_jack_bottom_lug": (lambda: _pick(A3.jack_static_local(), "jack: bottom pivot lug"), I, 2 * A3.N_ROWS),
    "V18_jack_top_lug": (lambda: _pick(A3.jack_platform_local(), "jack: top pivot lug"), I, 2 * A3.N_ROWS),
    "V19_jack_bearing_block": (lambda: _pick(A3.jack_static_local(), "jack: screw bearing block (printed)"), I, A3.N_ROWS),
    "V20_jack_motor_bracket": (lambda: _pick(A3.jack_static_local(), "jack: motor bracket (printed)"), I, A3.N_ROWS),
    "V21_jack_slider_shoe": (lambda: _pick(A3.jack_slider_local(), "jack: slider shoe (printed)"), I, 2 * A3.N_ROWS),
    "V22_jack_lead_nut_block": (lambda: _pick(A3.jack_slider_local(), "jack: lead nut block (printed)"), I, A3.N_ROWS),
    "V23_jack_coupling_flag": (lambda: _pick(A3.jack_screw_local(), "jack: shaft coupling + index flag"), R.RY(90), A3.N_ROWS),
}

def export_parts():
    dp, dd = E.ensure(os.path.join(OUT, "print")), E.ensure(os.path.join(OUT, "design"))
    for name, (fn, M, qty) in NEW_PARTS.items():
        t = time.time()
        s = fn()
        sh = s.shape; sh.label = name
        bd.export_step(sh, os.path.join(dd, name + ".step"))
        p = E.to_bed(s, M).shape; p.label = name
        bd.export_step(p, os.path.join(dp, name + ".step"))
        bd.export_stl(p, os.path.join(dp, name + ".stl"), tolerance=0.005, angular_tolerance=0.1)
        print(f"{name:20s} x{qty}  valid={sh.is_valid}  {sh.volume/1000:.1f} cm3  {time.time()-t:.1f}s", flush=True)


def export_pose(pose):
    t = time.time()
    items = A3.pose(pose)
    groups, count = {}, {}
    for name, s, col, g in items:
        if s.shape is None:
            continue
        sh = s.shape
        count[name] = count.get(name, 0) + 1
        sh.label = f"{name} {count[name]}" if sum(1 for n, *_ in items if n == name) > 1 else name
        sh.color = bd.Color(col[0] / 255, col[1] / 255, col[2] / 255)
        groups.setdefault(g, []).append(sh)
    asm = bd.Compound(children=[bd.Compound(children=v, label=GROUP_LABEL.get(g, g)) for g, v in groups.items()],
                      label=f"RackLift5_v3_{pose}")
    path = os.path.join(E.ensure(os.path.join(OUT, "assembly")), f"RackLift5_v3_{pose}.step")
    bd.export_step(asm, path)
    print(f"assembly {pose}: {sum(len(v) for v in groups.values())} bodies -> {path}  ({time.time()-t:.1f}s)", flush=True)


if __name__ == "__main__":
    what = sys.argv[1:] or ["parts", *A3.POSES]
    if "parts" in what:
        export_parts()
    for p in A3.POSES:
        if p in what:
            export_pose(p)
