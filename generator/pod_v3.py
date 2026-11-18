"""The Option 1 swivel pod (cad/build_t_slot_option1.py) as the v3 robot's hub - built from that script, so edits there carry over.

Frames.  Pod frame (build_t_slot_option1.py): rail along X at 0 deg, Y across it, Z from the rail up to the pod, plate bottom
Z = 0, rail centre Z = -12.475, swivel about Z through (0, 0), carriage turns +theta.  v3 hub frame (assembly_v3.py): rail along
z when the hub is vertical, x across, rail centre y = -50, robot at -y, hub turns rot('y', h) with h = 0 vertical, -90 horizontal.

    x = -Y,   y = -62.475 - Z,   z = X          (proper rotation + offset; rail centre -12.475 -> -50)
    pod +theta about Z  ==  rot('y', -theta)    ->  h = -theta
    the 20T servo gear turns -2 theta about Z   ==  rot('y', -2 h) about its axis at z = SERVO_PIVOT_X

The climb pinion lands at y = -67.8, z = 52.5 out along the rail - the same pitch line as the old v3 hub (CP_Y = -67.8).
"""
import hashlib
import os
import sys

import numpy as np
import csg

HERE = os.path.dirname(os.path.abspath(__file__))
CAD = os.path.join(HERE, "..", "cad")
CACHE = os.path.join(HERE, "..", "sim", "build", "pod_v3")
SOURCES = ("build_t_slot_option1.py", "build_t_slot_swivel_carriage.py", "build_openbuilds_gantry.py",
           "build_4632_370.py", "build_4632_370_kit.py", "build_td8125mg.py")
BREP = hasattr(csg, "_fix")                     # the B-rep backend (cad/brep/csg.py) is loaded instead of the mesh one

POD_TO_HUB = np.array([[0.0, -1.0, 0.0, 0.0],
                       [0.0, 0.0, -1.0, -62.475],
                       [1.0, 0.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0, 1.0]])
PIN_AXIS_POD = (52.5, 0.0, 5.325)               # climb pinion axis in the pod frame (T.PINION_X, 0, T.PINION_Z)
SERVO_Z = -37.5                                 # 20T servo gear axis: pod X = SERVO_PIVOT_X -> hub z

# pod part -> (v3 name, colour, role).  role: turn = turns with the hub, fixed = on the column, gear = servo gear set
PARTS = {
    "gantry_plate":  ("pod: MT0134 gantry plate (hub plate)", (40, 42, 48), "turn"),
    "v_wheels":      ("pod: V wheel stacks (4 x Delrin, spacers, M5)", (60, 64, 72), "turn"),
    "carriage_pod":  ("pod: printed carriage pod + 40T swivel gear", (46, 142, 230), "turn"),
    "motor_cap":     ("pod: printed motor cap (removable lid + can sleeve)", (27, 74, 153), "turn"),
    "pinion_cover":  ("pod: printed pinion cover (removable)", (143, 208, 255), "turn"),
    "motor_housing": ("pod: 4632-370 climb motor", (180, 188, 200), "turn"),
    "motor_shaft":   ("pod: 4632-370 output shaft", (216, 222, 230), "turn"),
    "retainer":      ("pod: printed shaft retainer", (51, 57, 66), "turn"),
    "body_mount":    ("pod: printed body mount (bolts to the column)", (51, 184, 115), "fixed"),
    "bearing_bot":   ("pod: 608ZZ bearing (lower)", (216, 222, 230), "fixed"),
    "bearing_top":   ("pod: 608ZZ bearing (upper)", (216, 222, 230), "fixed"),
    "m8_hardware":   ("pod: M8 x 40 pivot bolt + spacer + nyloc", (100, 116, 139), "fixed"),
    "servo_body":    ("pod: TD-8125MG swivel servo", (196, 30, 36), "fixed"),
    "gear_20t":      ("pod: printed 20T swivel gear", (245, 158, 11), "gear"),
    "alum_horn":     ("pod: 25T aluminium servo horn", (224, 64, 64), "gear"),
    "servo_spline":  ("pod: TD-8125MG output spline", (212, 173, 82), "gear"),
    "fasteners_turn":  ("pod: screws on the carriage pod (4 x M5 x 12 button head + nyloc, 4 x M3 x 6 button head)", (100, 116, 139), "turn"),
    "fasteners_fixed": ("pod: screws on the body mount (4 x M3 x 14 PT servo ears, 4 x M4 x 12 socket cap to the column)", (100, 116, 139), "fixed"),
    "fasteners_gear":  ("pod: screws on the 20T gear (4 x M3 x 12 socket cap + M3 spline screw)", (100, 116, 139), "gear"),
}
PINION_NAME = "printed climber pinion"          # (as the old hub: the clearance check skips it against the racks it meshes)


def _key():
    h = hashlib.sha1()
    for f in SOURCES:
        with open(os.path.join(CAD, f), "rb") as fh:
            h.update(fh.read())
    return h.hexdigest()[:12]


_SHAPES = None


def _shapes():
    """build123d shapes of every pod part in the pod frame (straight from build_t_slot_option1.py)"""
    global _SHAPES
    if _SHAPES is None:
        sys.path.insert(0, CAD)
        import build_t_slot_option1 as O
        _SHAPES = O.build_assembled_parts()
    return _SHAPES


def _tris(name):
    """triangles (n, 3, 3) of a pod part in the pod frame, cached per version of the CAD scripts"""
    path = os.path.join(CACHE, f"{_key()}_{name}.npy")
    if os.path.exists(path):
        return np.load(path)
    s = _shapes()[name]
    verts, faces = s.tessellate(0.05, 0.25)
    V = np.array([(v.X, v.Y, v.Z) for v in verts])
    T = V[np.array(faces)]
    os.makedirs(CACHE, exist_ok=True)
    np.save(path, T)
    return T


def _solid(name, M):
    """the pod part `name` as a csg Solid of the loaded backend, transformed by M (4 x 4, pod frame -> wanted frame)"""
    if BREP:
        return csg.Solid(shape=_shapes()[name]).transform(M)
    T = _tris(name)
    V = np.c_[T.reshape(-1, 3), np.ones(len(T) * 3)] @ M.T
    V = V[:, :3].reshape(-1, 3, 3)
    return csg.Solid([csg.Poly(t) for t in V if np.linalg.norm(np.cross(t[1] - t[0], t[2] - t[0])) > 1e-9])


def _T(x=0.0, y=0.0, z=0.0):
    M = np.eye(4)
    M[:3, 3] = (x, y, z)
    return M


def hub_items():
    """parts that turn with the hub (hub frame, rail along z at h = 0) - without the pinion"""
    return [(n, _solid(k, POD_TO_HUB), c) for k, (n, c, r) in PARTS.items() if r == "turn"]


def pinion_items():
    """the climb pinion centred on its own axle (axis x in the hub frame), for hub_pinion_local()"""
    return [(PINION_NAME, _solid("pinion", POD_TO_HUB @ _T(*[-v for v in PIN_AXIS_POD])), (235, 122, 36))]


def pinion_at():
    p = POD_TO_HUB @ np.array([*PIN_AXIS_POD, 1.0])
    return tuple(float(v) for v in p[:3])


def fixed_items():
    """parts on the column (hub frame at h = 0; the robot places them at the hub, turned by the hub's base angle)"""
    return [(n, _solid(k, POD_TO_HUB), c) for k, (n, c, r) in PARTS.items() if r == "fixed"]


def gear_items():
    """the servo gear set centred on its own axis (axis y in the hub frame); placed at (0, 0, SERVO_Z), turned rot('y', -2 h)"""
    return [(n, _solid(k, _T(0, 0, -SERVO_Z) @ POD_TO_HUB), c) for k, (n, c, r) in PARTS.items() if r == "gear"]
