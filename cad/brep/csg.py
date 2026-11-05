"""Drop-in B-rep replacement for RackLift's mesh csg.py, built on build123d / OpenCascade.

Same public API (box, cyl, extrude, extrude_ring, slot, hexagon, union_all, rot, Solid with
+ - * union/subtract/intersect, translate/rotate/transform/mirror, .polys) so the unchanged
parts.py / parts_v2.py / assembly_v2.py produce exact solids instead of triangle meshes:
  * cylinders / cones / slots are true analytic surfaces (the `seg` argument is ignored)
  * star-shaped rings whose inner loop is a (D-)circle get an exact circular / D bore
  * gear outlines from involute_profile() are rebuilt as smooth involute splines (see gears.py)
Units: mm.
"""
import math
import numpy as np
import build123d as bd
from OCP.gp import gp_Trsf, gp_Ax2, gp_Pnt, gp_Dir
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform

TOL = 1e-6


# ----------------------------------------------------------------- helpers
def _orient_xyz(a, b, h, axis):
    """csg.py convention: axis z: (a,b,h)=(x,y,z); axis x: (y,z,x); axis y: (x,z,y)"""
    if axis == 'z':
        return (a, b, h)
    if axis == 'x':
        return (h, a, b)
    return (a, h, b)


def _plane(axis, h):
    """build123d Plane whose local (u, v) = the csg 2-D coords and whose normal points to +axis"""
    if axis == 'z':
        return bd.Plane(origin=(0, 0, h), x_dir=(1, 0, 0), z_dir=(0, 0, 1))
    if axis == 'x':
        return bd.Plane(origin=(h, 0, 0), x_dir=(0, 1, 0), z_dir=(1, 0, 0))   # u=y, v=z
    # axis y: u=x, v=z -> x_dir=(1,0,0), y_dir=(0,0,1) => normal = x × z = -y
    return bd.Plane(origin=(0, h, 0), x_dir=(1, 0, 0), z_dir=(0, -1, 0))


def _prism(face2d_builder, h0, h1, axis):
    """face2d_builder(plane) -> Face on that plane; extrude it from h0 to h1 along +axis"""
    h0, h1 = min(h0, h1), max(h0, h1)
    pl = _plane(axis, h0)
    face = face2d_builder(pl)
    d = (h1 - h0)
    vec = {'x': (d, 0, 0), 'y': (0, d, 0), 'z': (0, 0, d)}[axis]
    return bd.Solid.extrude(face, bd.Vector(*vec))


def _poly_face(pl, pts):
    pts = _clean(pts)
    w = bd.Wire.make_polygon([pl.from_local_coords((float(u), float(v))) for u, v in pts], close=True)
    return bd.Face(w)


def _clean(pts):
    out = []
    for p in pts:
        p = (float(p[0]), float(p[1]))
        if not out or abs(out[-1][0] - p[0]) > 1e-7 or abs(out[-1][1] - p[1]) > 1e-7:
            out.append(p)
    if len(out) > 2 and abs(out[0][0] - out[-1][0]) < 1e-7 and abs(out[0][1] - out[-1][1]) < 1e-7:
        out.pop()
    return out


def _fix(shape):
    """return a single clean Solid (or Compound of solids) with coplanar faces merged"""
    if shape is None:
        return None
    if isinstance(shape, list):            # some build123d booleans (intersect) return a ShapeList
        shape = shape[0] if len(shape) == 1 else bd.Compound(children=list(shape))
    try:
        shape = shape.clean()
    except Exception:
        pass
    return shape


# ----------------------------------------------------------------- Solid wrapper
class Solid:
    """wraps a build123d Shape. `.polys` returns [self] so code that concatenates polys
    (Solid(a.polys + b.polys)) still works - the constructor fuses whatever it is given."""

    def __init__(self, polys=None, shape=None):
        if shape is not None:
            self.shape = shape
            return
        parts = [p for p in (polys or []) if isinstance(p, Solid) and p.shape is not None]
        if not parts:
            self.shape = None
        elif len(parts) == 1:
            self.shape = parts[0].shape
        else:
            self.shape = _fix(parts[0].shape.fuse(*[p.shape for p in parts[1:]]))

    @property
    def polys(self):
        return [self] if self.shape is not None else []

    # booleans
    def __add__(self, o): return self.union(o)
    def __sub__(self, o): return self.subtract(o)
    def __mul__(self, o): return self.intersect(o)

    def union(self, o):
        if o.shape is None: return Solid(shape=self.shape)
        if self.shape is None: return Solid(shape=o.shape)
        return Solid(shape=_fix(self.shape.fuse(o.shape)))

    def subtract(self, o):
        if o.shape is None or self.shape is None: return Solid(shape=self.shape)
        return Solid(shape=_fix(self.shape.cut(o.shape)))

    def intersect(self, o):
        if o.shape is None or self.shape is None: return Solid()
        return Solid(shape=_fix(self.shape.intersect(o.shape)))

    # transforms
    def transform(self, M):
        if self.shape is None: return Solid()
        M = np.asarray(M, float)
        R, t = M[:3, :3], M[:3, 3]
        s = self.shape
        if np.linalg.det(R) < 0:                       # mirror in x first, then a proper rotation
            s = s.mirror(bd.Plane.YZ)
            R = R @ np.diag([-1.0, 1.0, 1.0])
        tr = gp_Trsf()
        tr.SetValues(*R[0], t[0], *R[1], t[1], *R[2], t[2])
        out = BRepBuilderAPI_Transform(s.wrapped, tr, True).Shape()
        return Solid(shape=bd.Shape.cast(out) if hasattr(bd.Shape, "cast") else type(s)(out))

    def translate(self, x=0, y=0, z=0):
        if self.shape is None: return Solid()
        return Solid(shape=self.shape.moved(bd.Location((x, y, z))))

    def rotate(self, axis, deg):
        return self.transform(rot(axis, deg))

    def mirror(self, axis):
        M = np.eye(4); M["xyz".index(axis), "xyz".index(axis)] = -1
        return self.transform(M)

    def bbox(self):
        b = self.shape.bounding_box()
        return np.array([b.min.X, b.min.Y, b.min.Z]), np.array([b.max.X, b.max.Y, b.max.Z])

    @property
    def volume(self):
        return self.shape.volume if self.shape is not None else 0.0


def rot(axis, deg):
    a = math.radians(deg); c, s = math.cos(a), math.sin(a)
    M = np.eye(4)
    if axis == 'x': M[1:3, 1:3] = [[c, -s], [s, c]]
    elif axis == 'y': M[0, 0], M[0, 2], M[2, 0], M[2, 2] = c, s, -s, c
    else: M[0:2, 0:2] = [[c, -s], [s, c]]
    return M


def union_all(solids):
    solids = [s for s in solids if s.shape is not None]
    if not solids:
        return Solid()
    if len(solids) == 1:
        return solids[0]
    return Solid(shape=_fix(solids[0].shape.fuse(*[s.shape for s in solids[1:]])))


# ----------------------------------------------------------------- primitives
def box(x0, x1, y0, y1, z0, z1):
    x0, x1 = sorted((x0, x1)); y0, y1 = sorted((y0, y1)); z0, z1 = sorted((z0, z1))
    b = bd.Solid.make_box(x1 - x0, y1 - y0, z1 - z0, bd.Plane(origin=(x0, y0, z0)))
    return Solid(shape=b)


def cyl(r, h0, h1, axis='z', c=(0, 0), seg=32, r1=None):
    """true cylinder / cone along axis from h0 to h1, centre c in the other two coords
    (axis z: c=(x,y); axis x: c=(y,z); axis y: c=(x,z)). `seg` is ignored."""
    r1 = r if r1 is None else r1
    if seg <= 8 and abs(r - r1) < 1e-9:      # deliberate polygon (e.g. seg=6 hex nut trap)
        t = [2 * math.pi * i / seg for i in range(seg)]
        return extrude([(c[0] + r * math.cos(a), c[1] + r * math.sin(a)) for a in t], h0, h1, axis)
    if h1 < h0:
        h0, h1, r, r1 = h1, h0, r1, r
    base = bd.Vector(*_orient_xyz(c[0], c[1], h0, axis))
    d = {'x': (1, 0, 0), 'y': (0, 1, 0), 'z': (0, 0, 1)}[axis]
    pl = bd.Plane(origin=base, z_dir=d)
    if abs(r - r1) < 1e-9:
        s = bd.Solid.make_cylinder(r, h1 - h0, pl)
    else:
        s = bd.Solid.make_cone(r, r1, h1 - h0, pl)
    return Solid(shape=s)


def extrude(pts2d, h0, h1, axis='z'):
    return Solid(shape=_prism(lambda pl: _poly_face(pl, pts2d), h0, h1, axis))


def slot(p0, p1, r, h0, h1, axis='z', seg=24):
    """true stadium (two arcs + two lines) between 2-D points p0, p1 with radius r"""
    p0 = np.asarray(p0, float); p1 = np.asarray(p1, float)
    L = float(np.linalg.norm(p1 - p0))
    if L < 1e-9:
        return cyl(r, h0, h1, axis, tuple(p0))
    def build(pl):
        d = (p1 - p0) / L; n = np.array([-d[1], d[0]])
        P = lambda q: pl.from_local_coords((float(q[0]), float(q[1])))
        a, b = p0 + n * r, p1 + n * r
        e, f = p1 - n * r, p0 - n * r
        w = bd.Wire([bd.Edge.make_line(P(a), P(b)),
                     bd.Edge.make_three_point_arc(P(b), P(p1 + d * r), P(e)),
                     bd.Edge.make_line(P(e), P(f)),
                     bd.Edge.make_three_point_arc(P(f), P(p0 - d * r), P(a))])
        return bd.Face(w)
    return Solid(shape=_prism(build, h0, h1, axis))


def hexagon(af, z0, z1, c=(0, 0), axis='z', rot_deg=0):
    R = af / math.sqrt(3)
    pts = [(c[0] + R * math.cos(math.radians(60 * i + rot_deg)), c[1] + R * math.sin(math.radians(60 * i + rot_deg))) for i in range(6)]
    return extrude(pts, z0, z1, axis)


# ----------------------------------------------------------------- rings (gears, bores)
def _loop_face(pl, pts):
    """exact face for a closed 2-D loop: a tagged gear profile, a circle, a D-circle, or a polygon"""
    g = getattr(pts, "gear", None)
    if g is not None:
        from gears import gear_face
        return gear_face(pl, **g)
    P = np.asarray(pts, float)
    r = np.hypot(P[:, 0], P[:, 1])
    c = P.mean(0)
    if np.hypot(*c) < 0.05 * r.max() and r.max() - r.min() < 1e-6 * max(1.0, r.max()):
        return bd.Face(bd.Wire([bd.Edge.make_circle(float(r.max()), pl)]))
    # D-bore: points on a circle except a run clipped to a straight flat (y = const)
    R = float(np.max(r))
    on_c = np.abs(r - R) < 1e-6 * max(1.0, R)
    if on_c.sum() >= len(P) * 0.6:
        flat_y = float(P[~on_c, 1].max()) if (~on_c).any() else None
        if flat_y is not None and np.allclose(P[~on_c, 1], flat_y, atol=1e-6) and abs(flat_y) < R:
            disc = bd.Face(bd.Wire([bd.Edge.make_circle(R, pl)]))
            hp = _poly_face(pl, [(-2 * R, -2 * R), (2 * R, -2 * R), (2 * R, flat_y), (-2 * R, flat_y)])
            return disc.intersect(hp).faces()[0]
    return _poly_face(pl, pts)


def extrude_ring(outer, inner, h0, h1):
    """region between outer and inner loops (both around the origin), extruded along z"""
    def build(pl):
        fo = _loop_face(pl, outer)
        fi = _loop_face(pl, inner)
        return fo.cut(fi).faces()[0]
    return Solid(shape=_prism(build, h0, h1, 'z'))
