"""Minimal BSP-tree CSG (port of the csg.js algorithm) + primitives + exporters.
Pure numpy, no CAD kernel needed. Units: mm.
"""
import numpy as np, struct, math

EPS = 1e-5
COPLANAR, FRONT, BACK, SPANNING = 0, 1, 2, 3


class Poly:
    __slots__ = ("v", "n", "w")

    def __init__(self, v, n=None, w=None):
        self.v = np.asarray(v, dtype=float)
        if n is None:
            a, b, c = self.v[0], self.v[1], self.v[2]
            # robust normal (Newell) for possibly near-degenerate first triple
            vv = self.v
            nx = np.sum((vv[:, 1] - np.roll(vv[:, 1], -1)) * (vv[:, 2] + np.roll(vv[:, 2], -1)))
            ny = np.sum((vv[:, 2] - np.roll(vv[:, 2], -1)) * (vv[:, 0] + np.roll(vv[:, 0], -1)))
            nz = np.sum((vv[:, 0] - np.roll(vv[:, 0], -1)) * (vv[:, 1] + np.roll(vv[:, 1], -1)))
            n = np.array([nx, ny, nz])
            L = np.linalg.norm(n)
            n = n / L if L > 0 else np.array([0, 0, 1.0])
            w = float(n @ self.v.mean(0))
        self.n = n
        self.w = w

    def flipped(self):
        return Poly(self.v[::-1].copy(), -self.n, -self.w)


def split_poly(pn, pw, poly, cf, cb, f, b):
    t = poly.v @ pn - pw
    types = np.where(t < -EPS, BACK, np.where(t > EPS, FRONT, COPLANAR))
    ptype = 0
    for ty in np.unique(types):
        ptype |= int(ty)
    if ptype == COPLANAR:
        (cf if pn @ poly.n > 0 else cb).append(poly)
    elif ptype == FRONT:
        f.append(poly)
    elif ptype == BACK:
        b.append(poly)
    else:
        fv, bv = [], []
        V = poly.v
        k = len(V)
        for i in range(k):
            j = (i + 1) % k
            ti, tj = types[i], types[j]
            vi, vj = V[i], V[j]
            if ti != BACK:
                fv.append(vi)
            if ti != FRONT:
                bv.append(vi)
            if (ti | tj) == SPANNING:
                s = (pw - pn @ vi) / (pn @ (vj - vi))
                p = vi + (vj - vi) * s
                fv.append(p)
                bv.append(p)
        if len(fv) >= 3:
            f.append(Poly(np.array(fv), poly.n, poly.w))
        if len(bv) >= 3:
            b.append(Poly(np.array(bv), poly.n, poly.w))


class Node:
    __slots__ = ("pn", "pw", "front", "back", "polys")

    def __init__(self, polys=None):
        self.pn = None
        self.pw = None
        self.front = None
        self.back = None
        self.polys = []
        if polys:
            self.build(polys)

    def nodes(self):
        st = [self]
        out = []
        while st:
            n = st.pop()
            out.append(n)
            if n.front: st.append(n.front)
            if n.back: st.append(n.back)
        return out

    def invert(self):
        for n in self.nodes():
            n.polys = [p.flipped() for p in n.polys]
            if n.pn is not None:
                n.pn = -n.pn
                n.pw = -n.pw
            n.front, n.back = n.back, n.front

    def clip_polys(self, polys):
        res = []
        st = [(self, polys)]
        while st:
            node, ps = st.pop()
            if node.pn is None:
                res.extend(ps)
                continue
            f, b = [], []
            for p in ps:
                split_poly(node.pn, node.pw, p, f, b, f, b)
            if f:
                if node.front: st.append((node.front, f))
                else: res.extend(f)
            if b and node.back:
                st.append((node.back, b))
        return res

    def clip_to(self, bsp):
        for n in self.nodes():
            n.polys = bsp.clip_polys(n.polys)

    def all_polys(self):
        out = []
        for n in self.nodes():
            out.extend(n.polys)
        return out

    def build(self, polys):
        st = [(self, polys)]
        while st:
            node, ps = st.pop()
            if not ps:
                continue
            if node.pn is None:
                node.pn = ps[0].n.copy()
                node.pw = ps[0].w
            f, b = [], []
            for p in ps:
                split_poly(node.pn, node.pw, p, node.polys, node.polys, f, b)
            if f:
                if node.front is None: node.front = Node()
                st.append((node.front, f))
            if b:
                if node.back is None: node.back = Node()
                st.append((node.back, b))


def _bbox(polys):
    if not polys:
        return None
    V = np.vstack([p.v for p in polys])
    return V.min(0), V.max(0)


class Solid:
    def __init__(self, polys=None):
        self.polys = polys or []

    # ---- booleans ----
    def __add__(self, o): return self.union(o)
    def __sub__(self, o): return self.subtract(o)
    def __mul__(self, o): return self.intersect(o)

    def union(self, o):
        if not o.polys: return Solid(list(self.polys))
        if not self.polys: return Solid(list(o.polys))
        ba, bb = _bbox(self.polys), _bbox(o.polys)
        if np.any(ba[1] < bb[0] - 1e-6) or np.any(bb[1] < ba[0] - 1e-6):
            return Solid(self.polys + o.polys)  # disjoint fast-path
        a = Node(self.polys); b = Node(o.polys)
        a.clip_to(b); b.clip_to(a); b.invert(); b.clip_to(a); b.invert()
        a.build(b.all_polys())
        return Solid(a.all_polys())

    def subtract(self, o):
        if not o.polys or not self.polys: return Solid(list(self.polys))
        ba, bb = _bbox(self.polys), _bbox(o.polys)
        if np.any(ba[1] < bb[0] - 1e-6) or np.any(bb[1] < ba[0] - 1e-6):
            return Solid(list(self.polys))
        # only polys of self near the cutter need processing
        a = Node(self.polys); b = Node(o.polys)
        a.invert(); a.clip_to(b); b.clip_to(a); b.invert(); b.clip_to(a); b.invert()
        a.build(b.all_polys()); a.invert()
        return Solid(a.all_polys())

    def intersect(self, o):
        a = Node(self.polys); b = Node(o.polys)
        a.invert(); b.clip_to(a); b.invert(); a.clip_to(b); b.clip_to(a)
        a.build(b.all_polys()); a.invert()
        return Solid(a.all_polys())

    # ---- transforms ----
    def transform(self, M):
        M = np.asarray(M, float)
        R, t = M[:3, :3], M[:3, 3]
        det = np.linalg.det(R)
        if not self.polys:
            return Solid([])
        N = np.array([p.n for p in self.polys]) @ np.linalg.inv(R)      # plane normals map by the inverse transpose
        N /= np.linalg.norm(N, axis=1, keepdims=True)
        out = []
        for p, n in zip(self.polys, N):
            v = p.v @ R.T + t
            if det < 0:
                v = v[::-1]
            out.append(Poly(v, n, float(n @ v[0])))
        return Solid(out)

    def translate(self, x=0, y=0, z=0):
        out = []
        d = np.array([x, y, z], float)
        for p in self.polys:
            out.append(Poly(p.v + d, p.n, p.w + p.n @ d))
        return Solid(out)

    def rotate(self, axis, deg):
        return self.transform(rot(axis, deg))

    def mirror(self, axis):
        M = np.eye(4); M["xyz".index(axis), "xyz".index(axis)] = -1
        return self.transform(M)

    def bbox(self):
        return _bbox(self.polys)

    def triangles(self):
        tris = []
        for p in self.polys:
            v = p.v
            for i in range(1, len(v) - 1):
                tris.append((v[0], v[i], v[i + 1]))
        return np.array(tris) if tris else np.zeros((0, 3, 3))


def rot(axis, deg):
    a = math.radians(deg); c, s = math.cos(a), math.sin(a)
    M = np.eye(4)
    if axis == 'x': M[1:3, 1:3] = [[c, -s], [s, c]]
    elif axis == 'y': M[0, 0], M[0, 2], M[2, 0], M[2, 2] = c, s, -s, c
    else: M[0:2, 0:2] = [[c, -s], [s, c]]
    return M


def union_all(solids):
    solids = [s for s in solids if s.polys]
    while len(solids) > 1:
        nxt = []
        for i in range(0, len(solids), 2):
            nxt.append(solids[i].union(solids[i + 1]) if i + 1 < len(solids) else solids[i])
        solids = nxt
    return solids[0] if solids else Solid()


# ---------------- primitives ----------------
def box(x0, x1, y0, y1, z0, z1):
    x0, x1 = min(x0, x1), max(x0, x1); y0, y1 = min(y0, y1), max(y0, y1); z0, z1 = min(z0, z1), max(z0, z1)
    P = lambda *a: np.array(a, float)
    faces = [
        [P(x0, y0, z0), P(x0, y0, z1), P(x0, y1, z1), P(x0, y1, z0)],
        [P(x1, y0, z0), P(x1, y1, z0), P(x1, y1, z1), P(x1, y0, z1)],
        [P(x0, y0, z0), P(x1, y0, z0), P(x1, y0, z1), P(x0, y0, z1)],
        [P(x0, y1, z0), P(x0, y1, z1), P(x1, y1, z1), P(x1, y1, z0)],
        [P(x0, y0, z0), P(x0, y1, z0), P(x1, y1, z0), P(x1, y0, z0)],
        [P(x0, y0, z1), P(x1, y0, z1), P(x1, y1, z1), P(x0, y1, z1)],
    ]
    return Solid([Poly(np.array(f)) for f in faces])


def _orient(pts, axis):
    """pts are (a,b,h) with h along axis; map to xyz"""
    a, b, h = pts[..., 0], pts[..., 1], pts[..., 2]
    if axis == 'z': return np.stack([a, b, h], -1)
    if axis == 'x': return np.stack([h, a, b], -1)
    return np.stack([a, h, b], -1)  # 'y': a->x, b->z


def cyl(r, h0, h1, axis='z', c=(0, 0), seg=32, r1=None):
    """cylinder/cone along axis from h0 to h1, center c in the other two coords.
    For axis z: c=(x,y); axis x: c=(y,z); axis y: c=(x,z)"""
    r1 = r if r1 is None else r1
    t = np.linspace(0, 2 * math.pi, seg, endpoint=False)
    bot = np.stack([c[0] + r * np.cos(t), c[1] + r * np.sin(t), np.full(seg, float(h0))], -1)
    top = np.stack([c[0] + r1 * np.cos(t), c[1] + r1 * np.sin(t), np.full(seg, float(h1))], -1)
    bot, top = _orient(bot, axis), _orient(top, axis)
    polys = [Poly(bot[::-1].copy()), Poly(top.copy())]
    for i in range(seg):
        j = (i + 1) % seg
        polys.append(Poly(np.array([bot[i], bot[j], top[j], top[i]])))
    M = _orient(np.eye(3)[None], axis)[0]
    if np.linalg.det(M) < 0:
        polys = [p.flipped() for p in polys]
    return Solid(polys)


def _area2(pts):
    x, y = pts[:, 0], pts[:, 1]
    return 0.5 * np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y)


def ear_clip(pts):
    pts = [tuple(p) for p in pts]
    idx = list(range(len(pts)))
    if _area2(np.array(pts)) < 0:
        idx = idx[::-1]
    tris = []
    P = np.array(pts)

    def inside(p, a, b, c):
        def s(p1, p2, p3): return (p1[0] - p3[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p3[1])
        d1, d2, d3 = s(p, a, b), s(p, b, c), s(p, c, a)
        neg = (d1 < -1e-12) or (d2 < -1e-12) or (d3 < -1e-12)
        pos = (d1 > 1e-12) or (d2 > 1e-12) or (d3 > 1e-12)
        return not (neg and pos)
    guard = 0
    while len(idx) > 3 and guard < 100000:
        guard += 1
        n = len(idx)
        found = False
        for k in range(n):
            i0, i1, i2 = idx[(k - 1) % n], idx[k], idx[(k + 1) % n]
            a, b, c = P[i0], P[i1], P[i2]
            cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
            if cross <= 1e-12:
                continue
            ok = True
            for m in idx:
                if m in (i0, i1, i2): continue
                if inside(P[m], a, b, c):
                    ok = False; break
            if ok:
                tris.append((i0, i1, i2)); idx.pop(k); found = True; break
        if not found:
            # degenerate; drop a vertex
            idx.pop(0)
    if len(idx) == 3:
        tris.append(tuple(idx))
    return tris


def extrude(pts2d, h0, h1, axis='z'):
    """Extrude a simple polygon (list of (a,b)) between h0..h1 along axis."""
    P = np.asarray(pts2d, float)
    if _area2(P) < 0:
        P = P[::-1]
    n = len(P)
    tris = ear_clip(P)
    bot = np.column_stack([P, np.full(n, float(h0))])
    top = np.column_stack([P, np.full(n, float(h1))])
    B, T = _orient(bot, axis), _orient(top, axis)
    # determine handedness of mapping
    M = _orient(np.eye(3)[None], axis)[0]
    det = np.linalg.det(M)
    polys = []
    for (i, j, k) in tris:
        t = np.array([T[i], T[j], T[k]]); b = np.array([B[k], B[j], B[i]])
        if det < 0: t, b = t[::-1], b[::-1]
        polys += [Poly(t), Poly(b)]
    for i in range(n):
        j = (i + 1) % n
        q = np.array([B[i], B[j], T[j], T[i]])
        if det < 0: q = q[::-1]
        polys.append(Poly(q))
    return Solid(polys)


def slot(p0, p1, r, h0, h1, axis='z', seg=24):
    """stadium between 2D points p0,p1 (coords as for extrude on that axis) radius r, extruded h0..h1"""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0; L = np.linalg.norm(d)
    ang = math.atan2(d[1], d[0]) if L > 0 else 0.0
    t1 = np.linspace(ang + math.pi / 2, ang + 3 * math.pi / 2, seg // 2 + 1)
    t2 = np.linspace(ang - math.pi / 2, ang + math.pi / 2, seg // 2 + 1)
    pts = [(p0[0] + r * math.cos(a), p0[1] + r * math.sin(a)) for a in t1] + \
          [(p1[0] + r * math.cos(a), p1[1] + r * math.sin(a)) for a in t2]
    return extrude(pts, h0, h1, axis)


def hexagon(af, z0, z1, c=(0, 0), axis='z', rot_deg=0):
    R = af / math.sqrt(3)
    pts = [(c[0] + R * math.cos(math.radians(60 * i + rot_deg)), c[1] + R * math.sin(math.radians(60 * i + rot_deg))) for i in range(6)]
    return extrude(pts, z0, z1, axis)


# ---------------- export ----------------
def write_stl(solid_or_tris, path, name=b"part"):
    tris = solid_or_tris.triangles() if isinstance(solid_or_tris, Solid) else solid_or_tris
    with open(path, "wb") as f:
        f.write(name.ljust(80, b" ")[:80])
        f.write(struct.pack("<I", len(tris)))
        if len(tris) == 0: return
        n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
        L = np.linalg.norm(n, axis=1, keepdims=True); L[L == 0] = 1
        n = n / L
        rec = np.zeros(len(tris), dtype=[("n", "<f4", 3), ("v", "<f4", 9), ("a", "<u2")])
        rec["n"] = n; rec["v"] = tris.reshape(-1, 9)
        f.write(rec.tobytes())


def to_mesh(solid, tol=1e-4, ptol=2e-3):
    """Weld vertices, insert T-junction vertices on edges, fan-triangulate from polygon centroid.
    Returns (V, F) with a manifold-friendly topology."""
    polys = [p.v for p in solid.polys if len(p.v) >= 3]
    allv = np.vstack(polys)
    key = np.round(allv / tol).astype(np.int64)
    uniq, inv = np.unique(key, axis=0, return_inverse=True)
    inv = inv.ravel()
    V = np.zeros((len(uniq), 3)); cnt = np.zeros(len(uniq))
    np.add.at(V, inv, allv); np.add.at(cnt, inv, 1); V /= cnt[:, None]
    idx_polys = []; k = 0
    for p in polys:
        ids = list(inv[k:k + len(p)]); k += len(p)
        # drop consecutive duplicates
        ids2 = [i for j, i in enumerate(ids) if i != ids[j - 1]]
        if len(ids2) >= 3: idx_polys.append(ids2)
    order = np.argsort(V[:, 0]); xs = V[order, 0]
    cache = {}
    def on_edge(a, b):
        kk = (a, b) if a < b else (b, a)
        if kk in cache:
            r = cache[kk]; return r if a < b else r[::-1]
        pa, pb = V[kk[0]], V[kk[1]]
        lo, hi = min(pa[0], pb[0]) - 1e-3, max(pa[0], pb[0]) + 1e-3
        i0, i1 = np.searchsorted(xs, lo), np.searchsorted(xs, hi)
        cand = order[i0:i1]
        cand = cand[(cand != kk[0]) & (cand != kk[1])]
        res = []
        if len(cand):
            d = pb - pa; L2 = d @ d
            P = V[cand] - pa
            t = P @ d / L2
            perp = np.linalg.norm(P - np.outer(t, d), axis=1)
            m = (t > 1e-6) & (t < 1 - 1e-6) & (perp < ptol)
            sel = cand[m]; ts = t[m]
            res = list(sel[np.argsort(ts)])
        cache[kk] = res
        return res if a < b else res[::-1]
    F = []; extra = []
    nV = len(V)
    for ids in idx_polys:
        ring = []
        for j in range(len(ids)):
            a, b = ids[j], ids[(j + 1) % len(ids)]
            ring.append(a); ring.extend(on_edge(a, b))
        if len(ring) == 3:
            F.append(ring); continue
        c = V[ring].mean(0)
        ci = nV + len(extra); extra.append(c)
        for j in range(len(ring)):
            F.append([ci, ring[j], ring[(j + 1) % len(ring)]])
    if extra: V = np.vstack([V, np.array(extra)])
    return V, np.array(F)


def mesh_stats(V, F):
    e = np.sort(np.vstack([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]]), axis=1)
    _, c = np.unique(e, axis=0, return_counts=True)
    return {"verts": len(V), "tris": len(F), "edges_not_2": int(np.sum(c != 2))}


def write_stl_mesh(V, F, path, name=b"part"):
    write_stl(V[F], path, name)


def extrude_ring(outer, inner, h0, h1):
    """Extrude region between two star-shaped (about origin) loops along z. Clean, CSG-free."""
    def prep(pts):
        P = np.asarray(pts, float)
        a = np.arctan2(P[:, 1], P[:, 0])
        k = int(np.argmin(a)); P = np.roll(P, -k, 0); a = np.roll(a, -k)
        a = np.unwrap(a)
        if a[-1] < a[0]: P = P[::-1]; a = np.unwrap(np.arctan2(P[:, 1], P[:, 0]))
        a = a - 0  # increasing
        return P, a
    O, ao = prep(outer); I, ai = prep(inner)
    no, ni = len(O), len(I)
    tris2 = []  # entries: ('o',i) / ('i',j)
    i = j = 0
    # extend angles periodically
    while i < no or j < ni:
        oa = ao[i % no] + 2 * np.pi * (i // no); ia = ai[j % ni] + 2 * np.pi * (j // ni)
        on = ao[(i + 1) % no] + 2 * np.pi * ((i + 1) // no); inn = ai[(j + 1) % ni] + 2 * np.pi * ((j + 1) // ni)
        if (j >= ni) or (i < no and on <= inn):
            tris2.append((('o', i % no), ('o', (i + 1) % no), ('i', j % ni))); i += 1
        else:
            tris2.append((('o', i % no), ('i', (j + 1) % ni), ('i', j % ni))); j += 1
    def P3(t, h):
        return np.array([*(O[t[1]] if t[0] == 'o' else I[t[1]]), h])
    polys = []
    for a, b, c in tris2:
        polys.append(Poly(np.array([P3(a, h1), P3(b, h1), P3(c, h1)])))
        polys.append(Poly(np.array([P3(c, h0), P3(b, h0), P3(a, h0)])))
    for k in range(no):
        a, b = O[k], O[(k + 1) % no]
        polys.append(Poly(np.array([[*a, h0], [*b, h0], [*b, h1], [*a, h1]])))
    for k in range(ni):
        a, b = I[k], I[(k + 1) % ni]
        polys.append(Poly(np.array([[*b, h0], [*a, h0], [*a, h1], [*b, h1]])))
    return Solid(polys)
