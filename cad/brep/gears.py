"""Exact involute spur-gear outline (splines + arcs) matching parts.involute_profile()'s geometry."""
import math
import build123d as bd


def _th(r, z, rb, pa):
    inv = lambda a: math.tan(a) - a
    ar = math.acos(min(1.0, rb / r))
    return math.pi / (2 * z) + inv(math.radians(pa)) - inv(ar)


def gear_face(pl, z, m, pa, **_):
    rp = m * z / 2; rb = rp * math.cos(math.radians(pa)); ra = rp + m; rf = rp - 1.25 * m
    P = lambda r, a: pl.from_local_coords((r * math.cos(a), r * math.sin(a)))
    th = lambda r: _th(r, z, rb, pa)
    r0 = max(rb, rf)                       # where the involute starts
    radial = rb > rf                       # small radial line from the root circle up to the base circle
    tb = th(rb)
    n = 14
    radii = [r0 + (ra - r0) * (i / n) ** 1.3 for i in range(n + 1)]
    edges = []
    for k in range(z):
        ph = 2 * math.pi * k / z
        nph = 2 * math.pi * (k + 1) / z
        if radial:
            a_root_l = ph - tb - 0.02
            edges.append(bd.Edge.make_line(P(rf, a_root_l), P(rb, ph - tb)))
        edges.append(bd.Edge.make_spline([P(r, ph - th(r)) for r in radii]))
        ta = th(ra)
        edges.append(bd.Edge.make_three_point_arc(P(ra, ph - ta), P(ra, ph), P(ra, ph + ta)))
        edges.append(bd.Edge.make_spline([P(r, ph + th(r)) for r in radii[::-1]]))
        if radial:
            a_root_r = ph + tb + 0.02
            edges.append(bd.Edge.make_line(P(rb, ph + tb), P(rf, a_root_r)))
            a_next = nph - tb - 0.02
        else:
            a_root_r = ph + th(rf)
            a_next = nph - th(rf)
        edges.append(bd.Edge.make_three_point_arc(P(rf, a_root_r), P(rf, (a_root_r + a_next) / 2), P(rf, a_next)))
    return bd.Face(bd.Wire(edges))


class GearPts(list):
    """polyline points (for anything that still wants them) tagged with the exact gear parameters"""
    gear = None


def tagged_involute_profile(orig):
    def f(z=14, m=1.5, pa=20.0, n_inv=10, n_arc=4):
        g = GearPts(orig(z=z, m=m, pa=pa, n_inv=n_inv, n_arc=n_arc))
        g.gear = dict(z=z, m=m, pa=pa)
        return g
    return f
