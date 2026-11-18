"""Automatic part-ID engraving for printed parts that have no hand-placed mark (CLAUDE.md "Engraving").

    auto_engrave(solid, full, short) -> (engraved solid, info) or (solid, None) when nothing fits

Searches the part's largest flat faces, several points on each and both reading directions, for the biggest text
(5 .. 2 mm) of `full` (ID-BATCH), then `short` (ID only), such that:
  * a box of the text + 1 mm margin, ENGRAVE_DEPTH + 0.05 deep, lies entirely inside the part (the cut cannot break out)
  * nothing of the part lies within 2 mm above that box (the face is open, not covered by another feature of the part)
Bold text, cut ENGRAVE_DEPTH (0.6 mm = 3 layers at 0.2 mm).
"""
import build123d as bd

ENGRAVE_DEPTH = 0.6
SIZES = (5.0, 4.0, 3.0, 2.5, 2.0)
MARGIN = 1.0


def _vol(shape):
    if shape is None:
        return 0.0
    return sum(s.volume for s in shape.solids()) if hasattr(shape, "solids") else getattr(shape, "volume", 0.0)


def _axes(face):
    """in-plane unit axes (u along the face's longest straight edge, v = n x u) and the outward normal"""
    n = face.normal_at()
    best, blen = None, 0.0
    for e in face.edges():
        if e.geom_type == bd.GeomType.LINE and e.length > blen:
            best, blen = e, e.length
    if best is not None:
        u = (best.end_point() - best.start_point()).normalized()
    else:
        tmp = bd.Vector(1, 0, 0) if abs(n.X) < 0.9 else bd.Vector(0, 1, 0)
        u = n.cross(tmp).normalized()
    v = n.cross(u).normalized()
    return u, v, n


def _candidates(face, u, v):
    c = face.center()
    bb = face.bounding_box()
    span = max(bb.size.X, bb.size.Y, bb.size.Z)
    pts = [c]
    for fu, fv in ((0.25, 0), (-0.25, 0), (0, 0.25), (0, -0.25), (0.4, 0), (-0.4, 0), (0, 0.4), (0, -0.4),
                   (0.3, 0.3), (-0.3, 0.3), (0.3, -0.3), (-0.3, -0.3)):
        pts.append(c + u * (fu * span) + v * (fv * span))
    return pts


def _try(solid, text, size, origin, xdir, n):
    o = origin + n * 0.05
    with bd.BuildSketch(bd.Plane(origin=o, x_dir=xdir, z_dir=n)) as sk:
        bd.Text(text, font_size=size, font_style=bd.FontStyle.BOLD, align=(bd.Align.CENTER, bd.Align.CENTER))
    txt = sk.sketch
    tb = txt.bounding_box()
    # text extent in the plane: project the 3D bbox corners onto xdir / ydir
    ydir = n.cross(xdir)
    pts = [bd.Vector(x, y, z) for x in (tb.min.X, tb.max.X) for y in (tb.min.Y, tb.max.Y) for z in (tb.min.Z, tb.max.Z)]
    us = [(p - o).dot(xdir) for p in pts]
    vs = [(p - o).dot(ydir) for p in pts]
    w, h = max(us) - min(us) + 2 * MARGIN, max(vs) - min(vs) + 2 * MARGIN
    cu, cv = (max(us) + min(us)) / 2, (max(vs) + min(vs)) / 2
    pl = bd.Plane(origin=origin + xdir * cu + ydir * cv, x_dir=xdir, z_dir=n)      # margin box ON the surface
    with bd.BuildSketch(pl) as rs:
        bd.Rectangle(w, h)
    rect = rs.sketch
    below = bd.extrude(rect, amount=-(ENGRAVE_DEPTH + 0.05))
    if _vol(solid & below) < 0.995 * below.volume:
        return None
    above = bd.extrude(rect.moved(bd.Location(n * 0.1)), amount=2.0)
    if _vol(solid & above) > 1e-3:
        return None
    return solid - bd.extrude(txt, amount=-(ENGRAVE_DEPTH + 0.05))


def auto_engrave(solid, full, short, max_faces=8):
    faces = [f for f in solid.faces() if f.geom_type == bd.GeomType.PLANE]
    faces.sort(key=lambda f: f.area, reverse=True)
    faces = faces[:max_faces]
    prepared = []
    for f in faces:
        try:
            u, v, n = _axes(f)
        except Exception:
            continue
        prepared.append((f, u, v, n, _candidates(f, u, v)))
    for text in (full, short):
        for size in SIZES:
            for f, u, v, n, pts in prepared:
                for p in pts:
                    for xdir in (u, v):
                        try:
                            out = _try(solid, text, size, p, xdir, n)
                        except Exception:
                            out = None
                        if out is not None and len(out.solids()) == len(solid.solids()):
                            return out, {"text": text, "size": size, "at": (round(p.X, 1), round(p.Y, 1), round(p.Z, 1)),
                                         "normal": (round(n.X, 2), round(n.Y, 2), round(n.Z, 2))}
    return solid, None
