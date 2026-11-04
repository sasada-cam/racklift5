"""Tiny orthographic z-buffer renderer with flat shading + feature outlines."""
import numpy as np, math
from PIL import Image, ImageDraw, ImageFont


def view_matrix(az, el):
    a, e = math.radians(az), math.radians(el)
    # camera looks from direction d toward origin
    d = np.array([math.cos(e) * math.sin(a), -math.cos(e) * math.cos(a), math.sin(e)])
    f = -d
    up = np.array([0, 0, 1.0])
    r = np.cross(f, up); r /= np.linalg.norm(r)
    u = np.cross(r, f)
    return np.stack([r, u, -f])  # rows: screen x, screen y, depth(toward viewer)


def render(objs, path, az=-35, el=25, W=1400, H=1000, margin=40, bg=(250, 250, 252),
           title=None, labels=None, fit=None):
    R = view_matrix(az, el)
    allv = np.vstack([t.reshape(-1, 3) for t, c in objs if len(t)])
    if fit is not None:
        allv = np.asarray(fit)
    P = allv @ R.T
    mn, mx = P.min(0), P.max(0)
    sc = min((W - 2 * margin) / max(mx[0] - mn[0], 1e-9), (H - 2 * margin - (40 if title else 0)) / max(mx[1] - mn[1], 1e-9))
    cx, cy = (mn[0] + mx[0]) / 2, (mn[1] + mx[1]) / 2
    oy = 20 if title else 0
    zbuf = np.full((H, W), -np.inf)
    img = np.zeros((H, W, 3)); img[:] = np.array(bg) / 255
    nbuf = np.zeros((H, W, 3))
    idb = np.full((H, W), -1, np.int32)
    L1 = np.array([0.4, 0.5, 0.75]); L1 /= np.linalg.norm(L1)
    L2 = np.array([-0.6, 0.2, 0.3]); L2 /= np.linalg.norm(L2)
    oid = 0
    for tris, col in objs:
        oid += 1
        if len(tris) == 0: continue
        T = tris @ R.T
        X = (T[..., 0] - cx) * sc + W / 2
        Y = H / 2 + oy - (T[..., 1] - cy) * sc
        Z = T[..., 2]
        n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
        Ln = np.linalg.norm(n, axis=1); ok = Ln > 1e-12
        n[ok] /= Ln[ok, None]
        n[n[:, 2] < 0] *= -1  # two-sided
        shade = 0.35 + 0.5 * np.clip(n @ L1, 0, 1) + 0.2 * np.clip(n @ L2, 0, 1)
        base = np.array(col[:3], float) / 255
        x0 = np.clip(np.floor(X.min(1)).astype(int), 0, W - 1); x1 = np.clip(np.ceil(X.max(1)).astype(int), 0, W - 1)
        y0 = np.clip(np.floor(Y.min(1)).astype(int), 0, H - 1); y1 = np.clip(np.ceil(Y.max(1)).astype(int), 0, H - 1)
        for i in np.nonzero(ok)[0]:
            if x1[i] < x0[i] or y1[i] < y0[i]: continue
            xs = np.arange(x0[i], x1[i] + 1) + 0.5; ys = np.arange(y0[i], y1[i] + 1) + 0.5
            gx, gy = np.meshgrid(xs, ys)
            ax, ay, bx, by, qx, qy = X[i, 0], Y[i, 0], X[i, 1], Y[i, 1], X[i, 2], Y[i, 2]
            den = (by - qy) * (ax - qx) + (qx - bx) * (ay - qy)
            if abs(den) < 1e-12: continue
            w0 = ((by - qy) * (gx - qx) + (qx - bx) * (gy - qy)) / den
            w1 = ((qy - ay) * (gx - qx) + (ax - qx) * (gy - qy)) / den
            w2 = 1 - w0 - w1
            m = (w0 >= -1e-6) & (w1 >= -1e-6) & (w2 >= -1e-6)
            if not m.any(): continue
            z = w0 * Z[i, 0] + w1 * Z[i, 1] + w2 * Z[i, 2]
            sub = zbuf[y0[i]:y1[i] + 1, x0[i]:x1[i] + 1]
            upd = m & (z > sub)
            if not upd.any(): continue
            sub[upd] = z[upd]
            img[y0[i]:y1[i] + 1, x0[i]:x1[i] + 1][upd] = np.clip(base * shade[i], 0, 1)
            nbuf[y0[i]:y1[i] + 1, x0[i]:x1[i] + 1][upd] = n[i]
            idb[y0[i]:y1[i] + 1, x0[i]:x1[i] + 1][upd] = oid
    # outlines: object id change, normal crease, depth jump
    edge = np.zeros((H, W), bool)
    for dy, dx in ((0, 1), (1, 0)):
        a = idb[:H - dy, :W - dx]; b = idb[dy:, dx:]
        e = a != b
        na = nbuf[:H - dy, :W - dx]; nb = nbuf[dy:, dx:]
        e |= (np.sum(na * nb, -1) < 0.85) & (a >= 0) & (b >= 0)
        za = zbuf[:H - dy, :W - dx]; zb = zbuf[dy:, dx:]
        with np.errstate(invalid='ignore'):
            e |= (np.abs(za - zb) * sc > 6) & np.isfinite(za) & np.isfinite(zb)
        edge[:H - dy, :W - dx] |= e
    img[edge] = img[edge] * 0.35
    im = Image.fromarray((img * 255).astype(np.uint8))
    d = ImageDraw.Draw(im)
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 26)
        fs = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 18)
    except Exception:
        font = fs = ImageFont.load_default()
    if title:
        d.text((margin, 14), title, fill=(30, 30, 40), font=font)
    if labels:
        for (p3, text) in labels:
            p = np.asarray(p3) @ R.T
            px = (p[0] - cx) * sc + W / 2; py = H / 2 + oy - (p[1] - cy) * sc
            d.text((px, py), text, fill=(180, 30, 30), font=fs)
    im.save(path)
    return im
