"""Python copies of the box navigation maths (same numbers as the C++), for log replay."""
import math
import numpy as np

N = 90
ANG = (np.arange(N) * 4 + 2.0)  # deg, centre of each 4 deg point, 0 = front, clockwise


def _fit(x, y):
    mx, my = x.mean(), y.mean()
    dx, dy = x - mx, y - my
    phi = 0.5 * math.atan2(2 * (dx * dy).sum(), (dx * dx).sum() - (dy * dy).sum())
    ux, uy = math.cos(phi), math.sin(phi)
    if ux < 0:
        ux, uy = -ux, -uy
    return mx, my, ux, uy


def measure(scan_cm, side, prior=None):
    """side -1 left, +1 right. Returns dict(ok, d, ang, rms, pts) like WallFollow::measure."""
    bad = dict(ok=False, d=0, ang=0, rms=0, pts=0)
    cm = np.asarray(scan_cm, float)
    center = 270.0 if side < 0 else 90.0
    o = np.abs(((ANG - center + 540) % 360) - 180)
    m = (cm >= 20) & (cm <= 300) & (o <= 60)
    if m.sum() < 5:
        return bad
    r = cm[m] / 100.0
    a = np.radians(ANG[m])
    px = r * np.cos(a)
    py = r * np.sin(a) * (-1 if side < 0 else 1)
    off = o[m]
    if prior is not None and prior["ok"]:
        th = math.radians(prior["ang"])
        sel = np.abs(-math.sin(th) * px + math.cos(th) * py - prior["d"]) < 0.15
        if sel.sum() < 5:
            return bad
    else:
        sel = off <= 20
        if sel.sum() < 4:
            return bad
    mx, my, ux, uy = _fit(px[sel], py[sel])
    for _ in range(2):
        sel = np.abs(-uy * (px - mx) + ux * (py - my)) < 0.10
        if sel.sum() < 5:
            return bad
        mx, my, ux, uy = _fit(px[sel], py[sel])
    qx, qy = px[sel], py[sel]
    along = qx * ux + qy * uy
    if along.max() - along.min() < 0.6 or qx.max() < -0.1:
        return bad
    res = -uy * (qx - mx) + ux * (qy - my)
    return dict(ok=True, d=abs(-uy * mx + ux * my), ang=math.degrees(math.atan2(uy, ux)),
                rms=float(np.sqrt((res ** 2).mean())), pts=int(sel.sum()))


def corridor(scan_cm, phi_deg, half=0.35, look=1.5, minr=0.0):
    cm = np.asarray(scan_cm, float)
    ok = cm > minr * 100
    a = np.radians(ANG[ok] - phi_deg)
    r = cm[ok] / 100.0
    al, lat = r * np.cos(a), r * np.sin(a)
    s = (al > 0) & (al < look) & (np.abs(lat) < half)
    return float(al[s].min()) if s.any() else look
