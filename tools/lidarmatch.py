"""LiDAR scan matching for the route map - Python twin of RouteLocalizer.cpp (same numbers).

Scans: 90 points of 4 deg, index 0 = front, clockwise, cm, 0 = no return (Sensors::lidarCm).
Map keyframes are stored as uint8 in 4 cm steps (0 = no return, max 10.2 m) to fit the ESP32 flash.
"""
import numpy as np

N = 90
MAXR = 600.0      # cm: farther returns are not compared (outdoors they come and go)
TRUNC = 80.0      # cm: error cap per point (people, bushes, driving 0.5 m beside the taught line)
NEAR = 300.0      # cm: structure seen in only one of the two scans counts as TRUNC if closer than this
MIN_BINS = 12     # fewer comparable points -> no answer
ROT = 6           # rotation search +- 6 points = +- 24 deg
ANG = np.arange(N) * 4 + 2.0


def quantise(cm):
    return np.clip(np.round(np.asarray(cm, float) / 4.0), 0, 255).astype(np.uint8)


def dequantise(q):
    return np.asarray(q, float) * 4.0


def keyframe(S, i, half=3, min_share=0.4):
    """median scan of records i-half..i+half; a point counts only if it returned in >= 40 % of them"""
    w = S[max(0, i - half):i + half + 1].astype(float)
    v = w > 0
    w[~v] = np.nan
    with np.errstate(all="ignore"):
        med = np.nanmedian(w, axis=0)
    med = np.nan_to_num(med, nan=0.0)
    med[v.mean(axis=0) < min_share] = 0.0
    return med


def scan_cost(c, k, shift):
    """mean per-point range error (cm) between current scan c and keyframe k rotated by `shift` points.
    Each current point is compared with the best of 3 neighbouring keyframe points."""
    k0 = np.roll(k, -shift)
    km, kp = np.roll(k0, 1), np.roll(k0, -1)
    vc = (c > 20) & (c < MAXR)
    vk = (k0 > 20) & (k0 < MAXR)
    e = np.full(N, TRUNC)
    for kk in (k0, km, kp):
        ok = vc & (kk > 20) & (kk < MAXR)
        e = np.where(ok, np.minimum(e, np.abs(c - kk)), e)
    both = vc & vk
    one = (vc & ~vk & (c < NEAR)) | (~vc & vk & (k0 < NEAR))
    n = int(both.sum() + one.sum())
    if n < MIN_BINS:
        return None
    return float((np.minimum(e[both], TRUNC).sum() + TRUNC * one.sum()) / n)


def end_cost(c, k, center=0):
    """best scan_cost over rotations center +- ROT"""
    best = None
    for sh in range(center - ROT, center + ROT + 1):
        v = scan_cost(c, k, sh)
        if v is not None and (best is None or v < best):
            best = v
    return best


class EndDetector:
    """Dip detector for the end of a leg (same logic as RouteLocalizer::_endStep).
    feed() 2x per second after 30 % of the taught effort; dip() = clear minimum below the leg's
    typical level, passed (3 samples higher again)."""
    DROP = 12.0

    def __init__(self):
        self.hist, self.best, self.besti, self.rising, self.now = [], 9e9, None, 0, None

    def base(self):
        h = self.hist[:max(4, len(self.hist) // 2)]
        return float(np.median(h)) if h else None

    def feed(self, c, gate, i):
        if c is None:
            return
        self.now = c
        if len(self.hist) < 64:
            self.hist.append(c)
        if gate:
            if c < self.best:
                self.best, self.besti, self.rising = c, i, 0
            elif self.besti is not None and c > self.best + self.DROP / 2:
                self.rising += 1

    def dip(self):
        b = self.base()
        return self.besti is not None and b is not None and self.best < b - self.DROP and self.rising >= 3

    def looks_like_end(self):
        b = self.base()
        return self.now is None or b is None or self.now <= b - self.DROP / 3


def ahead_bins(c, half=0.35, look=0.7, minr=25):
    """LiDAR points inside the corridor ahead (the area RouteFollow::_front checks)"""
    a = np.radians(ANG)
    r = c / 100.0
    al, lat = r * np.cos(a), r * np.sin(a)
    return np.where((c > minr) & (al > 0.05) & (al < look) & (np.abs(lat) < half))[0]   # > 5 cm ahead: not beside the rover


def new_points(c, bins, keys, tol=30.0, rot=3):
    """points in `bins` that no keyframe in `keys` explains (no taught return within tol cm, +-rot points)"""
    out = []
    for b in bins:
        static = False
        for k in keys:
            for d in range(-rot, rot + 1):
                kb = k[(b + d) % N]
                if kb > 0 and abs(kb - c[b]) < tol:
                    static = True
                    break
            if static:
                break
        if not static:
            out.append(b)
    return out
