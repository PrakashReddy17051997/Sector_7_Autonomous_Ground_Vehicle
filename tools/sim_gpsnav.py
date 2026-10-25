"""Closed-loop simulation of GpsNav (same logic as GpsNav.cpp) on the log_009 route.
Rover model from log_003: 0.55 m/s at command 0.5 -> 1.1 m/s per unit; turn on the spot
31 deg/s at w = 0.5 -> 62 deg/s per unit. GPS: 1 Hz, slow-wandering error (~1 m) + 0.3 m jitter.
IMU yaw: true heading with an unknown offset (compass error 57 deg) and 0.5 deg/min drift."""
import math, numpy as np, sys
rng = np.random.default_rng(int(sys.argv[1]) if len(sys.argv) > 1 else 1)
WP = [(0.0,0.0),(2.8,-3.0),(6.1,-5.2),(9.8,-7.7),(13.2,-9.9),(16.8,-12.0),(20.3,-14.4),(23.2,-17.5),(26.2,-20.6),(28.8,-23.7),(30.6,-27.3),(34.6,-28.0),(32.7,-31.8),(31.2,-35.5),(28.5,-38.6),(25.4,-41.2),(22.4,-44.0),(19.5,-46.8),(16.5,-49.4),(13.2,-51.9),(9.9,-54.2),(6.9,-55.5)]
wrap = lambda a: (a + 180) % 360 - 180
arrive, slow, kT, wfrac, spd = 3.0, 6.0, 0.006, 1.0, 0.25
x, y, hd = 0.0, 0.0, 125.0                    # true pose; start pointing roughly towards WP1
yaw_off = 57.0                                 # compass/yaw error the rover does not know
gx = gy = 0.0; bias = np.array([0.0, 0.0])
state, idx, off, off_ok = "align", 1, 0.0, False
a_start = None; hist = []; dt = 0.02; t = 0.0; track = []; L = R = 0.0
yaw_iso = lambda: -(hd + yaw_off + 0.5 / 60 * t)
while t < 900 and state != "done":
    if abs(t - round(t)) < dt / 2:                       # 1 Hz GPS fix
        bias = 0.97 * bias + rng.normal(0, 0.2, 2)
        gx, gy = x + bias[0] + rng.normal(0, 0.3), y + bias[1] + rng.normal(0, 0.3)
        hist = (hist + [(gx, gy, yaw_iso())])[-6:]
        if off_ok and len(hist) >= 6:
            (ax, ay, aw), (bx, by, bw) = hist[0], hist[-1]
            if abs(wrap(bw - aw)) <= 6 and math.hypot(bx - ax, by - ay) >= 1.2:
                meas = wrap(math.degrees(math.atan2(bx - ax, by - ay)) + aw + 0.5 * wrap(bw - aw))
                off = wrap(off + 0.2 * wrap(meas - off))
    if state == "align":
        if a_start is None and hist: a_start = hist[-1]
        L = R = spd
        if a_start and hist:
            (ax, ay, aw), (bx, by, bw) = a_start, hist[-1]
            if math.hypot(bx - ax, by - ay) >= 2.0:
                off = wrap(math.degrees(math.atan2(bx - ax, by - ay)) + aw + 0.5 * wrap(bw - aw)); off_ok = True; state = "follow"
    else:
        h = wrap(-yaw_iso() + off) % 360
        ex, ny = WP[idx][0] - gx, WP[idx][1] - gy; dist = math.hypot(ex, ny)
        if dist < arrive:
            idx += 1
            if idx >= len(WP): state = "done"; L = R = 0; break
            continue
        err = wrap(math.degrees(math.atan2(ex, ny)) - h)
        v = spd * max(0.5, math.cos(math.radians(err))) * max(0.4, min(1, dist / slow))
        wl = min(1.0 * spd, v); w = max(-wl, min(wl, kT * err))
        L, R = v + w, v - w
    v_t, w_t = 1.1 * (L + R) / 2, 62.0 * (L - R) / 2          # m/s, deg/s (L>R turns right)
    hd = (hd + w_t * dt) % 360
    x += v_t * dt * math.sin(math.radians(hd)); y += v_t * dt * math.cos(math.radians(hd))
    t += dt; track.append((t, x, y))
tr = np.array(track)
# cross-track error to the polyline
def seg_dist(p, a, b):
    a, b, p = map(np.array, (a, b, p)); ab = b - a; u = np.clip(np.dot(p - a, ab) / np.dot(ab, ab), 0, 1); return np.linalg.norm(p - a - u * ab)
cte = [min(seg_dist((px, py), WP[i], WP[i + 1]) for i in range(len(WP) - 1)) for _, px, py in tr[::25]]
print(f"state={state} wp={idx}/{len(WP)} time={t:.0f}s learned offset={off:.1f} (true {-yaw_off:.1f})  "
      f"cross-track median {np.median(cte):.2f} m, 95% {np.percentile(cte,95):.2f} m, max {max(cte):.2f} m")
if len(sys.argv) > 2:
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(7, 9)); w = np.array(WP)
    ax.plot(w[:, 0], w[:, 1], "o--", color="0.5", label="waypoints (log_009)")
    for (wx, wy) in WP: ax.add_patch(plt.Circle((wx, wy), arrive, color="g", alpha=.08))
    ax.plot(tr[:, 1], tr[:, 2], color="tab:blue", label="simulated rover")
    ax.set_aspect("equal"); ax.grid(alpha=.3); ax.legend(); ax.set_xlabel("east (m)"); ax.set_ylabel("north (m)")
    ax.set_title(f"GpsNav simulation: {state}, cross-track median {np.median(cte):.1f} m, max {max(cte):.1f} m")
    fig.tight_layout(); fig.savefig(sys.argv[2], dpi=110)
