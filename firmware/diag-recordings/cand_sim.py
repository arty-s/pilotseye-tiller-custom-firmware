"""Rank damper-velocity candidates by the closed-loop simulator (loop_sim.py).

For each candidate: the largest Kd (0.4 step) that stays quiet in every scenario on both
plants (nominal and the worst-case one that reproduces the v9 center oscillation), plus
rest buzz (u rms with sensor dither) and the lag at a hand-speed 2 Hz motion.
"""
import math, loop_sim as L

PLANTS = {"nominal": dict(g=180, fric=42, sens_tau=0.0012),
          "worst": dict(g=230, fric=15, sens_tau=0.0012)}
SCEN = {"release": dict(x0=-1.5), "small": dict(x0=0.03), "kick": dict(kick=(0.2, 0.35, 0.05))}
KDS = [0.4, 0.8, 1.2, 1.6, 2.0, 2.4, 2.8, 3.2]


def quiet(mk, kd, apply_now):
    for p in PLANTS.values():
        for s in SCEN.values():
            out = L.run(mk(), kd, t_end=1.4, apply_now=apply_now, **p, **s)
            rms, _ = L.osc_metrics(out, 1.0)
            if rms > 0.3:
                return False
    return True


def kd_max(mk, apply_now):
    best = 0.0
    for kd in KDS:
        if not quiet(mk, kd, apply_now):
            break
        best = kd
    return best


def buzz(mk, apply_now, kd=1.6):
    out = L.run(mk(), kd, t_end=1.0, apply_now=apply_now, noise_counts=0.35, x0=0.0005, **PLANTS["nominal"])
    rms, _ = L.osc_metrics(out, 0.3)
    return rms


def hand_lag(mk):
    """Phase lag (ms) of the estimate for a 2 Hz, 0.3 rad hand motion (open loop)."""
    est = mk(); f, amp = 2.0, 0.3; num = den = 0.0; best = None
    samples = []
    for i in range(1500):
        t = i * 1000
        a = amp * math.sin(2 * math.pi * f * t * 1e-6)
        samples.append((t, est(round(a / L.COUNT) * L.COUNT, t)))
    ref = lambda t: amp * 2 * math.pi * f * math.cos(2 * math.pi * f * t * 1e-6)
    errs = {}
    for lag in range(0, 40):
        errs[lag] = sum((v - ref(t - lag * 1000)) ** 2 for t, v in samples[500:])
    return min(errs, key=errs.get)


CANDS = [
    ("v9 now (adaptive+LPF2)", lambda: L.adaptive(tf=0.002)),
    ("diff+biquad 40Hz Q.55", lambda: L.diff_biquad(40, 0.55)),
    ("diff+biquad 70Hz Q.55", lambda: L.diff_biquad(70, 0.55)),
    ("diff+biquad 100Hz Q.55", lambda: L.diff_biquad(100, 0.55)),
    ("diff+biquad 70Hz Q.707", lambda: L.diff_biquad(70, 0.707)),
    ("diff+biquad 70 dz0.15", lambda: L.diff_biquad(70, 0.55, dz=0.15)),
    ("diff+biquad 100 dz0.15", lambda: L.diff_biquad(100, 0.55, dz=0.15)),
]

if __name__ == "__main__":
    print("%-26s %-6s %7s %9s %8s" % ("candidate", "apply", "Kd max", "buzz V", "lag ms"))
    for name, mk in CANDS:
        lag = hand_lag(mk)
        for now in (False, True):
            print("%-26s %-6s %7.1f %9.3f %8d" % (name, "now" if now else "next", kd_max(mk, now), buzz(mk, now), lag),
                  flush=True)
