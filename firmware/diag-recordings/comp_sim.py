"""Hand-held damping gain (the metric the observer failed on hardware), hand ripple, Kd limit."""
import math, loop_sim as L, cand_sim as S

def hand_run(mk, kd=1.6, f=1.0, amp=0.12):
    out = L.run(mk(), kd, t_end=3.0, apply_now=True, hand=(5000, 100, f, amp), noise_counts=0.1, log=True,
                **S.PLANTS["nominal"])
    out = [o for o in out if o[0] > 0.5]
    t = [o[0] for o in out]; th = [o[1] for o in out]; v = [o[3] for o in out]
    vt = [(th[i+10]-th[i-10])/(t[i+10]-t[i-10]) for i in range(10, len(out)-10)]
    vv = v[10:len(out)-10]
    gain = sum(a*b for a, b in zip(vv, vt)) / sum(b*b for b in vt)
    y = [kd*x for x in v]
    r = [y[i] - sum(y[i-5:i+6]) / 11 for i in range(5, len(y) - 5)]
    return gain, math.sqrt(sum(e*e for e in r) / len(r))

if __name__ == "__main__":
    S.KDS = [1.6, 2.4, 3.2, 4.0]
    C = [("biquad70 dz.3 (v10c)", lambda: L.diff_biquad(70, 0.55, dz=0.3)),
         ("observer12 (v11b)", lambda: L.observer(12, 0.1)),
         ("observer20", lambda: L.observer(20, 0.1)),
         ("comp obs20 split6 bq", lambda: L.complementary(20, 6)),
         ("comp obs20 split10 bq", lambda: L.complementary(20, 10)),
         ("comp obs12 split6 bq", lambda: L.complementary(12, 6)),
         ("comp obs20 split4 bq", lambda: L.complementary(20, 4)),
         ("comp obs20 split6 1st", lambda: L.complementary(20, 6, order=1))]
    for n, mk in C:
        g1, hf1 = hand_run(mk)
        g2, _ = hand_run(mk, f=2.5, amp=0.06)
        print("%-22s hand gain 1Hz %.2f 2.5Hz %.2f  hand ripple %.3f V  Kd max %.1f" % (n, g1, g2, hf1, S.kd_max(mk, True)), flush=True)
