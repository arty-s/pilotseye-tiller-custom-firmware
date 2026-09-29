"""Hand holds the handle 0.5 rad off center, then lets go: is the damper there at once?"""
import math, loop_sim as L

def release(mk, kd=1.6, hold=0.5):
    out = L.run(mk(), kd, t_end=1.2, apply_now=True, hand=(5000, 100, 0.0, hold, 0.5), log=True, x0=hold,
                g=180, fric=42, sens_tau=0.0012)
    post = [o for o in out if o[0] >= 0.5]
    t0 = post[0][0]
    vt = [(post[i+1][1]-post[i-1][1])/(post[i+1][0]-post[i-1][0]) for i in range(1, len(post)-1)]
    peak = max(abs(x) for x in vt)
    t_half = next((o[0]-t0 for o in post if abs(o[1]) < hold/2), float('nan'))
    t_in = next((o[0]-t0 for o in post if abs(o[1]) < 0.05), float('nan'))
    lat = next((o[0]-t0 for o in post if abs(kd*o[3]) > 0.5), float('nan'))
    return peak, t_half*1e3, t_in*1e3, lat*1e3

C = [("no damper (Kd 0)", lambda: L.diff_biquad(70, 0.55, dz=0.3), 0.0),
     ("v10c biquad70", lambda: L.diff_biquad(70, 0.55, dz=0.3), 1.6),
     ("v11b observer12", lambda: L.observer(12, 0.1), 1.6),
     ("v11d comp 12/20", lambda: L.complementary(12, 20), 1.6),
     ("comp 12/40", lambda: L.complementary(12, 40), 1.6),
     ("comp 16/30", lambda: L.complementary(16, 30), 1.6)]
for n, mk, kd in C:
    print("%-18s peak speed %5.2f rad/s  half-way %4.0f ms  in 3deg %4.0f ms  damper>0.5V after %4.0f ms" % ((n,) + release(mk, kd)), flush=True)
