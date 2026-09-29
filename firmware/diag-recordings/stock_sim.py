"""Stock firmware law (RE 29.09) vs ours: hand release from 0.5 rad, hand wiggle ripple, stand release."""
import math, loop_sim as L, comp_sim as CS, release_sim as RS

P = dict(g=180, fric=42, sens_tau=0.0012)

def rel(mk, law=None, kd=1.6, vlim=6.0):
    out = L.run(mk(), kd, t_end=1.2, apply_now=True, hand=(5000, 100, 0.0, 0.5, 0.5), log=True, x0=0.5, law=law, vlim=vlim, **P)
    post = [o for o in out if o[0] >= 0.5]; t0 = post[0][0]
    vt = [(post[i+1][1]-post[i-1][1])/(post[i+1][0]-post[i-1][0]) for i in range(1, len(post)-1)]
    t_in = next((o[0]-t0 for o in post if abs(o[1]) < 0.05), float('nan'))
    over = max(0.0, -min(o[1] for o in post)) * 57.3
    tail = [o[4] for o in out if o[0] > 1.0]; m = sum(tail)/len(tail)
    osc = math.sqrt(sum((x-m)**2 for x in tail)/len(tail))
    return max(abs(x) for x in vt), t_in*1e3, over, osc

def wig(mk, law=None, kd=1.6, vlim=6.0):
    out = L.run(mk(), kd, t_end=3.0, apply_now=False, hand=(5000, 100, 1.0, 0.12), noise_counts=0.1, log=True, law=law, vlim=vlim, **P)
    y = [o[4] for o in out if o[0] > 0.5]
    r = [y[i] - sum(y[i-5:i+6]) / 11 for i in range(5, len(y) - 5)]
    return math.sqrt(sum(e*e for e in r) / len(r))

print("%-28s %8s %9s %9s %8s %9s" % ("", "peak", "to 3deg", "overshoot", "settled", "wiggle"))
print("%-28s %8s %9s %9s %8s %9s" % ("", "rad/s", "ms", "deg", "osc V", "ripple V"))
for name, mk, law, vl in [
        ("ours v11d, damp max", lambda: L.complementary(12, 20), None, 6.0),
        ("ours v11d, damp 0", lambda: L.complementary(12, 20), "kd0", 6.0),
        ("stock blue max, red max", L.stock_velocity, L.stock_law(1.5, 10), 12.0),
        ("stock blue max, red min", L.stock_velocity, L.stock_law(1.5, 2), 12.0),
        ("stock blue min, red max", L.stock_velocity, L.stock_law(0.3, 10), 12.0),
        ("stock blue min, red min", L.stock_velocity, L.stock_law(0.3, 2), 12.0)]:
    kd = 0.0 if law == "kd0" else 1.6
    lw = None if law in (None, "kd0") else law
    r = rel(mk, lw, kd, vl)
    lw2 = None if law in (None, "kd0") else L.stock_law(*{id(law): None}.get(0, ()) ) if False else lw
    print("%-28s %8.2f %9.0f %9.1f %8.3f %9.3f" % (name, *r, wig(mk, lw, kd, vl)), flush=True)
