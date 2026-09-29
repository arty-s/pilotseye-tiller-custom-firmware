"""Closed-loop simulator of the tiller spring/damper: plant fitted from the v7 recording
(plant_fit.py, chunk 4: g=180 rad/s^2/V, c=70 1/s, Coulomb 42 rad/s^2), AS5600 12-bit
quantization + internal filter lag, ~1 kHz loop with the recorded jitter, and SimpleFOC's
one-pass voltage application delay (optionally removed).

Validation targets (hardware, 29.09): v7 fixed 20 ms window @ Kd 1.6 -> 25 Hz limit cycle
after release; v9 adaptive window @ Kd 1.6 -> ~33 Hz, +-8 counts at the center.
"""
import math, random

COUNT = 2 * math.pi / 4096
G, C, FRIC = 180.0, 70.0, 42.0


# ---------------------------------------------------------------- estimators
def window_v7():
    return adaptive(fast=1e9)


def adaptive(win=20000, vmin=2000, fast=8.0, tf=0.0, n=64):
    A = [0.0] * n; T = [0] * n; st = {"c": 0, "h": 0, "y": 0.0, "tp": None}

    def upd(angle, now):
        A[st["h"]] = angle; T[st["h"]] = now; newest = st["h"]
        st["h"] = (st["h"] + 1) % n; st["c"] = min(st["c"] + 1, n)
        v = 0.0
        for k in range(1, st["c"]):
            i = (newest - k + n) % n; age = now - T[i]; d = angle - A[i]
            if (age >= vmin and abs(d) >= fast * COUNT) or age >= win or k == st["c"] - 1:
                if age >= vmin:
                    d = math.copysign(max(abs(d) - 1.05 * COUNT, 0.0), d); v = d / (age * 1e-6)
                break
        if tf > 0:
            if st["tp"] is not None:
                dt = (now - st["tp"]) * 1e-6; al = tf / (tf + dt); st["y"] = al * st["y"] + (1 - al) * v
            else:
                st["y"] = v
            st["tp"] = now; v = st["y"]
        return v
    return upd


class Biquad:
    """RBJ low-pass, fixed fs (same code as the firmware)."""
    def __init__(self, fc, q, fs=1000.0):
        w = 2 * math.pi * fc / fs; al = math.sin(w) / (2 * q); cw = math.cos(w); a0 = 1 + al
        self.b0 = (1 - cw) / 2 / a0; self.b1 = (1 - cw) / a0; self.b2 = self.b0
        self.a1 = -2 * cw / a0; self.a2 = (1 - al) / a0
        self.z1 = self.z2 = 0.0

    def __call__(self, x):
        y = self.b0 * x + self.z1
        self.z1 = self.b1 * x - self.a1 * y + self.z2
        self.z2 = self.b2 * x - self.a2 * y
        return y


def diff_biquad(fc=70.0, q=0.55, fs=1000.0, dz=0.0, span=1):
    """Velocity = (a[k]-a[k-span]) / (t[k]-t[k-span]) with real sample timestamps, then biquad.
    dz: soft dead zone (rad/s) applied AFTER the filter (quiet at rest)."""
    bq = Biquad(fc, q, fs); hist = []

    def upd(angle, now):
        hist.append((angle, now))
        if len(hist) > span + 1:
            hist.pop(0)
        if len(hist) <= span:
            return 0.0
        a0, t0 = hist[0]
        dtu = now - t0
        raw = (angle - a0) / (dtu * 1e-6) if dtu > 0 else 0.0
        v = bq(raw)
        if dz > 0:
            v = math.copysign(max(abs(v) - dz, 0.0), v)
        return v
    return upd


def observer(f_obs=30.0, dz=0.1, g=180.0, c=70.0, vlim=6.0):
    """Model-based velocity: theta'' = g*U - c*w + d, triple pole at 2*pi*f_obs.
    The applied voltage comes in through .feed_u() (the U put on the phases this pass)."""
    p = 2 * math.pi * f_obs
    st = {"th": None, "w": 0.0, "d": 0.0, "t": None, "u": 0.0}

    def upd(meas, now):
        if st["th"] is None:
            st["th"], st["t"] = meas, now
            return 0.0
        dt = (now - st["t"]) * 1e-6
        st["t"] = now
        if dt <= 0 or dt > 0.02:
            st.update(th=meas, w=0.0, d=0.0)
            return 0.0
        th = st["th"] + st["w"] * dt
        w = st["w"] + (g * st["u"] - c * st["w"] + st["d"]) * dt
        e = meas - th
        st["th"] = th + 3 * p * dt * e
        st["w"] = w + 3 * p * p * dt * e
        st["d"] = st["d"] + p * p * p * dt * e
        v = st["w"]
        return math.copysign(max(abs(v) - dz, 0.0), v)

    def feed_u(u):
        st["u"] = max(-vlim, min(vlim, u))
    upd.feed_u = feed_u
    return upd


def complementary(f_obs=20.0, f_split=6.0, order=2, dz=0.1, g=180.0, c=70.0, vlim=6.0, cascade=1, post_fc=None):
    """v = v_obs + LPF_split(v_meas - v_obs): low frequencies (hand motion) from the measured
    first difference (unbiased), high frequencies (release / oscillation modes) from the
    model observer (no lag). The observer alone reads a hand-driven motion as 0.2..-0.2 of
    its true speed: the spring force the hand holds looks like an acceleration to the model."""
    obs = observer(f_obs, 0.0, g, c, vlim)
    lp = Biquad(f_split, 0.707) if order == 2 else None
    lp2 = Biquad(f_split, 0.707) if cascade == 2 else None
    post = Biquad(post_fc, 0.707) if post_fc else None
    st = {"a": None, "t": None, "raw": 0.0, "y": 0.0}

    def upd(meas, now):
        vo = obs(meas, now)
        if st["a"] is not None and now > st["t"]:
            st["raw"] = (meas - st["a"]) / ((now - st["t"]) * 1e-6)
        st["a"], st["t"] = meas, now
        e = st["raw"] - vo
        if lp:
            ef = lp(e)
            if lp2:
                ef = lp2(ef)
        else:
            al = 1.0 - math.exp(-2 * math.pi * f_split * 1e-3); st["y"] += al * (e - st["y"]); ef = st["y"]
        v = vo + ef
        if post:
            v = post(v)
        return math.copysign(max(abs(v) - dz, 0.0), v)
    upd.feed_u = obs.feed_u
    return upd


def stock_velocity(tf=0.1):
    """Stock firmware: SimpleFOC velocity (angle difference per pass) through LowPassFilter Tf=0.1 s."""
    st = {"a": None, "t": None, "y": 0.0}
    def upd(meas, now):
        if st["a"] is None:
            st["a"], st["t"] = meas, now
            return 0.0
        dt = (now - st["t"]) * 1e-6
        raw = (meas - st["a"]) / dt if dt > 0 else 0.0
        st["a"], st["t"] = meas, now
        al = tf / (tf + dt); st["y"] = al * st["y"] + (1 - al) * raw
        return st["y"]
    return upd


def stock_law(kv, vlim, p_angle=25.0, vmax=12.0, tf_angle=0.01):
    """Stock angle mode: vel_sp = clamp(P*(center - theta_f), +-vlim); u = Kv*(vel_sp - w_f)."""
    st = {"th": None, "t": None}
    def law(meas, v, t, center):
        if st["th"] is None:
            st["th"], st["t"] = meas, t
        dt = t - st["t"]; st["t"] = t
        al = tf_angle / (tf_angle + dt) if dt > 0 else 1.0
        st["th"] = al * st["th"] + (1 - al) * meas
        vsp = max(-vlim, min(vlim, p_angle * (center - st["th"])))
        return kv * (vsp - v)
    return law


# ---------------------------------------------------------------- simulation
def run(est, kd, kp=20.0, vlim=6.0, x0=0.0, t_end=1.2, apply_now=False, sens_tau=0.0007,
        noise_counts=0.0, seed=1, kick=None, g=G, c=C, fric=FRIC, damp_scale=None, log=False, hand=None, law=None):
    """hand=(kh, bh, f, amp[, t_release]): a hand holding the handle, spring kh / damper bh (rad/s^2 units) to a slow sine target (f=0: fixed at amp), released at t_release."""
    """kick=(t_on, t_off, rad): shift the center by rad during [t_on, t_off) (stand test)."""
    rnd = random.Random(seed)
    h = 50e-6
    th, om, filt = x0, 0.0, x0
    t, tnext = 0.0, 0.0
    u_applied, u_pending = 0.0, 0.0
    out = []
    while t < t_end:
        if t >= tnext:
            # one control pass: I2C read happens ~0.1 ms into the pass
            meas = round((filt + rnd.gauss(0, noise_counts * COUNT)) / COUNT) * COUNT
            now = int(t * 1e6)
            if not apply_now:
                u_applied = u_pending            # loopFOC applies the PREVIOUS move()
            v = est(meas, now)
            center = kick[2] if kick and kick[0] <= t < kick[1] else 0.0
            ds = damp_scale(v) if damp_scale else 1.0
            u = law(meas, v, t, center) if law else kp * (center - meas) - kd * ds * v
            u_pending = max(-vlim, min(vlim, u))
            if apply_now:
                u_applied = u_pending
            if hasattr(est, 'feed_u'):
                est.feed_u(u_applied if apply_now else u_pending)
            if log:
                out.append((t, th, meas, v, u))
            else:
                out.append((t, u))
            r = rnd.random()
            dtl = 0.0009 + 0.0002 * r if r < 0.93 else 0.0005 + 0.0014 * rnd.random()
            tnext = t + dtl
        acc = g * u_applied - c * om
        if hand and len(hand) == 5 and t >= hand[4]:
            pass                                   # hand let go at hand[4] seconds
        elif hand:
            kh, bh, hf_, ha = hand[:4]; w_ = 2 * math.pi * hf_
            if hf_ > 0:
                acc += kh * (ha * math.sin(w_ * t) - th) + bh * (ha * w_ * math.cos(w_ * t) - om)
            else:                                  # hold still at ha
                acc += kh * (ha - th) - bh * om
        if abs(om) < 1e-3 and abs(acc) <= fric:
            om, acc = 0.0, 0.0
        else:
            acc -= fric * (1 if om > 0 else -1)
        om += acc * h
        th += om * h
        filt += (th - filt) * (h / sens_tau if sens_tau > 0 else 1.0)
        t += h
    return out


def osc_metrics(out, t_from):
    """RMS of u and dominant frequency (zero crossings of u - mean) after t_from."""
    us = [(t, u) for t, u, *_ in [(o[0], o[-1]) for o in out] if t >= t_from]
    if len(us) < 10:
        return 0.0, 0.0
    m = sum(u for _, u in us) / len(us)
    rms = math.sqrt(sum((u - m) ** 2 for _, u in us) / len(us))
    zc = [us[i][0] for i in range(1, len(us)) if (us[i][1] - m) * (us[i - 1][1] - m) < 0]
    f = (len(zc) - 1) / 2 / (zc[-1] - zc[0]) if len(zc) > 3 else 0.0
    return rms, f


if __name__ == "__main__":
    print("validation (Kd 1.6, Kp 20, F 6):")
    for name, mk in (("v7 fixed 20ms", window_v7), ("v9 adaptive+LPF2", lambda: adaptive(tf=0.002))):
        for label, x0 in (("release 1.5 rad", -1.5), ("small 0.03 rad", 0.03)):
            out = run(mk(), 1.6, x0=x0, t_end=1.5)
            rms, f = osc_metrics(out, 0.9)
            print("  %-18s %-16s  u_rms=%.2f V  f=%.1f Hz" % (name, label, rms, f))
