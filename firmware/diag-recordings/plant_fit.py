"""Fit the tiller plant  a'' = g*U - c*w - f*sign(w) + b  from the v7 release recording.

U is the voltage actually on the motor: the u computed at pass k is applied by the
NEXT loopFOC() (SimpleFOC applies current_sp after its sensor read), clamped to +-VLIM.
"""
import json, math, pathlib

HERE = pathlib.Path(__file__).parent
VLIM = 6.0
CH = json.load(open(HERE / "2026-09-29_v7_release_limit-cycle-25Hz.json"))


def solve(A, y):
    n = len(A[0])
    M = [[sum(r[i] * r[j] for r in A) for j in range(n)] for i in range(n)]
    v = [sum(r[i] * yy for r, yy in zip(A, y)) for i in range(n)]
    for i in range(n):                       # Gauss-Jordan
        p = max(range(i, n), key=lambda r: abs(M[r][i]))
        M[i], M[p], v[i], v[p] = M[p], M[i], v[p], v[i]
        for r in range(n):
            if r != i:
                k = M[r][i] / M[i][i]
                M[r] = [a - k * b for a, b in zip(M[r], M[i])]
                v[r] -= k * v[i]
    return [v[i] / M[i][i] for i in range(n)]


def grid(rows, dt=0.0005):
    """Resample onto a uniform grid: angle linear-interp, applied voltage zero-order hold."""
    t = [r[0] * 1e-6 for r in rows]; a = [r[1] for r in rows]; u = [r[3] for r in rows]
    T, A, U = [], [], []
    k = 1
    tt = t[1]
    while tt < t[-1]:
        while t[k + 1] <= tt:
            k += 1
        A.append(a[k] + (a[k + 1] - a[k]) * (tt - t[k]) / (t[k + 1] - t[k]))
        # at time tt the motor carries u from the pass before the latest sensor read
        U.append(max(-VLIM, min(VLIM, u[k - 1])))
        T.append(tt); tt += dt
    return A, U, dt


def fit(h=10, chunks=(1, 2, 3, 4)):
    Arows, Y = [], []
    for ci in chunks:
        A, U, dt = grid(CH[ci])
        H = h * dt
        for i in range(2 * h, len(A) - 2 * h):
            y = (A[i + h] - 2 * A[i] + A[i - h]) / (H * H)
            w = [(h - abs(j)) / (h * h) for j in range(-h + 1, h)]   # triangular kernel of the 2nd difference
            idx = range(i - h + 1, i + h)
            Ue = sum(wj * U[k] for wj, k in zip(w, idx))
            We = sum(wj * (A[k + 1] - A[k - 1]) / (2 * dt) for wj, k in zip(w, idx))
            Se = sum(wj * (1 if (A[k + h // 2] - A[k - h // 2]) > 0 else -1 if (A[k + h // 2] - A[k - h // 2]) < 0 else 0)
                     for wj, k in zip(w, idx))
            Arows.append([Ue, -We, -Se, 1.0]); Y.append(y)
    p = solve(Arows, Y)
    pred = [sum(a * b for a, b in zip(r, p)) for r in Arows]
    ss = sum((y - q) ** 2 for y, q in zip(Y, pred)); st = sum((y - sum(Y) / len(Y)) ** 2 for y in Y)
    return p, 1 - ss / st, len(Y)


if __name__ == "__main__":
    for h in (6, 10, 16):
        p, r2, n = fit(h)
        print("h=%2d  g=%.1f rad/s2/V  c=%.2f 1/s  f=%.1f rad/s2  b=%.1f   R2=%.3f n=%d" % (h, *p, r2, n))
    for ci in (1, 2, 3, 4):
        p, r2, n = fit(10, (ci,))
        print("chunk %d  g=%.1f c=%.2f f=%.1f b=%.1f R2=%.3f" % (ci, *p, r2))
