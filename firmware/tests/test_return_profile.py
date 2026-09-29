import math
import pathlib
import re
import sys
import unittest

ACTIVE, RETURN, QUIET, FULL, MOTION = 1.00, 1.00, 0.040, 0.080, 0.10
IDLE_MS, RAMP_MS = 700, 1400


class Profile:
    """Executable dynamics model of firmware/return_profile.h."""
    def __init__(self, now=0):
        self.returning = False
        self.last_away = self.return_start = now

    def update(self, now, error, velocity):
        moving_away = abs(velocity) >= MOTION and error * velocity < 0
        if moving_away:
            self.returning = False
            self.last_away = now
        elif not self.returning and now - self.last_away >= IDLE_MS:
            self.returning = True
            self.return_start = now
        t = min(1.0, (now - self.return_start) / RAMP_MS) if self.returning else 0.0
        blend = t * t * (3.0 - 2.0 * t)
        scale = ACTIVE + (RETURN - ACTIVE) * blend
        magnitude = abs(error)
        if magnitude <= QUIET:
            spring_error = 0.0
        elif magnitude < FULL:
            t_center = (magnitude - QUIET) / (FULL - QUIET)
            spring_error = error * t_center * t_center * (3.0 - 2.0 * t_center)
        else:
            spring_error = error
        return scale, spring_error, blend




class ReturnProfileTests(unittest.TestCase):
    def test_uses_only_the_stock_simplefoc_pwm_driver(self):
        # Mixing the legacy and new IDF MCPWM drivers aborts at boot (driver_ng
        # CONFLICT) before USB comes up -> BOOT+RST needed. Keep one driver only.
        root = pathlib.Path(__file__).parents[1] / "piloteye_tiller"
        source = (root / "piloteye_tiller.cpp").read_text()
        self.assertIn("BLDCDriver3PWM  driver", source)
        self.assertNotIn("legacy_mcpwm_driver.h", source)
        self.assertFalse((root / "legacy_mcpwm_driver.h").exists())

    def test_commander_table_fits(self):
        # SimpleFOC Commander::add writes call_list[20] without a bounds check.
        source = (pathlib.Path(__file__).parents[1] / "piloteye_tiller" / "piloteye_tiller.cpp").read_text()
        ids = re.findall(r"command\.add\('(.)'", source)
        self.assertLessEqual(len(ids), 20)
        self.assertEqual(len(ids), len(set(ids)))

    def test_damper_velocity_is_not_simplefoc_held_velocity(self):
        source = (pathlib.Path(__file__).parents[1] / "piloteye_tiller" / "piloteye_tiller.cpp").read_text()
        task = source[source.index("void motorTask"):source.index("// hid_task:")]
        self.assertNotIn("getVelocity", task)
        self.assertIn("dampingVelocity.update(", task)

    def test_motor_init_failures_leave_usb_alive_and_motor_disabled(self):
        source = (pathlib.Path(__file__).parents[1] / "piloteye_tiller" / "piloteye_tiller.cpp").read_text()
        setup = source[source.index("void setup()"):]
        self.assertLess(setup.index("USB.begin();"), setup.index("driver.init()"))
        self.assertIn('holdBootDiagnostic("driver-failed")', setup)
        self.assertIn('holdBootDiagnostic("motor-init-failed")', setup)
        self.assertIn('holdBootDiagnostic("init-foc-failed")', setup)

    def test_motor_loop_uses_single_sensor_update_in_simplefoc_order(self):
        source = (pathlib.Path(__file__).parents[1] / "piloteye_tiller" / "piloteye_tiller.cpp").read_text()
        task = source[source.index("void motorTask"):source.index("// hid_task:")]
        self.assertNotIn("sensor.update();", task)
        self.assertLess(task.index("motor.loopFOC();"), task.index("motor.move(u);"))

    def test_model_constants_match_firmware_header(self):
        header = (pathlib.Path(__file__).parents[1] / "piloteye_tiller" / "return_profile.h").read_text()
        expected = {
            "ACTIVE_SPRING_SCALE": ACTIVE,
            "RETURN_SPRING_SCALE": RETURN,
            "CENTER_QUIET_RAD": QUIET,
            "CENTER_FULL_RAD": FULL,
            "MOTION_THRESHOLD_RAD_S": MOTION,
            "IDLE_DELAY_MS": IDLE_MS,
            "RETURN_RAMP_MS": RAMP_MS,
        }
        for name, value in expected.items():
            match = re.search(rf"{name}\s*=\s*([0-9.]+)", header)
            self.assertIsNotNone(match, name)
            self.assertEqual(float(value), float(match.group(1)), name)

    def test_spring_never_disappears_while_steering(self):
        scale, error, blend = Profile().update(300, -0.4, 0.5)
        self.assertEqual(ACTIVE, scale)
        self.assertLess(error, 0)
        self.assertEqual(0.0, blend)

    def test_idle_return_has_no_torque_step(self):
        p = Profile()
        self.assertEqual(ACTIVE, p.update(IDLE_MS - 1, -0.4, 0.0)[0])
        self.assertEqual(ACTIVE, p.update(IDLE_MS, -0.4, 0.0)[0])
        mid = p.update(IDLE_MS + RAMP_MS // 2, -0.4, 0.0)[0]
        self.assertGreaterEqual(mid, ACTIVE)
        self.assertLessEqual(mid, RETURN)
        self.assertAlmostEqual(RETURN, p.update(IDLE_MS + RAMP_MS, -0.4, 0.0)[0])

    def test_new_away_motion_immediately_cancels_strong_return(self):
        p = Profile()
        p.update(IDLE_MS, -0.4, 0.0)
        self.assertGreater(p.update(IDLE_MS + RAMP_MS, -0.3, -0.2)[2], 0.99)
        scale, _, blend = p.update(IDLE_MS + RAMP_MS + 1, -0.3, 0.2)
        self.assertEqual(ACTIVE, scale)
        self.assertEqual(0.0, blend)

    def test_center_deadzone_removes_zero_crossing_chatter(self):
        p = Profile()
        self.assertEqual(0.0, p.update(0, 0.031, 0.0)[1])  # captured vibrating rest point
        self.assertEqual(0.0, p.update(1, -QUIET, 0.0)[1])
        self.assertGreater(p.update(2, 0.060, 0.0)[1], 0.0)
        self.assertAlmostEqual(0.090, p.update(3, 0.090, 0.0)[1])

    def test_natural_motion_toward_center_does_not_cancel_return(self):
        p = Profile()
        p.update(IDLE_MS, -0.4, 0.0)
        self.assertGreater(p.update(IDLE_MS + RAMP_MS, -0.2, -0.5)[2], 0.99)

    def test_window_velocity_is_zero_at_rest_and_decays_after_stop(self):
        # Python twin of WindowVelocity. SimpleFOC 2.4 held 0.19..0.79 rad/s on a
        # still handle (hardware, 29.09); the window estimate must return 0.
        count, update = 2 * math.pi / 4096, window_velocity()

        t, a = 0, 3.0
        for _ in range(300):
            t += 1000
            a += 0.0008
            moving = update(round(a / count) * count, t)
        self.assertGreater(moving, 0.5)
        for _ in range(40):
            t += 1000
            v = update(round(a / count) * count + count * (t // 1000 % 2), t)  # 1-count jitter
        self.assertEqual(0.0, v)

    def test_window_velocity_lag_is_short_during_fast_return(self):
        # 25 Hz, 0.08 rad oscillation = the recorded release limit cycle. The
        # fixed 20 ms window lagged ~10 ms (90 deg); the damper needs < 2 ms.
        count, f, amp = 2 * math.pi / 4096, 25.0, 0.08
        update, lags = window_velocity(), []
        for i in range(400):
            t = i * 1000
            a = amp * math.sin(2 * math.pi * f * t * 1e-6)
            v = update(3.7 + round(a / count) * count, t)
            if i > 100 and abs(v) > 0:
                # delay d such that true velocity at t-d equals the estimate
                true_now = amp * 2 * math.pi * f * math.cos(2 * math.pi * f * t * 1e-6)
                lags.append(abs(v - true_now))
        rms_err = math.sqrt(sum(e * e for e in lags) / len(lags))
        peak_v = amp * 2 * math.pi * f
        self.assertLess(rms_err / peak_v, 0.25)   # fixed 20 ms window: ~1.0 (90 deg)


class DampingVelocityTests(unittest.TestCase):
    SRC = (pathlib.Path(__file__).parents[1] / "piloteye_tiller" / "piloteye_tiller.cpp").read_text()

    def test_voltage_is_applied_in_the_same_pass(self):
        # loopFOC() applies current_sp one pass (~1 ms) late; v10 applies u right away.
        task = self.SRC[self.SRC.index("void motorTask"):self.SRC.index("// hid_task:")]
        move = task.index("motor.move(u);")
        apply = task.index("motor.setPhaseVoltage(")
        self.assertLess(move, apply)
        self.assertIn("if (motor.enabled)", task[move:apply])

    def test_never_writes_the_burn_register(self):
        self.assertNotRegex(self.SRC, r"write\(\s*0x[fF][fF]\s*\)")

    def test_quiet_at_rest_with_sensor_dither(self):
        count, update = 2 * math.pi / 4096, damping_velocity()
        base, out = 3.7, []
        for i in range(2000):
            jitter = count * (i % 2) if i < 1000 else (count if i % 97 == 0 else 0.0)
            out.append(update(base + jitter, i * 1000))
        self.assertEqual([0.0] * 1500, out[500:])

    def test_tracks_hand_motion_with_short_lag(self):
        count, update = 2 * math.pi / 4096, damping_velocity()
        f, amp, est = 2.0, 0.3, []
        for i in range(1500):
            a = amp * math.sin(2 * math.pi * f * i * 1e-3)
            est.append(update(3.7 + round(a / count) * count, i * 1000))
        ref = lambda k, lag: amp * 2 * math.pi * f * math.cos(2 * math.pi * f * (k - lag) * 1e-3)
        lag = min(range(0, 20), key=lambda L: sum((est[k] - ref(k, L)) ** 2 for k in range(500, 1500)))
        self.assertLessEqual(lag, 6)

    def test_closed_loop_sim_is_stable_at_knob_max(self):
        # Plant fitted to the 29.09 recordings; this sim reproduces both hardware
        # oscillations (v7 25 Hz, v9 at Kd 1.6). DAMP_MAX must stay inside its limit.
        sys.path.insert(0, str(pathlib.Path(__file__).parents[1] / "diag-recordings"))
        import loop_sim as L
        kd = float(re.search(r"DAMP_MAX\s*=\s*([0-9.]+)f", self.SRC).group(1))
        for plant in (dict(g=180, fric=42, sens_tau=0.0012), dict(g=230, fric=15, sens_tau=0.0012)):
            for scen in (dict(x0=-1.5), dict(x0=0.03), dict(kick=(0.2, 0.35, 0.05))):
                for est in (L.complementary(*self.velocity_defaults()), L.diff_biquad(70, 0.55, dz=0.3)):
                    out = L.run(est, kd, t_end=1.4, apply_now=True, **plant, **scen)
                    self.assertLess(L.osc_metrics(out, 1.0)[0], 0.3, (plant, scen))

    def velocity_defaults(self):
        obs = float(re.search(r"g_obsHz\s*=\s*([0-9.]+)f", self.SRC).group(1))
        split = float(re.search(r"g_splitHz\s*=\s*([0-9.]+)f", self.SRC).group(1))
        return obs, split

    def test_damper_still_damps_a_hand_driven_motion(self):
        # v11b (pure observer 12 Hz) read a hand-driven wiggle at -0.2 of its speed on
        # hardware: quiet, but no damping ("looser" handle). The sim reproduces that.
        sys.path.insert(0, str(pathlib.Path(__file__).parents[1] / "diag-recordings"))
        import loop_sim as L, comp_sim as CS
        self.assertLess(CS.hand_run(lambda: L.observer(12, 0.1))[0], 0.0)
        obs, split = self.velocity_defaults()
        gain, ripple = CS.hand_run(lambda: L.complementary(obs, split))
        self.assertGreater(gain, 0.5)
        self.assertLess(ripple, 0.04)

    def test_factory_mode_matches_the_stock_image(self):
        # Constants recovered from backup/app0.bin (29.09): P_angle 25, LPF angle 10 ms,
        # LPF velocity 100 ms, 12 V, knob maps 20..100/10 and 30..150/100 on the raw ADC.
        for name, val in (("STOCK_P_ANGLE", 25.0), ("STOCK_TF_ANGLE", 0.01), ("STOCK_TF_VEL", 0.1),
                          ("STOCK_V_LIMIT", 12.0), ("STOCK_RAMP_V_S", 1000.0)):
            self.assertEqual(val, float(re.search(name + r"\s*=\s*([0-9.]+)f", self.SRC).group(1)), name)
        self.assertIn("map((long)(rF + 0.5f), 0, 4095, 20, 100) / 10", self.SRC)
        # 12 V overmodulates SVPWM (notchy torque, owner 29.09): every mode is capped at 6.9 V.
        self.assertIn("fminf(STOCK_V_LIMIT, V_LINEAR_MAX)", self.SRC)
        self.assertIn("FORCE_V_MAX = V_LINEAR_MAX", self.SRC)
        self.assertIn("map((long)(rD + 0.5f), 0, 4095, 30, 150) / 100.0f", self.SRC)

    def test_factory_law_is_stable_at_knob_extremes(self):
        sys.path.insert(0, str(pathlib.Path(__file__).parents[1] / "diag-recordings"))
        import loop_sim as L
        for kv in (0.3, 1.5):
            for vl in (2.0, 10.0):
                for scen in (dict(x0=-1.5), dict(x0=0.03), dict(kick=(0.2, 0.35, 0.3))):
                    out = L.run(L.stock_velocity(), 0.0, t_end=1.6, apply_now=True, vlim=12.0,
                                law=L.stock_law(kv, vl), g=230, fric=15, sens_tau=0.0012, **scen)
                    self.assertLess(L.osc_metrics(out, 1.2)[0], 0.3, (kv, vl, scen))

    def test_factory_mode_is_default_and_observer_fed_with_the_applied_voltage(self):
        self.assertRegex(self.SRC, r"volatile int\s+g_ctrlMode\s*=\s*2;")
        task = self.SRC[self.SRC.index("void motorTask"):self.SRC.index("// hid_task:")]
        self.assertLess(task.index("motor.setPhaseVoltage(uApplied"), task.index("dampingVelocity.feedU(uApplied)"))

    def test_observer_is_stable_at_max_bandwidth_and_slowest_loop(self):
        # 'W' caps the explicit-Euler observer at 40 Hz: error-dynamics spectral radius
        # 0.82 at the slowest recorded pass (1.9 ms); 60 Hz would be 2.2 (diverges).
        self.assertIn("constrain(x, 5.0f, 40.0f)", self.SRC)
        self.assertLess(observer_error_radius(40, 0.0019), 0.9)
        self.assertGreater(observer_error_radius(60, 0.0019), 1.0)


class IdleSleepTests(unittest.TestCase):
    """v14: driver off after the handle rests untouched; python twin of struct IdleSleep."""
    SRC = DampingVelocityTests.SRC
    DEG = 0.0174533
    COUNT = 2 * math.pi / 4096

    def const(self, name):
        m = re.search(name + r"\s*=\s*([0-9.]+)f?\s*(\*\s*0\.0174533f)?", self.SRC)
        return float(m.group(1)) * (self.DEG if m.group(2) else 1.0)

    def twin(self):
        still, near, wake = self.const("IDLE_STILL_RAD"), self.const("IDLE_NEAR_RAD"), self.const("IDLE_WAKE_RAD")

        class Twin:
            asleep, still_ref, still_ms, sleep_angle = False, 0.0, 0, 0.0

            def update(s, a, center, now, timeout_s, busy=False):
                if s.asleep:
                    if abs(a - s.sleep_angle) > wake or busy or timeout_s == 0:
                        s.asleep, s.still_ref, s.still_ms = False, a, now
                        return -1
                    return 0
                if abs(a - s.still_ref) > still or busy:
                    s.still_ref, s.still_ms = a, now
                    return 0
                if timeout_s and abs(a - center) < near and now - s.still_ms >= timeout_s * 1000:
                    s.asleep, s.sleep_angle = True, a
                    return 1
                return 0
        return Twin()

    def simulate(self, trace, center=0.0, timeout_s=60):
        t, edges = self.twin(), []
        for ms, a in trace:
            e = t.update(a, center, ms, timeout_s)
            if e:
                edges.append((ms, e))
        return edges

    def noise(self, ms):   # AS5600 dither: +-2 counts
        return ((ms * 7919) % 5 - 2) * self.COUNT

    def test_constants(self):
        self.assertEqual(self.const("g_idleSleepS"), 60)
        self.assertAlmostEqual(self.const("IDLE_WAKE_RAD"), 1 * self.DEG, places=6)
        self.assertAlmostEqual(self.const("IDLE_STILL_RAD"), 0.5 * self.DEG, places=6)
        # wake threshold far above the sensor dither, still window above it too
        self.assertGreater(self.const("IDLE_WAKE_RAD"), 8 * self.COUNT)
        self.assertGreater(self.const("IDLE_STILL_RAD"), 4 * self.COUNT)

    def test_sleeps_after_a_minute_at_rest_despite_dither(self):
        edges = self.simulate([(ms, self.noise(ms)) for ms in range(0, 70000)])
        self.assertEqual(edges, [(60000, 1)])

    def test_friction_parked_off_center_still_sleeps(self):
        # soft knob end: friction can park the factory law ~1.8 deg off center
        edges = self.simulate([(ms, 1.8 * self.DEG + self.noise(ms)) for ms in range(0, 61000)])
        self.assertEqual(edges, [(60000, 1)])

    def test_does_not_sleep_when_held_off_center(self):
        edges = self.simulate([(ms, 10 * self.DEG + self.noise(ms)) for ms in range(0, 120000)])
        self.assertEqual(edges, [])

    def test_slow_movement_restarts_the_minute(self):
        # nudged 1 deg at 40 s: sleeps 60 s after the nudge, not at 60 s
        trace = [(ms, (self.DEG if ms >= 40000 else 0.0) + self.noise(ms)) for ms in range(0, 101000)]
        self.assertEqual(self.simulate(trace), [(100000, 1)])

    def test_dither_never_wakes_and_a_touch_does(self):
        trace = [(ms, self.noise(ms)) for ms in range(0, 200000)]
        trace += [(200000 + i, i * 0.003 * self.DEG) for i in range(1, 1000)]   # 3 deg/s grab
        edges = self.simulate(trace)
        self.assertEqual(edges[0], (60000, 1))
        self.assertEqual(edges[1][1], -1)
        self.assertLess(edges[1][0] - 200000, 350)                              # woke within 1 deg
        self.assertEqual(len(edges), 2)

    def test_sleep_disabled_with_zero_timeout(self):
        self.assertEqual(self.simulate([(ms, 0.0) for ms in range(0, 200000)], timeout_s=0), [])

    def test_firmware_wiring(self):
        task = self.SRC[self.SRC.index("void motorTask"):self.SRC.index("// hid_task:")]
        # driver really switched off / on, only from the motor task
        self.assertIn("if (sleepEdge > 0) motor.disable();", task)
        self.assertIn("else if (sleepEdge < 0) motor.enable();", task)
        self.assertEqual(self.SRC.count("motor.disable()"), 1)
        # calibration wakes the driver before it drives the motor
        self.assertLess(task.index("motor.enable(); g_asleep = false;"), task.index("g_autocal_request = false; autoCalibrate();"))
        # soft start caps u before move(), and the factory ramp restarts from the capped u
        self.assertLess(task.index("idle.capScale(nowMs)"), task.index("motor.move(u)"))
        self.assertIn("stU = u;", task)
        self.assertIn("WAKE_RAMP_MS   = 200", self.SRC)
        # the kick stand test keeps the driver awake
        self.assertIn("g_idleSleepS, kick != 0)", task)


def observer_error_radius(f, dt, c=70.0):
    p = 2 * math.pi * f
    L = [3 * p * dt, 3 * p * p * dt, p ** 3 * dt]
    A = [[1, dt, 0], [0, 1 - c * dt, dt], [0, 0, 1]]
    E = [[A[i][j] - L[i] * A[0][j] for j in range(3)] for i in range(3)]
    v, m = [1.0, 0.3, 0.1], 0.0
    for _ in range(3000):
        v = [sum(E[i][k] * v[k] for k in range(3)) for i in range(3)]
        m = max(abs(x) for x in v)
        v = [x / m for x in v]
    return m


def damping_velocity(fc=70.0, q=0.55, dead=0.3):
    """Python twin of DampingVelocity in piloteye_tiller.cpp."""
    w = 2 * math.pi * fc / 1000.0; al = math.sin(w) / (2 * q); cw = math.cos(w); a0 = 1 + al
    b0 = (1 - cw) / 2 / a0; b1 = (1 - cw) / a0; b2 = b0; a1 = -2 * cw / a0; a2 = (1 - al) / a0
    s = {"have": False, "a": 0.0, "t": 0, "raw": 0.0, "z1": 0.0, "z2": 0.0}

    def update(angle, now):
        if not s["have"]:
            s.update(have=True, a=angle, t=now, raw=0.0)
        else:
            dt = now - s["t"]
            if dt > 20000:
                s.update(raw=0.0, z1=0.0, z2=0.0)
            elif dt > 0:
                s["raw"] = (angle - s["a"]) / (dt * 1e-6)
            s["a"], s["t"] = angle, now
        x = s["raw"]
        y = b0 * x + s["z1"]
        s["z1"] = b1 * x - a1 * y + s["z2"]
        s["z2"] = b2 * x - a2 * y
        return math.copysign(max(abs(y) - dead, 0.0), y)

    return update


def window_velocity():
    """Python twin of WindowVelocity in piloteye_tiller.cpp."""
    count, win, vmin, fast_counts, n = 2 * math.pi / 4096, 20000, 2000, 8.0, 64
    buf_a, buf_t, state = [0.0] * n, [0] * n, {"count": 0, "head": 0}

    def update(angle, now):
        buf_a[state["head"]], buf_t[state["head"]] = angle, now
        newest = state["head"]
        state["head"] = (state["head"] + 1) % n
        state["count"] = min(state["count"] + 1, n)
        for k in range(1, state["count"]):
            i = (newest - k + n) % n
            age = now - buf_t[i]
            d = angle - buf_a[i]
            fast = age >= vmin and abs(d) >= fast_counts * count
            if fast or age >= win or k == state["count"] - 1:
                if age < vmin:
                    return 0.0
                d = math.copysign(max(abs(d) - 1.05 * count, 0.0), d)
                return d / (age * 1e-6)
        return 0.0

    return update



if __name__ == "__main__":
    unittest.main()
