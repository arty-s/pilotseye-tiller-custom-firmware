/*
 * PilotsEYE / Simplayer A320-B737 USB Tiller  -  clean replacement firmware
 * ------------------------------------------------------------------------
 * Target : GoouuTech ESP32-S3-N16R8 on carrier "FOC-USB-V6"
 * Stack  : Arduino-ESP32 core 3.3.12 (ESP-IDF 5.5, TinyUSB 0.21) +
 *          SimpleFOC 2.4.0 + native TinyUSB HID (ESP32 "USB" stack).
 *
 * WHY THIS EXISTS
 *   The stock firmware randomly "dies until you press RESET": the AS5600 angle
 *   sensor is read over I2C inside the FOC loop with no timeout / no error
 *   handling. Motor EMI (or a glitch) can leave the I2C bus stuck; the read then
 *   blocks forever, the loop stops, USB stops being serviced and the host drops
 *   the device until a hardware reset re-inits I2C.
 *
 *   This firmware keeps the SAME USB identity (so MSFS / X-Plane bindings are
 *   unchanged) and fixes the root cause:
 *     - a custom AS5600 reader that NEVER blocks (bounded Wire timeout) and
 *       recovers a stuck bus automatically (9-clock unstick + re-init);
 *     - both RTOS tasks are on the Task Watchdog, so any residual hang triggers
 *       a clean reboot instead of a dead device;
 *     - the HID task always ships the last valid axis, so a momentary sensor
 *       hiccup never shows up as a disconnect.
 *
 * STARTUP HOMING (MOZA-style): on every boot the motor gently drives the stick to
 * each physical end-stop, takes the two extremes as the axis range and the midpoint
 * as center. Re-run anytime with the 'A' serial command or a long button press.
 *
 * PINOUT (reverse-engineered from the stock image)
 *   Motor  DRV8313 / BLDCDriver3PWM : PWM_A=GPIO4  PWM_B=GPIO5  PWM_C=GPIO6  EN=GPIO7
 *   Sensor AS5600 (I2C 0x36, 400k)  : SDA=GPIO42   SCL=GPIO41
 *   Button "Pedal Disc"             : GPIO38  INPUT_PULLUP (active-LOW)
 *   Knobs (read once at boot)       : force=GPIO1  damping=GPIO2  (ADC1)
 *   Motor pole pairs                : 14
 *   USB HID : VID 0x031F  PID 0x9001  "PilotsEYE-Studios" / "PilotsEYE-"
 *             report ID 3, 1 button + 16-bit X axis (0..65535), self-powered
 *
 * BUILD (Tools / FQBN)
 *   Board            : ESP32S3 Dev Module
 *   USB Mode         : USB-OTG (TinyUSB)      <-- REQUIRED (see guard below)
 *   USB CDC On Boot  : Disabled               (we create USBCDC UsbSerial manually so VID/PID apply)
 *   Flash Mode       : DIO 80MHz              (matches stock bootloader header)
 *   Flash Size       : 16MB
 *   PSRAM            : OPI PSRAM
 *   Partition        : Huge APP (3MB No OTA)
 */

#if ARDUINO_USB_MODE
#error "Set USB Mode to 'USB-OTG (TinyUSB)'. This firmware needs the TinyUSB HID stack (ARDUINO_USB_MODE must be 0)."
#endif

#include <Arduino.h>          // required: a .cpp does not get this auto-injected
#include <Wire.h>
#include <Preferences.h>
#include <SimpleFOC.h>
#include "USB.h"
#include "USBHID.h"
#include "USBCDC.h"
#include "esp_task_wdt.h"
#include "esp_system.h"
#include "soc/usb_struct.h"
#include "tusb.h"
#include "device/dcd.h"
#include "return_profile.h"

// IDF 5 no longer exports the old convenience macro, while the ESP32-S3
// peripheral still has seven hardware IN endpoint register banks (EP0..EP6).
#ifndef USB_IN_EP_NUM
#define USB_IN_EP_NUM 7
#endif

// ----------------------------------------------------------------------------
// Configuration
// ----------------------------------------------------------------------------
static const int PIN_PWM_A = 4;
static const int PIN_PWM_B = 5;
static const int PIN_PWM_C = 6;
static const int PIN_EN    = 7;
static const int PIN_SDA   = 42;
static const int PIN_SCL   = 41;
static const int PIN_BTN   = 38;   // active LOW
static const int  PIN_KNOB_FORCE = 1;
static const int  PIN_KNOB_DAMP  = 2;
static const bool KNOB_FORCE_INVERT = true;   // this unit's force pot is wired reversed (knob max = low ADC)
static const bool KNOB_DAMP_INVERT  = false;

static const int   MOTOR_POLE_PAIRS = 14;
// Confirmed PSU: 12 V / 2 A. FOC duty scales with Uq / SUPPLY_VOLTAGE.
// A 24 V setting on this PSU halves the commanded motor voltage.
static const float SUPPLY_VOLTAGE   = 12.0f;

// USB identity (must match the stock device so sim profiles keep working).
// NB: the ESP32-S3 variant's pins_arduino.h already #defines USB_VID/USB_PID/
// USB_MANUFACTURER/USB_PRODUCT, and TinyUSB defines HID_REPORT_ID - so ours are
// deliberately prefixed to avoid the macro collision.
static const uint16_t TILLER_VID = 0x031F;
static const uint16_t TILLER_PID = 0x9001;
static const char*    TILLER_MANUFACTURER = "PilotsEYE-Studios";
static const char*    TILLER_PRODUCT      = "PilotsEYE-";
static const uint8_t  TILLER_REPORT_ID    = 3;

// Self-centering spring+damper defaults (knobs override force/damp at boot).
static const float KP_SPRING   = 20.0f;  // knob-max stiffness requested by owner
static const float FORCE_V_MIN = 0.5f;   // knob-min -> max centering force (volts)
// Highest clean Uq: SpaceVectorPWM is linear only up to Vdc/sqrt(3) = 6.93 V at 12 V. Above
// that the phase voltages clip and the torque ripples 6x per electrical turn (84x per rev
// = a notch every 4.3 deg) for at most ~10 % more torque. Owner 29.09: 12 V felt "ribbed",
// 6.9 V smooth -> 6.9 V is the knob maximum in every mode.
static const float V_LINEAR_MAX = 6.9f;
static const float FORCE_V_MAX = V_LINEAR_MAX;   // knob-max
// Scale spring slope with the force ceiling: turning down softens the whole response.
static float springForForce(float forceV) { return KP_SPRING * forceV / FORCE_V_MAX; }
static const float DAMP_MIN    = 0.0f;   // knob-min -> damping (V per rad/s)
static const float DAMP_MAX    = 1.6f;   // knob-max, doubled damping range
// Damper velocity comes from our own estimator, NOT from SimpleFOC: since 2.4.0
// Sensor::getVelocity() recomputes only when the angle changes and otherwise holds
// the last value forever, so a resting handle keeps a stale 0.2..0.8 rad/s and the
// damper pushes on it -> vibration (proved on hardware 29.09: spring-only quiet,
// spring + damper 1.6 vibrates).
// History (all recorded with 'R', diag-recordings/): a fixed 20 ms window lagged
// ~10 ms -> 25 Hz limit cycle on a hard release (v7); the adaptive window (v8/v9)
// stretched across a whole half-period on a +-8 count oscillation, lagged > 90 deg
// and turned the damper into a stiff spring -> 33 Hz center oscillation (v9).
// v10 (DampingVelocity): first difference of consecutive samples over their real
// timestamps -> 2nd-order low-pass (OpenFFBoard-style) -> soft dead zone, and the
// voltage is applied in the same pass. Closed-loop sim fitted to the recordings
// (diag-recordings/loop_sim.py, cand_sim.py): v9 stable to Kd 0.8, v10 to ~2.4.
// Hardware stand 29.09 ('J0.3' = release from 17 deg, stand.py / variants.py):
// 100 Hz cutoff oscillates (~80 Hz) from Kd 2.2; 70 Hz holds Kd 2.4, oscillates at
// 2.8 -> DAMP_MAX 1.6 keeps a 1.5x margin. AS5600 SF 4x/8x ('Q') made no repeatable
// difference, so the chip stays at its power-on default.
static const float    AS5600_COUNT_RAD = 6.28318530718f / 4096.0f; // one sensor count
static const float    VEL_LPF_Q        = 0.55f;
// Live-tunable: 'W' low-pass cutoff (Hz), 'N' dead zone (rad/s; kills 1-count
// sensor dither at rest, sim: rest buzz 0.15 -> 0.03 V at Kd 1.6).
volatile float g_velCutHz   = 70.0f;
volatile float g_velDeadRad = 0.1f;
// v11 (default, 'O' toggles back to the v10c biquad): model-based observer. The
// owner felt a weak buzz holding / slowly wiggling the handle with v10c; recorded
// 29.09 (hold_ab.py): damper term 0.08..0.14 V of 40..100 Hz ripple from single-
// count sensor steps, 0.01 V with the damper off. The observer predicts the motion
// from the voltage it applies (plant fitted to the recordings: theta'' = G*U - C*w
// + d, d = hand/friction estimate) and only corrects with the sensor, so a low
// bandwidth costs no lag on a free release. Sim: ripple 0.05 -> 0.03 V at 20 Hz,
// stable to Kd >= 4 with G -35..+45 %, C -40..+60 %, friction 5..90, 2x sensor lag.
// Hardware 29.09: release from 17 deg ('J0.3') quiet up to Kd 4.0 at 20/12/8 Hz (biquad:
// oscillates at 2.8). Hold/wiggle ripple (hold_ab2.py): biquad 0.10, observer 20 Hz 0.04,
// 12 Hz 0.025 V. AS5600 HYST 2 made it worse (0.06), FTH 2 no change -> both stay off.
static const float OBS_G = 180.0f;     // rad/s^2 per volt (plant_fit.py, v7 chunk 4)
static const float OBS_C = 70.0f;      // 1/s, back-EMF + viscous
// 'O<n>': 0 = our spring + damper with the observer velocity, 1 = the same with
// the v10c biquad velocity, 2 = the factory control law (below, default).
volatile int   g_ctrlMode    = 2;     // owner 29.09: factory mode is the default (quiet at center, liked it)
// Factory law ('O2'), recovered from the stock app image 29.09 (backup/app0.bin, SimpleFOC
// 2.3.4 angle mode; there the knobs are read at boot only, here they stay live):
//   vel_sp = clamp(25 * (center - theta_f), +-vlim);  u = Kv * (vel_sp - w_f), ramp 1000 V/s
//   theta_f = LPF 10 ms of the angle, w_f = LPF 100 ms of the per-pass velocity, 12 V cap.
//   red  GPIO1: vlim = map(raw, 0, 4095, 20, 100) / 10  -> 2..10 rad/s in integer steps
//   blue GPIO2: Kv   = map(raw, 0, 4095, 30, 150) / 100 -> 0.30..1.50 V per rad/s
// The stock code maps the RAW ADC with no inversion, so on this unit (red pot wired
// reversed, KNOB_FORCE_INVERT) the red knob works the other way round than in mode 0,
// exactly as it did with the factory firmware. Blue sets the stiffness too (25*Kv).
static const float STOCK_P_ANGLE  = 25.0f;
static const float STOCK_TF_ANGLE = 0.01f;
static const float STOCK_TF_VEL   = 0.1f;
static const float STOCK_RAMP_V_S = 1000.0f;
static const float STOCK_V_LIMIT  = 12.0f;   // stock value; applied as min(12, V_LINEAR_MAX)
volatile float g_stockVlim = 6.0f;     // rad/s
volatile float g_stockKv   = 0.9f;     // V per rad/s
volatile float g_obsHz       = 12.0f;  // 'W' in observer mode (triple pole)
// v11c: complementary split. The pure observer (v11b, 12 Hz) was quiet but the owner
// felt the handle "looser": with a hand on it, the spring force the hand holds looks
// to the model like an acceleration toward center, so the estimate read a hand-driven
// motion at -0.2..+0.3 of its true speed (recorded 29.09, hold2_p1/p5) -> no damping.
// Now v = v_obs + LPF_split(v_meas - v_obs): hand motion (<= split) from the measured
// difference (unbiased), release / oscillation modes from the observer (no lag).
// Sim (comp_sim.py, reproduces the -0.18): hand gain 0.70 @1 Hz, 0.61 @2.5 Hz, ripple
// 0.024 V (v11b 0.015, v10c 0.052), stable to Kd >= 4. 'Y' 0 = pure observer.
volatile float g_splitHz     = 20.0f;  // 'Y'. Owner A/B 29.09 (hold_ab3.py): W16 Y12 ripple 0.044 V, W12 Y20 0.029 V, hand gain 0.85..0.90 both; owner preferred the middle
// v14 idle sleep (owner 29.09): the handle rests near center untouched for g_idleSleepS
// -> the DRV8313 is switched off (EN low, phases high-Z: silent, no hold, no brake).
// "Untouched" = stays within IDLE_STILL_RAD of one spot; the factory law has no
// integrator, so at the soft knob end friction can park it up to ~1.8 deg off center,
// hence the looser IDLE_NEAR_RAD. The sensor keeps running; moving the handle more than
// IDLE_WAKE_RAD from where it fell asleep re-enables the driver, and the voltage cap
// then ramps 0 -> full over WAKE_RAMP_MS so a fast grab does not meet the full damper
// at once. 'Os<n>' sets the timeout in seconds, 'Os0' disables sleep.
static const float    IDLE_STILL_RAD = 0.5f * 0.0174533f;
static const float    IDLE_NEAR_RAD  = 3.0f * 0.0174533f;
static const float    IDLE_WAKE_RAD  = 1.0f * 0.0174533f;
static const uint32_t WAKE_RAMP_MS   = 200;
volatile uint32_t g_idleSleepS = 60;
volatile bool     g_asleep     = false;
volatile uint32_t g_idleForMs  = 0;    // how long the handle has been still (status)

struct IdleSleep {
  bool asleep = false;
  float stillRef = 0.0f, sleepAngle = 0.0f;
  uint32_t stillMs = 0, wakeMs = 0;
  void reset(float a, uint32_t now) { stillRef = a; stillMs = now; }
  // Returns +1 when the caller must disable the driver, -1 to enable it, 0 nothing.
  int update(float a, float center, uint32_t now, uint32_t timeoutS, bool busy) {
    if (asleep) {
      if (fabsf(a - sleepAngle) > IDLE_WAKE_RAD || busy || timeoutS == 0) {
        asleep = false; wakeMs = now | 1; reset(a, now);
        return -1;
      }
      return 0;
    }
    if (fabsf(a - stillRef) > IDLE_STILL_RAD || busy) { reset(a, now); return 0; }
    if (timeoutS && fabsf(a - center) < IDLE_NEAR_RAD && now - stillMs >= timeoutS * 1000UL) {
      asleep = true; sleepAngle = a;
      return +1;
    }
    return 0;
  }
  // Voltage-cap scale after a wake: 0 -> 1 over WAKE_RAMP_MS.
  float capScale(uint32_t now) {
    if (!wakeMs) return 1.0f;
    uint32_t t = now - wakeMs;
    if (t >= WAKE_RAMP_MS) { wakeMs = 0; return 1.0f; }
    return (float)t / (float)WAKE_RAMP_MS;
  }
};
// driveToStop's stall detector keeps the adaptive window (WindowVelocity): there
// it only has to tell "moving" from "pressed against the stop".
static const uint32_t VEL_WINDOW_US    = 20000;                   // longest window (rest / slow)
static const uint32_t VEL_MIN_US       = 2000;                    // shortest window (loop dt jitter 0.4..1.9 ms)
static const float    VEL_FAST_COUNTS  = 8.0f;                    // close the window early after this move

// Auto-calibration: gently drive to each mechanical stop and record the travel.
static const float CAL_PROBE_V   = 2.0f;   // preserve previous physical homing voltage (4 V at 24 V scaling)
static const float CAL_MOVE_VEL  = 0.5f;   // rad/s: above this the stick is "moving"
static const float CAL_STALL_VEL = 0.15f;  // rad/s: below this (after it moved) = "at the stop"

// Sensor-health / recovery thresholds
static const uint32_t I2C_TIMEOUT_MS       = 8;    // Wire never blocks longer than this
static const uint32_t I2C_ERRORS_TO_RECOVER = 4;   // consecutive errors before bus unstick

// Task watchdog: if a task stalls this long -> panic reboot (auto-recovery).
static const int WDT_TIMEOUT_S = 4;
static const uint32_t SUPERVISOR_STALL_MS = 5000;
static const uint32_t USB_REPORT_STALL_MS = 1000;
static const uint32_t USB_RECONNECT_OFF_MS = 250;
static const uint32_t USB_RECOVERY_VERIFY_MS = 6000;
// Temporary staged boot probe for the IDF 5 migration. 0 = normal firmware;
// 1 = USB only, 2 = sensor, 3 = PWM driver, 4 = motor.init, 5 = initFOC.
static const int BOOT_DIAG_STOP_AFTER = 0;

static const uint32_t CALIB_MAGIC = 0xC0FFEE02;    // reuse stock magic marker

// ----------------------------------------------------------------------------
// HID report descriptor - captured VERBATIM from the stock device (53 bytes,
// read live over hidapi) so enumeration is byte-identical and MSFS / X-Plane
// bindings never change. Report ID 3, 1 button + 16-bit X axis (0..65535);
// input-only (no output reports -> no native FFB channel, matches stock).
// Wire payload after the report id = 3 bytes -> host sees a 4-byte input report.
// ----------------------------------------------------------------------------
static const uint8_t HID_REPORT_DESCRIPTOR[] = {
  0x05, 0x01,                    // Usage Page (Generic Desktop)
  0x09, 0x05,                    // Usage (Gamepad)
  0xA1, 0x01,                    // Collection (Application)
  0x85, 0x03,                    //   Report ID (3)
  0x05, 0x09,                    //   Usage Page (Button)
  0x19, 0x01,                    //   Usage Minimum (Button 1)
  0x29, 0x01,                    //   Usage Maximum (Button 1)
  0x15, 0x00,                    //   Logical Minimum (0)
  0x25, 0x01,                    //   Logical Maximum (1)
  0x75, 0x01,                    //   Report Size (1)
  0x95, 0x01,                    //   Report Count (1)
  0x81, 0x02,                    //   Input (Data,Var,Abs)
  0x75, 0x07,                    //   Report Size (7)
  0x95, 0x01,                    //   Report Count (1)
  0x81, 0x03,                    //   Input (Const,Var,Abs) - padding
  0x05, 0x01,                    //   Usage Page (Generic Desktop)
  0x09, 0x01,                    //   Usage (Pointer)
  0xA1, 0x00,                    //   Collection (Physical)
  0x09, 0x30,                    //     Usage (X)
  0x15, 0x00,                    //     Logical Minimum (0)
  0x27, 0xFF, 0xFF, 0x00, 0x00,  //     Logical Maximum (65535)
  0x75, 0x10,                    //     Report Size (16)
  0x95, 0x01,                    //     Report Count (1)
  0x81, 0x02,                    //     Input (Data,Var,Abs)
  0xC0,                          //   End Collection (Physical)
  0xC0                           // End Collection (Application)
};

USBHID HID;
volatile uint32_t g_hidNotReady = 0;
volatile uint32_t g_hidSendFail = 0;

class TillerHID : public USBHIDDevice {
public:
  TillerHID() {
    static bool once = false;
    if (!once) { once = true; HID.addDevice(this, sizeof(HID_REPORT_DESCRIPTOR)); }
  }
  void begin() { HID.begin(); }
  uint16_t _onGetDescriptor(uint8_t *dst) override {
    memcpy(dst, HID_REPORT_DESCRIPTOR, sizeof(HID_REPORT_DESCRIPTOR));
    return sizeof(HID_REPORT_DESCRIPTOR);
  }
  bool send(bool button, uint16_t axis) {
    uint8_t r[3];
    r[0] = button ? 0x01 : 0x00;
    r[1] = (uint8_t)(axis & 0xFF);
    r[2] = (uint8_t)(axis >> 8);
    if (!HID.ready()) {
      g_hidNotReady++;
      return false;
    }
    bool ok = HID.SendReport(TILLER_REPORT_ID, r, sizeof(r));
    if (!ok) g_hidSendFail++;
    return ok;
  }
};
TillerHID tillerHID;

// ----------------------------------------------------------------------------
// Robust AS5600 sensor: bounded I2C reads + automatic bus recovery.
// Implements SimpleFOC's Sensor interface (getSensorAngle in radians 0..2PI).
// ----------------------------------------------------------------------------
class AS5600Robust : public Sensor {
public:
  volatile uint32_t err_streak = 0;   // consecutive failed reads
  volatile uint32_t recoveries = 0;   // total bus recoveries performed
  volatile bool     healthy    = false;
  volatile uint32_t sample_us  = 0;   // micros() at the middle of the last good I2C read
  // AS5600 HYST acts only on ANGLE (0x0E), not on RAW ANGLE (0x0C). With ZPOS=MPOS=MANG=0
  // (never burned on this unit, checked by 'Q h') ANGLE = RAW + hysteresis.
  volatile uint8_t  angle_reg  = 0x0C;

  void init(TwoWire *w, int sda, int scl) {
    _wire = w; _sda = sda; _scl = scl;
    _wire->begin(_sda, _scl, 400000);
    _wire->setTimeOut(I2C_TIMEOUT_MS);     // <-- key: Wire can never block forever
    _last_rad = readAngleOnce(&_ok);
    Sensor::init();                         // sets angle_prev from getSensorAngle()
  }

  // SimpleFOC calls this every loop. Must be fast and non-blocking.
  float getSensorAngle() override {
    bool ok = false;
    uint32_t t0 = micros();
    float a = readAngleOnce(&ok);
    if (ok) {
      sample_us = t0 + (micros() - t0) / 2;   // velocity uses the real sample instant, not loop time
      _last_rad = a;
      err_streak = 0;
      healthy = true;
    } else {
      if (++err_streak >= I2C_ERRORS_TO_RECOVER) { recoverBus(); err_streak = 0; }
      healthy = false;
      // return last good angle so the control loop keeps running smoothly
    }
    return _last_rad;
  }

private:
  TwoWire *_wire = nullptr;
  int _sda = -1, _scl = -1;
  uint8_t _addr = 0x36;
  float _last_rad = 0.0f;
  bool _ok = false;

  // Read the 12-bit raw angle (registers 0x0C/0x0D). Returns radians [0,2PI).
  float readAngleOnce(bool *ok) {
    *ok = false;
    _wire->beginTransmission(_addr);
    _wire->write(angle_reg);                  // RAW ANGLE 0x0C, or ANGLE 0x0E when HYST is on
    if (_wire->endTransmission(false) != 0)   // repeated start, keep bus
      return _last_rad;
    if (_wire->requestFrom((int)_addr, 2) != 2)
      return _last_rad;
    int hi = _wire->read();
    int lo = _wire->read();
    if (hi < 0 || lo < 0) return _last_rad;
    uint16_t raw = ((uint16_t)hi << 8 | (uint16_t)lo) & 0x0FFF;
    *ok = true;
    return (float)raw * (_2PI / 4096.0f);
  }

  // Free a slave that is holding SDA low: clock SCL 9x, issue a STOP, re-init.
  void recoverBus() {
    recoveries++;
    _wire->end();
    pinMode(_sda, INPUT_PULLUP);
    pinMode(_scl, OUTPUT);
    for (int i = 0; i < 9; i++) {
      digitalWrite(_scl, LOW);  delayMicroseconds(5);
      digitalWrite(_scl, HIGH); delayMicroseconds(5);
    }
    // STOP: SDA low->high while SCL high
    pinMode(_sda, OUTPUT);
    digitalWrite(_sda, LOW);  delayMicroseconds(5);
    digitalWrite(_scl, HIGH); delayMicroseconds(5);
    digitalWrite(_sda, HIGH); delayMicroseconds(5);
    _wire->begin(_sda, _scl, 400000);
    _wire->setTimeOut(I2C_TIMEOUT_MS);
  }
};

// ----------------------------------------------------------------------------
// Velocity over an adaptive time window: (angle now - older angle) / dt, where the
// older sample is the first one that is >= VEL_FAST_COUNTS away (and >= VEL_MIN_US
// old) or VEL_WINDOW_US old. Returns exactly 0 for a still handle, and a one-count
// difference (sensor jitter) is removed by a soft one-count deadband on the delta.
// ----------------------------------------------------------------------------
class WindowVelocity {
public:
  void reset() { count_ = 0; head_ = 0; }
  float update(float angle, uint32_t nowUs) {
    angle_[head_] = angle; time_[head_] = nowUs;
    int newest = head_;
    head_ = (head_ + 1) % N;
    if (count_ < N) count_++;
    for (int k = 1; k < count_; k++) {
      int i = (newest - k + N) % N;
      uint32_t age = nowUs - time_[i];
      float d = angle - angle_[i];
      bool fast = age >= VEL_MIN_US && fabsf(d) >= VEL_FAST_COUNTS * AS5600_COUNT_RAD;
      if (fast || age >= VEL_WINDOW_US || k == count_ - 1) {
        if (age < VEL_MIN_US) return 0.0f;           // not enough history yet
        d = copysignf(fmaxf(fabsf(d) - 1.05f * AS5600_COUNT_RAD, 0.0f), d);  // 5%: float rounding of 1 count
        return d / (age * 1e-6f);
      }
    }
    return 0.0f;
  }
private:
  static const int N = 64;                            // >= window at 1 kHz loop
  float    angle_[N];
  uint32_t time_[N];
  int count_ = 0, head_ = 0;
};

// ----------------------------------------------------------------------------
// Damper velocity (v10): (angle - previous angle) / (their sample-time difference)
// -> RBJ 2nd-order low-pass designed for the ~1 kHz loop -> soft dead zone.
// A failed I2C read keeps sample_us, so dt == 0 holds the previous difference; a
// gap > 20 ms (calibration, stall) restarts the filter from rest.
// ----------------------------------------------------------------------------
class DampingVelocity {
public:
  void reset() { have_ = false; raw_ = 0.0f; z1_ = z2_ = 0.0f; }
  float update(float angle, uint32_t nowUs) {
    if (fc_ != g_velCutHz) design(g_velCutHz);
    if (!have_) {
      have_ = true; a_ = angle; t_ = nowUs; raw_ = 0.0f;
    } else {
      uint32_t dt = nowUs - t_;
      if (dt > 20000) { reset(); have_ = true; }
      else if (dt > 0) raw_ = (angle - a_) / (dt * 1e-6f);
      a_ = angle; t_ = nowUs;
    }
    float y = b0_ * raw_ + z1_;                        // transposed direct form II
    z1_ = b1_ * raw_ - a1_ * y + z2_;
    z2_ = b2_ * raw_ - a2_ * y;
    return y;                                          // dead zone is applied by the caller
  }
private:
  void design(float fc) {
    fc_ = fc;
    float w = 2.0f * PI * fc / 1000.0f, al = sinf(w) / (2.0f * VEL_LPF_Q), cw = cosf(w), a0 = 1.0f + al;
    b0_ = (1.0f - cw) / 2.0f / a0; b1_ = (1.0f - cw) / a0; b2_ = b0_;
    a1_ = -2.0f * cw / a0; a2_ = (1.0f - al) / a0;
  }
  bool have_ = false;
  float a_ = 0.0f, raw_ = 0.0f, fc_ = -1.0f;
  uint32_t t_ = 0;
  float b0_ = 0, b1_ = 0, b2_ = 0, a1_ = 0, a2_ = 0, z1_ = 0, z2_ = 0;
public:
  float raw() const { return raw_; }                   // last first difference, rad/s
};

// RBJ 2nd-order low-pass at the ~1 kHz loop rate (Q 0.707), for the complementary split.
class LowPass2 {
public:
  void reset() { z1_ = z2_ = 0.0f; }
  float operator()(float x, float fc) {
    if (fc != fc_) {
      fc_ = fc;
      float w = 2.0f * PI * fc / 1000.0f, al = sinf(w) / (2.0f * 0.7071f), cw = cosf(w), a0 = 1.0f + al;
      b0_ = (1.0f - cw) / 2.0f / a0; b1_ = (1.0f - cw) / a0; a1_ = -2.0f * cw / a0; a2_ = (1.0f - al) / a0;
    }
    float y = b0_ * x + z1_;
    z1_ = b1_ * x - a1_ * y + z2_;
    z2_ = b0_ * x - a2_ * y;
    return y;
  }
private:
  float fc_ = -1.0f, b0_ = 0, b1_ = 0, a1_ = 0, a2_ = 0, z1_ = 0, z2_ = 0;
};

// ----------------------------------------------------------------------------
// Damper velocity (v11): observer  theta' = w,  w' = OBS_G*U - OBS_C*w + d,  d' = 0
// with a triple pole at 2*pi*g_obsHz. feedU() gets the voltage actually put on the
// phases this pass (it drives the prediction until the next sample). A failed I2C
// read (sample_us unchanged) skips the pass; a gap > 20 ms restarts from rest.
// Python twin: diag-recordings/loop_sim.py observer().
// ----------------------------------------------------------------------------
class DampingObserver {
public:
  void reset() { have_ = false; w_ = d_ = u_ = 0.0f; }
  void feedU(float u) { u_ = u; }
  float update(float angle, uint32_t nowUs) {
    if (!have_) { have_ = true; th_ = angle; t_ = nowUs; w_ = d_ = 0.0f; return 0.0f; }
    uint32_t dtu = nowUs - t_;
    if (dtu == 0) return w_;
    t_ = nowUs;
    if (dtu > 20000) { th_ = angle; w_ = d_ = 0.0f; return 0.0f; }
    float dt = dtu * 1e-6f, p = 2.0f * PI * g_obsHz;
    float th = th_ + w_ * dt;
    float w  = w_ + (OBS_G * u_ - OBS_C * w_ + d_) * dt;
    float e  = angle - th;
    th_ = th + 3.0f * p * dt * e;
    w_  = w  + 3.0f * p * p * dt * e;
    d_  = d_ + p * p * p * dt * e;
    return w_;
  }
private:
  bool have_ = false;
  float th_ = 0.0f, w_ = 0.0f, d_ = 0.0f, u_ = 0.0f;
  uint32_t t_ = 0;
};

// ----------------------------------------------------------------------------
// SimpleFOC objects
// ----------------------------------------------------------------------------
BLDCMotor       motor  = BLDCMotor(MOTOR_POLE_PAIRS);
// Stock SimpleFOC 2.4 MCPWM backend (soak-proven in build-modern-final). The
// 28.09 legacy-MCPWM rewrite was dropped: it rested on the wrong "PWM vibrates at
// U=0" conclusion, and the vibration was the damper velocity (see WindowVelocity).
BLDCDriver3PWM  driver = BLDCDriver3PWM(PIN_PWM_A, PIN_PWM_B, PIN_PWM_C, PIN_EN);
AS5600Robust    sensor;
USBCDC          UsbSerial(0);   // manual CDC: Commander + diagnostics + 1200bps touch-reflash
Commander       command = Commander(UsbSerial);
Preferences     prefs;

[[noreturn]] static void holdBootDiagnostic(const char *stage) {
  uint32_t sent = 0, last = 0;
  for (;;) {
    if (tillerHID.send(false, 0x8000)) sent++;
    if (millis() - last >= 1000) {
      last = millis();
      UsbSerial.printf("BOOT-DIAG stage=%s uptime=%lu hid_sent=%lu psram=%lu\n",
                       stage, (unsigned long)millis(), (unsigned long)sent,
                       (unsigned long)ESP.getPsramSize());
    }
    delay(8);
  }
}

// Live tunables (force/damp set from knobs at boot; all tunable over Commander)
float g_forceV = 3.0f;    // max centering force (voltage_limit)
float g_dampK  = 0.2f;    // damping gain (V per rad/s)
float g_kp     = KP_SPRING;
volatile float g_returnBlend = 0.0f;
volatile bool  g_returning = false;
volatile float g_filteredVelocity = 0.0f;
volatile float g_springVoltage = 0.0f;
volatile float g_dampingVoltage = 0.0f;
volatile float g_motorVoltage = 0.0f;
// Quiet-center dead zone (return_profile.h). Off by default: it was added against
// the damper vibration and left the handle resting ~2-3 deg off center. Its 'Z'
// command was dropped 29.09 for 'Y' (Commander table is full at 20).
volatile bool  g_quietCenter = false;

// 'R' recorder: motorTask captures REC_N consecutive control passes (time, angle,
// velocity, command), loop() then dumps them. Proof for loop dynamics instead of
// sparse 'S' snapshots.
static const int REC_N = 1500;
struct RecSample { uint32_t t; float a, v, u; };
RecSample g_rec[REC_N];
volatile int g_recState = 0;   // 0 idle, 1 recording, 2 ready to dump
volatile int g_recLen = 0;

// 'J' stand test: shift the spring center by g_kickRad for KICK_MS while recording,
// so a Kd sweep measures the real stability limit with the handle released. If the
// response keeps oscillating (KICK_ABORT_FLIPS big sign flips of U after the kick),
// motorTask drops the damper to 0 at once instead of rattling the whole recording.
// 'J<rad>' sets the size: 0.05 (3 deg) barely beats the friction; 0.3..0.6 pulls the
// handle 17..34 deg away and the return is a hard release, where v7 oscillated.
static const float    KICK_RAD_DEFAULT = 0.05f;   // ~3 deg
static const uint32_t KICK_MS          = 400;
volatile float g_kickRad = KICK_RAD_DEFAULT;
static const float    KICK_FLIP_V      = 1.5f;
static const int      KICK_ABORT_FLIPS = 8;
volatile uint32_t g_kickStart = 0;     // millis() of the kick, 0 = no stand test running
volatile bool     g_kickAborted = false;

// 'Q' AS5600 CONF experiments (volatile in the chip): SF slow filter (0x07 bits 1:0;
// 0=16x 2.2 ms .. 3=2x 0.29 ms), FTH fast-filter threshold (0x07 bits 4:2; 0 = off),
// HYST (0x08 bits 3:2; 0 = off, 1..3 LSB, acts on ANGLE 0x0E only). An ESP32 reset does
// NOT power-cycle the sensor, so motorTask restores all three to 0 at start. Serviced
// by motorTask (it owns I2C). Request = (field << 8) | value; field 0 = read only.
enum { CONF_READ = 0, CONF_SF = 1, CONF_FTH = 2, CONF_HYST = 3 };
volatile int g_confRequest = -1;
volatile int g_confResult  = -2;       // (CONF_H << 8) | CONF_L; -1 I2C error, -3 ZPOS/MPOS/MANG set

// Mechanical range calibration (persisted)
struct Calib {
  uint32_t magic;
  float    angle_min;
  float    angle_max;
  float    angle_center;
};
Calib g_cal = { CALIB_MAGIC, -0.6f, 0.6f, 0.0f };  // safe default until calibrated
bool  g_calibrated = false;

// Shared state between tasks
volatile uint16_t g_axis   = 0x8000;   // last computed HID axis
volatile bool     g_button = false;
volatile bool     g_calib_request = false;   // 'C' manual sweep; serviced in motorTask
volatile bool     g_autocal_request = false; // 'A' / startup: home to stops; serviced in motorTask
volatile bool     g_stream = false;          // 'H' toggles a periodic status line
bool              g_button_ok = true;         // false if the (broken) button reads stuck at boot
volatile bool     g_calibrating = false;      // true while auto/manual cal drives the motor (pause live knobs)
bool              g_live_knobs = true;         // read pots continuously ('L' toggles; off = serial F/D control)
int               g_lastRawF = 0, g_lastRawD = 0;  // last raw knob ADC (diagnostics)

// The old task watchdog could not see a live hidTask whose USB sends all failed.
// Keep subsystem heartbeats and a rolling crash snapshot independently instead.
enum DiagFault : uint32_t {
  DIAG_NONE = 0,
  DIAG_MOTOR_STALL = 1,
  DIAG_HID_TASK_STALL = 2,
  DIAG_ARDUINO_LOOP_STALL = 3,
  DIAG_USB_REPORT_STALL = 4,
  DIAG_USB_RECOVERY_FAILED = 5,
};

struct DiagRecord {
  uint32_t magic, version, fault, uptime_ms;
  uint32_t motor_hb, hid_hb, loop_hb, hid_ok, hid_fail;
  uint32_t hid_not_ready, hid_send_fail;
  uint32_t i2c_recoveries, i2c_error_streak;
  uint32_t usb_started, usb_suspended, reset_reason;
  uint32_t usb_recoveries;
  uint32_t gintsts, gintmsk, gahbcfg, gusbcfg;
  uint32_t dcfg, dctl, dsts, daint, daintmsk, diepmsk;
  uint32_t ep_ctl[USB_IN_EP_NUM];
  uint32_t ep_int[USB_IN_EP_NUM];
  uint32_t ep_size[USB_IN_EP_NUM];
  uint32_t ep_fifo[USB_IN_EP_NUM];
  uint16_t axis, reserved;
};

static const uint32_t DIAG_MAGIC = 0x54494C52; // "TILR"
static const uint32_t DIAG_VERSION = 5;
RTC_NOINIT_ATTR DiagRecord g_rtcDiag;
DiagRecord g_bootDiag;
bool g_bootDiagValid = false;
volatile uint32_t g_motorHb = 0, g_hidHb = 0, g_loopHb = 0;
volatile uint32_t g_hidOk = 0, g_hidFail = 0, g_lastHidOkMs = 0;
volatile bool g_usbStarted = false, g_usbSuspended = false;
volatile bool g_usbRecoveryRequest = false;
volatile uint32_t g_usbRecoveries = 0;
esp_err_t g_wdtInitResult = ESP_FAIL;
esp_err_t g_motorWdtResult = ESP_FAIL;
esp_err_t g_hidWdtResult = ESP_FAIL;

static const char *diagFaultName(uint32_t fault) {
  switch (fault) {
    case DIAG_MOTOR_STALL: return "motor-task-stall";
    case DIAG_HID_TASK_STALL: return "hid-task-stall";
    case DIAG_ARDUINO_LOOP_STALL: return "arduino-loop-stall";
    case DIAG_USB_REPORT_STALL: return "usb-report-stall";
    case DIAG_USB_RECOVERY_FAILED: return "usb-recovery-failed";
    default: return "none";
  }
}

static void fillDiagRecord(DiagRecord &d, DiagFault fault) {
  d.magic = DIAG_MAGIC; d.version = DIAG_VERSION; d.fault = (uint32_t)fault;
  d.uptime_ms = millis(); d.motor_hb = g_motorHb; d.hid_hb = g_hidHb;
  d.loop_hb = g_loopHb; d.hid_ok = g_hidOk; d.hid_fail = g_hidFail;
  d.hid_not_ready = g_hidNotReady; d.hid_send_fail = g_hidSendFail;
  d.i2c_recoveries = sensor.recoveries; d.i2c_error_streak = sensor.err_streak;
  d.usb_started = g_usbStarted ? 1 : 0; d.usb_suspended = g_usbSuspended ? 1 : 0;
  d.usb_recoveries = g_usbRecoveries;
  d.reset_reason = (uint32_t)esp_reset_reason();
  d.gintsts = USB0.gintsts; d.gintmsk = USB0.gintmsk;
  d.gahbcfg = USB0.gahbcfg; d.gusbcfg = USB0.gusbcfg;
  d.dcfg = USB0.dcfg; d.dctl = USB0.dctl; d.dsts = USB0.dsts;
  d.daint = USB0.daint; d.daintmsk = USB0.daintmsk; d.diepmsk = USB0.diepmsk;
  for (int i = 0; i < USB_IN_EP_NUM; i++) {
    d.ep_ctl[i] = USB0.in_ep_reg[i].diepctl;
    d.ep_int[i] = USB0.in_ep_reg[i].diepint;
    d.ep_size[i] = USB0.in_ep_reg[i].dieptsiz;
    d.ep_fifo[i] = USB0.in_ep_reg[i].dtxfsts;
  }
  d.axis = g_axis; d.reserved = 0;
}

static void persistFault(DiagFault fault) {
  fillDiagRecord(g_rtcDiag, fault);
  Preferences diagPrefs;
  if (diagPrefs.begin("tiller-diag", false)) {
    diagPrefs.putBytes("last", &g_rtcDiag, sizeof(g_rtcDiag));
    diagPrefs.end();
  }
}

static void saveFaultAndRestart(DiagFault fault) {
  persistFault(fault);
  motor.move(0);
  delay(30);
  esp_restart();
}

static bool loadSavedDiag(DiagRecord &d) {
  Preferences diagPrefs;
  if (diagPrefs.begin("tiller-diag", true)) {
    bool ok = diagPrefs.getBytesLength("last") == sizeof(d) &&
              diagPrefs.getBytes("last", &d, sizeof(d)) == sizeof(d);
    diagPrefs.end();
    if (ok && d.magic == DIAG_MAGIC && d.version == DIAG_VERSION) return true;
  }
  if (g_bootDiagValid) { d = g_bootDiag; return true; }
  return false;
}

static void printDiagRecord(Stream &out, const DiagRecord &d) {
  out.printf("diag fault=%s(%lu) uptime=%lums reset=%lu | motor=%lu hid=%lu loop=%lu "
             "hid_ok=%lu hid_fail=%lu not_ready=%lu send_fail=%lu | usb_started=%lu suspended=%lu usb_recov=%lu | "
             "i2c_streak=%lu recov=%lu axis=%u\n",
             diagFaultName(d.fault), (unsigned long)d.fault, (unsigned long)d.uptime_ms,
             (unsigned long)d.reset_reason, (unsigned long)d.motor_hb,
             (unsigned long)d.hid_hb, (unsigned long)d.loop_hb,
             (unsigned long)d.hid_ok, (unsigned long)d.hid_fail,
             (unsigned long)d.hid_not_ready, (unsigned long)d.hid_send_fail,
             (unsigned long)d.usb_started, (unsigned long)d.usb_suspended,
             (unsigned long)d.usb_recoveries,
             (unsigned long)d.i2c_error_streak, (unsigned long)d.i2c_recoveries,
             (unsigned)d.axis);
  out.printf("usbreg GINTSTS=%08lX GINTMSK=%08lX GAHBCFG=%08lX GUSBCFG=%08lX "
             "DCFG=%08lX DCTL=%08lX DSTS=%08lX DAINT=%08lX DAINTMSK=%08lX DIEPMSK=%08lX\n",
             (unsigned long)d.gintsts, (unsigned long)d.gintmsk,
             (unsigned long)d.gahbcfg, (unsigned long)d.gusbcfg,
             (unsigned long)d.dcfg, (unsigned long)d.dctl, (unsigned long)d.dsts,
             (unsigned long)d.daint, (unsigned long)d.daintmsk, (unsigned long)d.diepmsk);
  for (int i = 0; i < USB_IN_EP_NUM; i++) {
    if (d.ep_ctl[i] || d.ep_int[i] || d.ep_size[i] || d.ep_fifo[i])
      out.printf("usbreg INEP%d CTL=%08lX INT=%08lX SIZE=%08lX FIFO=%08lX\n", i,
                 (unsigned long)d.ep_ctl[i], (unsigned long)d.ep_int[i],
                 (unsigned long)d.ep_size[i], (unsigned long)d.ep_fifo[i]);
  }
}

static void usbEvent(void *, esp_event_base_t, int32_t eventId, void *) {
  if (eventId == ARDUINO_USB_STARTED_EVENT) {
    g_usbStarted = true; g_usbSuspended = false;
  } else if (eventId == ARDUINO_USB_STOPPED_EVENT) {
    g_usbStarted = false; g_usbSuspended = false;
  } else if (eventId == ARDUINO_USB_SUSPEND_EVENT) {
    g_usbSuspended = true;
  } else if (eventId == ARDUINO_USB_RESUME_EVENT) {
    g_usbSuspended = false;
  }
}

// ----------------------------------------------------------------------------
// Calibration persistence
// ----------------------------------------------------------------------------
void loadCalib() {
  prefs.begin("tiller", true);
  size_t n = prefs.getBytesLength("cal");
  if (n == sizeof(Calib)) {
    Calib tmp;
    prefs.getBytes("cal", &tmp, sizeof(tmp));
    if (tmp.magic == CALIB_MAGIC && tmp.angle_max > tmp.angle_min) {
      g_cal = tmp;
      g_calibrated = true;
    }
  }
  prefs.end();
}

void saveCalib() {
  prefs.begin("tiller", false);
  prefs.putBytes("cal", &g_cal, sizeof(g_cal));
  prefs.end();
}

// ----------------------------------------------------------------------------
// Map a physical shaft angle to the 16-bit HID axis over the calibrated range.
// ----------------------------------------------------------------------------
uint16_t angleToAxis(float a) {
  float span = g_cal.angle_max - g_cal.angle_min;
  if (span < 1e-3f) span = 1e-3f;
  float t = (a - g_cal.angle_min) / span;      // 0..1
  if (t < 0) t = 0; else if (t > 1) t = 1;
  return (uint16_t)(t * 65535.0f + 0.5f);
}

// ----------------------------------------------------------------------------
// Mechanical range calibration: user sweeps the tiller lock-to-lock while we
// record min/max. Centering is disabled during the sweep. ~6 seconds.
// ----------------------------------------------------------------------------
void calibrateRange() {
  UsbSerial.println("[MechCal] Sweep the tiller fully both ways for 6 s...");
  g_calibrating = true;                         // pause live-knob voltage_limit updates
  float lo =  1e9f, hi = -1e9f;
  float saved_limit = motor.voltage_limit;
  motor.voltage_limit = 0;                      // let the user move it freely
  uint32_t t0 = millis();
  while (millis() - t0 < 6000) {
    motor.loopFOC();
    motor.move(0);
    float a = motor.shaft_angle;
    if (a < lo) lo = a;
    if (a > hi) hi = a;
    esp_task_wdt_reset();
    delay(1);
  }
  motor.voltage_limit = saved_limit;
  if (hi - lo > 0.05f) {
    g_cal.magic = CALIB_MAGIC;
    g_cal.angle_min = lo;
    g_cal.angle_max = hi;
    g_cal.angle_center = 0.5f * (lo + hi);
    saveCalib();
    g_calibrated = true;
    UsbSerial.printf("[MechCal] Saved: min=%.3f max=%.3f center=%.3f\n",
                  g_cal.angle_min, g_cal.angle_max, g_cal.angle_center);
  } else {
    UsbSerial.println("[MechCal] Range too small, keeping previous calibration.");
  }
  g_calibrating = false;                          // resume live-knob updates
}

// ----------------------------------------------------------------------------
// Auto-calibration (MOZA-style startup homing): drive to each mechanical stop,
// take the two extremes as the axis range and the midpoint as center.
// Runs ONLY inside motorTask so nothing else touches the motor concurrently.
// ----------------------------------------------------------------------------
// Mechanical angle from the sample loopFOC() just took (same frame as shaft_angle).
static float mechAngle() {
  return motor.sensor_direction * sensor.getAngle() - motor.sensor_offset;
}

static float driveToStop(float v) {
  uint32_t t0 = millis(), stall_since = 0;
  bool moved = false;
  static WindowVelocity velocity;                    // SimpleFOC 2.4 shaft_velocity never decays at a stop
  velocity.reset();                                  // static: keep 0.5 KB off the 4 KB motor stack
  while (millis() - t0 < 4000) {
    motor.loopFOC();
    motor.move(v);                                   // constant push toward the stop
    float vel = fabsf(velocity.update(mechAngle(), sensor.sample_us));
    if (vel > CAL_MOVE_VEL) moved = true;
    if (moved) {
      if (vel < CAL_STALL_VEL) {                     // stalled = pressed against the stop
        if (!stall_since) stall_since = millis();
        else if (millis() - stall_since > 350) break;
      } else stall_since = 0;
    } else if (millis() - t0 > 1500) {
      break;                                         // never moved -> no motor power; give up fast
    }
    esp_task_wdt_reset();
    delay(2);
  }
  motor.move(0);
  for (int i = 0; i < 40; i++) { motor.loopFOC(); motor.move(0); delay(1); }  // settle
  return motor.shaft_angle;
}

void autoCalibrate() {
  UsbSerial.println("[AutoCal] homing to mechanical stops...");
  g_calibrating = true;                         // pause live-knob voltage_limit updates
  float saved = motor.voltage_limit;
  motor.voltage_limit = CAL_PROBE_V;
  float a = driveToStop(+CAL_PROBE_V);
  float b = driveToStop(-CAL_PROBE_V);
  motor.voltage_limit = saved;
  float lo = (a < b) ? a : b, hi = (a < b) ? b : a;   // sensor direction independent
  if (hi - lo > 0.10f) {
    g_cal.magic = CALIB_MAGIC;
    g_cal.angle_min = lo;
    g_cal.angle_max = hi;
    g_cal.angle_center = 0.5f * (lo + hi);
    g_calibrated = true;
    saveCalib();
    UsbSerial.printf("[AutoCal] min=%.3f max=%.3f center=%.3f\n", lo, hi, g_cal.angle_center);
  } else {
    UsbSerial.println("[AutoCal] no travel detected (motor power off?) - keeping previous/provisional.");
  }
  // ease back to center at probe strength so the stick starts centered
  uint32_t t0 = millis();
  while (millis() - t0 < 1500) {
    motor.loopFOC();
    float u = 3.0f * (g_cal.angle_center - motor.shaft_angle); // preserve startup return slope on 12 V PSU
    if (u >  CAL_PROBE_V) u =  CAL_PROBE_V;
    if (u < -CAL_PROBE_V) u = -CAL_PROBE_V;
    motor.move(u);
    esp_task_wdt_reset();
    delay(2);
  }
  motor.move(0);
  g_calibrating = false;                         // resume live-knob updates
}

// ----------------------------------------------------------------------------
// Commander handlers (live tuning + diagnostics over USB serial)
// ----------------------------------------------------------------------------
void cmdMotor(char *c)  { command.motor(&motor, c); }
void cmdForce(char *c)  { command.scalar(&g_forceV, c); g_forceV = constrain(g_forceV, 0.0f, V_LINEAR_MAX); motor.voltage_limit = g_forceV; }
void cmdDamp(char *c)   { command.scalar(&g_dampK, c); }
void cmdKp(char *c)     { command.scalar(&g_kp, c); }
void cmdCalib(char *c)  { (void)c; g_calib_request = true; }    // manual sweep (serviced by motorTask)
void cmdAuto(char *c)   { (void)c; g_autocal_request = true; }  // MOZA-style home-to-stops
void cmdStream(char *c) { (void)c; g_stream = !g_stream; }
static int as5600_rd(uint8_t reg, int nbytes) {   // raw AS5600 register read for diagnostics
  Wire.beginTransmission(0x36); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom((int)0x36, nbytes) != nbytes) return -1;
  int v = 0; for (int i = 0; i < nbytes; i++) v = (v << 8) | (Wire.read() & 0xFF);
  return v;
}
// Only for the volatile CONF register 0x07/0x08. NEVER write 0xFF (BURN): it is
// one-time and permanently programs the chip.
static bool as5600_wr_conf(uint8_t reg, uint8_t v) {
  if (reg != 0x07 && reg != 0x08) return false;
  Wire.beginTransmission(0x36); Wire.write(reg); Wire.write(v);
  return Wire.endTransmission() == 0;
}
// motorTask side of 'Q': read-modify-write one CONF field, return (CONF_H << 8) | CONF_L.
static int serviceConf(int req) {
  int field = req >> 8, val = req & 0xFF;
  int h = as5600_rd(0x07, 1), l = as5600_rd(0x08, 1);
  if (h < 0 || l < 0) return -1;
  bool ok = true;
  if (field == CONF_SF)   ok = as5600_wr_conf(0x07, (uint8_t)((h & ~0x03) | (val & 3)));
  if (field == CONF_FTH)  ok = as5600_wr_conf(0x07, (uint8_t)((h & ~0x1C) | ((val & 7) << 2)));
  if (field == CONF_HYST) {
    int zm = as5600_rd(0x01, 2), mp = as5600_rd(0x03, 2), mg = as5600_rd(0x05, 2);
    if (zm < 0 || mp < 0 || mg < 0) return -1;
    if (((zm | mp | mg) & 0x0FFF) != 0) return -3;          // ANGLE would be rescaled
    ok = as5600_wr_conf(0x08, (uint8_t)((l & ~0x0C) | ((val & 3) << 2)));
    if (ok) sensor.angle_reg = (val & 3) ? 0x0E : 0x0C;
  }
  if (!ok) return -1;
  h = as5600_rd(0x07, 1); l = as5600_rd(0x08, 1);
  if (h < 0 || l < 0) return -1;
  return (h << 8) | l;
}
void cmdSensor(char *c) {   // AS5600 magnet diagnostics: MD/ML/MH + field strength
  (void)c;
  int st = as5600_rd(0x0B, 1), raw = as5600_rd(0x0C, 2), agc = as5600_rd(0x1A, 1), mag = as5600_rd(0x1B, 2);
  UsbSerial.printf("[AS5600] STATUS=0x%02X  MD=%d ML=%d MH=%d | RAW=%d AGC=%d MAGNITUDE=%d\n",
    st, (st < 0 ? -1 : (st >> 5) & 1), (st < 0 ? -1 : (st >> 4) & 1), (st < 0 ? -1 : (st >> 3) & 1),
    raw, agc, mag);
}
void cmdVelDead(char *c) {     // 'N': damper velocity dead zone, rad/s (0..2)
  float x = g_velDeadRad; command.scalar(&x, c);
  g_velDeadRad = constrain(x, 0.0f, 2.0f);
}
void cmdVelFilter(char *c) {   // 'W': observer bandwidth 5..40 Hz, or biquad cutoff 20..300 Hz ('O' mode)
  if (g_ctrlMode != 1) {
    float x = g_obsHz; command.scalar(&x, c);
    g_obsHz = constrain(x, 5.0f, 40.0f);   // explicit-Euler observer: keep p*dt small at dt up to 1.9 ms
  } else {
    float x = g_velCutHz; command.scalar(&x, c);
    g_velCutHz = constrain(x, 20.0f, 300.0f);
  }
}
void cmdRecord(char *c) { (void)c; if (g_recState == 0) { g_recLen = 0; g_recState = 1; } }
void cmdKick(char *c) {        // 'J[rad]': stand test = kick the center + record the response
  if (g_live_knobs) { UsbSerial.println("kick: turn live knobs off first (L), then set D/P/F"); return; }
  if (g_recState != 0 || g_kickStart) { UsbSerial.println("kick: busy"); return; }
  while (c && (*c == ' ' || *c == '\r' || *c == '\n')) c++;   // terminals send "J\r"
  float r = (c && *c) ? (float)atof(c) : KICK_RAD_DEFAULT;
  g_kickRad = constrain(r, -0.6f, 0.6f);          // stays well inside the +-1.6 rad travel
  g_kickAborted = false;
  g_recLen = 0;
  g_kickStart = millis() | 1;   // never 0
  g_recState = 1;
}
// 'Q' reads CONF; 'Qs<0..3>' SF, 'Qf<0..7>' FTH, 'Qh<0..3>' HYST (switches to ANGLE 0x0E).
void cmdSensorFilter(char *c) {
  int field = CONF_READ, val = 0;
  while (c && (*c == ' ' || *c == '\r' || *c == '\n')) c++;   // terminals send "Q\r"
  if (c && *c) {
    char k = *c;
    val = atoi(c + 1);
    if (k == 's' && val >= 0 && val <= 3) field = CONF_SF;
    else if (k == 'f' && val >= 0 && val <= 7) field = CONF_FTH;
    else if (k == 'h' && val >= 0 && val <= 3) field = CONF_HYST;
    else { UsbSerial.println("Q | Qs0..3 (SF 16x..2x) | Qf0..7 (FTH, 0 off) | Qh0..3 (HYST LSB)"); return; }
  }
  g_confResult = -2;
  g_confRequest = (field << 8) | val;
  for (int i = 0; i < 50 && g_confRequest >= 0; i++) delay(1);
  int r = g_confResult;
  if (r < 0) {
    UsbSerial.printf("AS5600 CONF: %s\n", r == -1 ? "I2C error" : r == -3 ? "ZPOS/MPOS/MANG set, HYST refused" : "no answer");
    return;
  }
  int h = r >> 8, l = r & 0xFF;
  UsbSerial.printf("AS5600 CONF_H=0x%02X CONF_L=0x%02X SF=%d FTH=%d HYST=%d WD=%d | reading %s\n",
                   h, l, h & 3, (h >> 2) & 7, (l >> 2) & 3, (h >> 5) & 1,
                   sensor.angle_reg == 0x0E ? "ANGLE 0x0E" : "RAW 0x0C");
}
void cmdVelMode(char *c) {     // 'O<n>': 0 ours (observer), 1 ours (v10c biquad), 2 factory law
  while (c && (*c == ' ' || *c == '\r' || *c == '\n')) c++;
  if (c && *c == 's') {        // 'Os<n>': idle sleep after n seconds still, 0 = never (the table is full)
    if (c[1] && c[1] != '\r' && c[1] != '\n') g_idleSleepS = (uint32_t)constrain(atoi(c + 1), 0, 3600);
    UsbSerial.printf("idle sleep=%lus asleep=%d still=%.1fs\n", (unsigned long)g_idleSleepS,
                     (int)g_asleep, g_idleForMs * 0.001f);
    return;
  }
  if (c && *c) {
    int m = atoi(c);
    if (m < 0 || m > 2) { UsbSerial.println("O0 ours | O1 ours+biquad | O2 factory"); return; }
    g_ctrlMode = m;
    g_velDeadRad = m == 1 ? 0.3f : 0.1f;
    if (!g_calibrating) motor.voltage_limit = m == 2 ? fminf(STOCK_V_LIMIT, V_LINEAR_MAX) : g_forceV;
  }
  UsbSerial.printf("mode=%d (%s) dead=%.2f vlim=%.0f Kv=%.2f\n", (int)g_ctrlMode,
                   g_ctrlMode == 2 ? "factory" : g_ctrlMode == 1 ? "biquad" : "observer",
                   g_velDeadRad, g_stockVlim, g_stockKv);
}
void cmdSplit(char *c) {       // 'Y': complementary split Hz (0 = pure observer, max 30)
  float x = g_splitHz; command.scalar(&x, c);
  g_splitHz = constrain(x, 0.0f, 30.0f);
}
void cmdLive(char *c)  { (void)c; g_live_knobs = !g_live_knobs;
                         UsbSerial.printf("live knobs=%d\n", (int)g_live_knobs); }
void cmdKnobs(char *c) {   // raw ADC of candidate knob pins (turn each knob, see which changes)
  (void)c;
  UsbSerial.printf("[ADC] G1=%d G2=%d G3=%d G8=%d G9=%d G10=%d | live=%d force=%.2fV damp=%.3f\n",
    analogRead(1), analogRead(2), analogRead(3), analogRead(8), analogRead(9), analogRead(10),
    (int)g_live_knobs, g_forceV, g_dampK);
}
void cmdStatus(char *c) {
  (void)c;
  UsbSerial.printf("angle=%.3f vel=%.2f filtV=%.3f axis=%u dir=%d | force=%.2fV damp=%.3f kp=%.1f "
                "U=%.3f spring=%.3f damping=%.3f return=%s/%.2f rawF=%d rawD=%d | "
                "min=%.3f center=%.3f max=%.3f cal=%d | sensor_ok=%d recov=%lu | "
                "asleep=%d still=%.1fs sleep_after=%lus\n",
                motor.shaft_angle, motor.shaft_velocity, g_filteredVelocity,
                (unsigned)g_axis, (int)motor.sensor_direction,
                g_forceV, g_dampK, g_kp, g_motorVoltage, g_springVoltage, g_dampingVoltage,
                g_returning ? "idle" : "active", g_returnBlend,
                g_lastRawF, g_lastRawD,
                g_cal.angle_min, g_cal.angle_center, g_cal.angle_max, (int)g_calibrated,
                (int)sensor.healthy, (unsigned long)sensor.recoveries,
                (int)g_asleep, g_idleForMs * 0.001f, (unsigned long)g_idleSleepS);
}
void cmdDiag(char *c) {
  (void)c;
  DiagRecord d;
  if (g_bootDiagValid) {
    UsbSerial.println("diag snapshot before this boot:");
    printDiagRecord(UsbSerial, g_bootDiag);
  }
  if (loadSavedDiag(d)) {
    UsbSerial.println("diag last persistent fault:");
    printDiagRecord(UsbSerial, d);
  } else if (!g_bootDiagValid) UsbSerial.println("diag: no saved record");
  UsbSerial.printf("diag live motor=%lu hid=%lu loop=%lu hid_ok=%lu hid_fail=%lu not_ready=%lu send_fail=%lu "
                   "usb_started=%d suspended=%d usb_recov=%lu last_hid_ok_age=%lums wdt=%d/%d/%d\n",
                   (unsigned long)g_motorHb, (unsigned long)g_hidHb,
                   (unsigned long)g_loopHb, (unsigned long)g_hidOk,
                   (unsigned long)g_hidFail, (unsigned long)g_hidNotReady,
                   (unsigned long)g_hidSendFail, (int)g_usbStarted,
                   (int)g_usbSuspended, (unsigned long)g_usbRecoveries,
                   (unsigned long)(millis() - g_lastHidOkMs),
                   (int)g_wdtInitResult, (int)g_motorWdtResult, (int)g_hidWdtResult);
}
void cmdUsbRecover(char *c) {
  (void)c;
  UsbSerial.println("USB local recovery requested");
  g_usbRecoveryRequest = true;
}

// ----------------------------------------------------------------------------
// Tasks
// ----------------------------------------------------------------------------
// motor_task: FOC + self-centering spring/damper. Owns the sensor and motor.
void motorTask(void *arg) {
  g_motorWdtResult = esp_task_wdt_add(NULL);
  autoCalibrate();                 // MOZA-style startup homing to the stops (every boot)
  static DampingVelocity dampingBiquad;
  static DampingObserver dampingVelocity;
  static LowPass2 splitLpf;
  g_confRequest = (CONF_SF << 8) | 0;  // the sensor survives an ESP32 reset: restore
  g_confResult = serviceConf(g_confRequest);                 // power-on SF/FTH/HYST
  if (g_confResult >= 0) g_confResult = serviceConf((CONF_FTH << 8) | 0);
  if (g_confResult >= 0) g_confResult = serviceConf((CONF_HYST << 8) | 0);
  g_confRequest = -1;
  TillerReturnProfile returnProfile;
  returnProfile.reset(millis());
  int kickFlips = 0; float kickLastU = 0.0f;
  bool stInit = false; float stTh = 0.0f, stW = 0.0f, stU = 0.0f; uint32_t stT = 0;   // factory law state
  IdleSleep idle;
  idle.reset(mechAngle(), millis());
  for (;;) {
    // Calibration drives the motor itself: wake the driver first.
    if ((g_autocal_request || g_calib_request) && idle.asleep) {
      idle.asleep = false; idle.wakeMs = 0; motor.enable(); g_asleep = false;
    }
    // Calibration requests are serviced HERE so only this task drives the motor.
    if (g_autocal_request) {
      g_autocal_request = false; autoCalibrate();
      dampingVelocity.reset(); dampingBiquad.reset(); splitLpf.reset(); stInit = false;
      returnProfile.reset(millis());
    }
    if (g_calib_request) {
      g_calib_request = false; calibrateRange();
      dampingVelocity.reset(); dampingBiquad.reset(); splitLpf.reset(); stInit = false;
      returnProfile.reset(millis());
    }
    if (g_confRequest >= 0) {                // 'Q': this task owns the I2C bus
      g_confResult = serviceConf(g_confRequest);
      g_confRequest = -1;
    }

    // SimpleFOC 2.4 updates the sensor inside loopFOC(). Take exactly one AS5600
    // sample per control pass. An extra sensor.update() here used to create
    // near-zero-dt velocity spikes.
    motor.loopFOC();
    uint32_t nowUs = sensor.sample_us;
    float a = mechAngle();
    float vObs = dampingVelocity.update(a, nowUs);         // both run: 'O' switches without a jump
    float vBiq = dampingBiquad.update(a, nowUs);
    float split = g_splitHz;
    float vComp = split > 0.5f ? vObs + splitLpf(dampingBiquad.raw() - vObs, split) : vObs;
    float vf = g_ctrlMode == 1 ? vBiq : vComp;
    float v = copysignf(fmaxf(fabsf(vf) - g_velDeadRad, 0.0f), vf);

    float center = g_cal.angle_center;
    uint32_t kick = g_kickStart;
    if (kick && millis() - kick < KICK_MS) center += g_kickRad;

    // Factory-law filters run every pass, so switching modes never jumps.
    float dtPass = 0.001f;
    if (!stInit || nowUs - stT > 20000) { stTh = a; stW = 0.0f; stT = nowUs; stInit = true; }
    else if (nowUs != stT) {
      dtPass = (nowUs - stT) * 1e-6f; stT = nowUs;
      float ka = STOCK_TF_ANGLE / (STOCK_TF_ANGLE + dtPass), kv = STOCK_TF_VEL / (STOCK_TF_VEL + dtPass);
      stTh = ka * stTh + (1.0f - ka) * a;
      stW  = kv * stW  + (1.0f - kv) * dampingBiquad.raw();
    }

    float error = center - a;
    TillerReturnProfile::Output profile = returnProfile.update(millis(), error, v);
    float springVoltage, dampingVoltage, u;
    if (g_ctrlMode == 2) {
      float velSp = _constrain(STOCK_P_ANGLE * (center - stTh), -g_stockVlim, g_stockVlim);
      springVoltage = g_stockKv * velSp;
      dampingVoltage = g_stockKv * stW;
      float step = STOCK_RAMP_V_S * dtPass;                  // SimpleFOC PID output ramp
      u = _constrain(springVoltage - dampingVoltage, stU - step, stU + step);
      v = stW;
    } else {
      float springError = g_quietCenter ? profile.springError : error;
      springVoltage = g_kp * profile.springScale * springError;
      dampingVoltage = g_dampK * profile.dampingScale * v;
      u = springVoltage - dampingVoltage;
    }
    uint32_t nowMs = millis();
    int sleepEdge = idle.update(a, center, nowMs, g_idleSleepS, kick != 0);
    if (sleepEdge > 0) motor.disable();          // EN low: phases high-Z, silent
    else if (sleepEdge < 0) motor.enable();
    g_asleep = idle.asleep;
    g_idleForMs = idle.asleep ? g_idleSleepS * 1000UL : nowMs - idle.stillMs;
    if (idle.asleep) u = 0.0f;
    float cap = motor.voltage_limit * idle.capScale(nowMs);   // soft start after a wake
    u = _constrain(u, -cap, cap);
    stU = u;
    g_returnBlend = profile.returnBlend;
    g_returning = profile.returning;
    g_filteredVelocity = v;
    g_springVoltage = springVoltage;
    g_dampingVoltage = dampingVoltage;
    g_motorVoltage = u;
    // (future FFB: replace/augment `u` here with a force streamed from the PC)
    motor.move(u);
    // Put u on the phases NOW. loopFOC() applies current_sp only after its own sensor
    // read, i.e. one whole pass (~1 ms) late; that delay was the largest single term
    // in the damper's phase lag (sim: stable Kd 1.6 -> 2.4). The next loopFOC()
    // re-applies the same u with a fresh electrical angle. Same clamp as loopFOC().
    float uApplied = motor.enabled ? _constrain(u, -motor.voltage_limit, motor.voltage_limit) : 0.0f;
    if (motor.enabled)
      motor.setPhaseVoltage(uApplied, 0, motor.electrical_angle);
    dampingVelocity.feedU(uApplied);          // drives the observer's prediction to the next sample

    if (kick) {                               // 'J' stand: abort a sustained oscillation
      if (millis() - kick >= KICK_MS && fabsf(u) > KICK_FLIP_V) {
        if (u * kickLastU < 0 && ++kickFlips >= KICK_ABORT_FLIPS && !g_kickAborted) {
          g_dampK = 0.0f;
          if (g_ctrlMode == 2) g_ctrlMode = 0;   // factory law has no separate damper: fall back to spring only
          g_kickAborted = true;
        }
        kickLastU = u;
      }
    } else {
      kickFlips = 0; kickLastU = 0.0f;
    }

    if (g_recState == 1) {
      int n = g_recLen;
      g_rec[n] = { nowUs, a, v, u };
      g_recLen = n + 1;
      if (n + 1 >= REC_N) { g_recState = 2; g_kickStart = 0; }
    }

    g_axis = angleToAxis(a);
    g_motorHb++;

    esp_task_wdt_reset();
    // small yield keeps the RTOS scheduler + USB stack breathing
    vTaskDelay(1);
  }
}

// hid_task: button debounce (short=HID button, long=recalibrate) + ship report.
void hidTask(void *arg) {
  g_hidWdtResult = esp_task_wdt_add(NULL);
  bool     prev_raw = false;
  uint32_t press_t0 = 0;
  bool     long_fired = false;

  for (;;) {
    bool pressed = g_button_ok && (digitalRead(PIN_BTN) == LOW);  // active LOW; off if stuck at boot

    if (pressed && !prev_raw) { press_t0 = millis(); long_fired = false; }
    if (pressed && !long_fired && (millis() - press_t0 > 1500)) {
      long_fired = true;                            // long press -> auto-calibrate (home to stops)
      g_autocal_request = true;                     // serviced by motorTask
    }
    // report the button only for short presses (mirror stock: long = calibration)
    g_button = pressed && !long_fired;
    prev_raw = pressed;

    if (tillerHID.send(g_button, g_axis)) {
      g_hidOk++; g_lastHidOkMs = millis();
    } else {
      g_hidFail++;
    }
    g_hidHb++;

    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(8));                    // ~125 Hz HID
  }
}

void supervisorTask(void *arg) {
  (void)arg;
  uint32_t prevMotor = g_motorHb, prevHid = g_hidHb, prevLoop = g_loopHb;
  uint32_t motorChanged = millis(), hidChanged = millis(), loopChanged = millis();
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(250));
    uint32_t now = millis();
    if (g_motorHb != prevMotor) { prevMotor = g_motorHb; motorChanged = now; }
    if (g_hidHb != prevHid) { prevHid = g_hidHb; hidChanged = now; }
    if (g_loopHb != prevLoop) { prevLoop = g_loopHb; loopChanged = now; }
    fillDiagRecord(g_rtcDiag, DIAG_NONE);
    if (!g_calibrating && now - motorChanged > SUPERVISOR_STALL_MS)
      saveFaultAndRestart(DIAG_MOTOR_STALL);
    if (now - hidChanged > SUPERVISOR_STALL_MS)
      saveFaultAndRestart(DIAG_HID_TASK_STALL);
    if (now - loopChanged > SUPERVISOR_STALL_MS)
      saveFaultAndRestart(DIAG_ARDUINO_LOOP_STALL);
    bool usbStalled = g_usbStarted && !g_usbSuspended && g_hidOk > 0 &&
                      now - g_lastHidOkMs > USB_REPORT_STALL_MS;
    if (usbStalled || g_usbRecoveryRequest) {
      g_usbRecoveryRequest = false;
      g_usbRecoveries++;
      uint32_t hidOkBefore = g_hidOk;
      persistFault(DIAG_USB_REPORT_STALL);

      // Re-enumerate only the USB device. Motor/sensor tasks continue without a
      // gap. On this old ESP32-S3 DCD, tud_disconnect() does not emit an internal
      // bus-reset event, so explicitly reset TinyUSB's software endpoint state;
      // the real host reset after reconnect then resets the DWC2 hardware state.
      tud_disconnect();
      vTaskDelay(pdMS_TO_TICKS(50));
      dcd_event_bus_reset(0, TUSB_SPEED_FULL, false);
      vTaskDelay(pdMS_TO_TICKS(USB_RECONNECT_OFF_MS - 50));
      tud_connect();

      uint32_t verifyStart = millis();
      while (g_hidOk == hidOkBefore && millis() - verifyStart < USB_RECOVERY_VERIFY_MS) {
        fillDiagRecord(g_rtcDiag, DIAG_USB_REPORT_STALL);
        vTaskDelay(pdMS_TO_TICKS(50));
      }
      if (g_hidOk == hidOkBefore) {
        // Never restart the whole tiller for a USB-only fault: steering torque
        // must remain continuous. Persist the failed attempt and let the next
        // supervisor pass retry local USB re-enumeration.
        persistFault(DIAG_USB_RECOVERY_FAILED);
      }
    }
  }
}

// ----------------------------------------------------------------------------
// Setup
// ----------------------------------------------------------------------------
void setup() {
  // Copy the wear-free RTC snapshot before supervisor starts overwriting it.
  g_bootDiagValid = g_rtcDiag.magic == DIAG_MAGIC && g_rtcDiag.version == DIAG_VERSION;
  if (g_bootDiagValid) g_bootDiag = g_rtcDiag;
  UsbSerial.begin(115200);                 // USB CDC (Commander / diagnostics / bootloader touch)
  delay(300);
  UsbSerial.println("\nPilotsEYE tiller - clean firmware (CDC build)");

  pinMode(PIN_BTN, INPUT_PULLUP);
  delay(5);
  g_button_ok = (digitalRead(PIN_BTN) == HIGH);   // if the wire is shorted/stuck LOW at boot, disable the button

  // Initial knob read; loop() then reads them LIVE, so force/damping adjust without a RESET.
  analogReadResolution(12);
  int rawF = analogRead(PIN_KNOB_FORCE); if (KNOB_FORCE_INVERT) rawF = 4095 - rawF;
  int rawD = analogRead(PIN_KNOB_DAMP);  if (KNOB_DAMP_INVERT)  rawD = 4095 - rawD;
  g_forceV = FORCE_V_MIN + (FORCE_V_MAX - FORCE_V_MIN) * (rawF / 4095.0f);
  g_kp = springForForce(g_forceV);
  g_dampK  = DAMP_MIN   + (DAMP_MAX   - DAMP_MIN)   * (rawD / 4095.0f);
  UsbSerial.printf("Knobs: force=%.2fV  damp=%.3f  (raw %d/%d)\n", g_forceV, g_dampK, rawF, rawD);

  loadCalib();
  UsbSerial.printf("Calibration: %s  min=%.3f max=%.3f center=%.3f\n",
                g_calibrated ? "loaded" : "DEFAULT (long-press button to calibrate range)",
                g_cal.angle_min, g_cal.angle_max, g_cal.angle_center);

  // USB identity - identical to stock so MSFS / X-Plane profiles keep working.
  USB.VID(TILLER_VID);
  USB.PID(TILLER_PID);
  USB.manufacturerName(TILLER_MANUFACTURER);
  USB.productName(TILLER_PRODUCT);
  USB.firmwareVersion(0x0100);          // bcdDevice = 1.00 (matches stock)
  USB.usbAttributes(0xC0);              // self-powered (matches stock bmAttributes)
  USB.onEvent(usbEvent);
  // serialNumber left default: the core uses the chip MAC, identical to stock.
  tillerHID.begin();
  USB.begin();
  delay(1500);
  UsbSerial.println("BOOT-DIAG stage=usb-ok");
  if (BOOT_DIAG_STOP_AFTER == 1) holdBootDiagnostic("usb-ok");

  // Sensor + motor
  sensor.init(&Wire, PIN_SDA, PIN_SCL);
  UsbSerial.println("BOOT-DIAG stage=sensor-ok");
  if (BOOT_DIAG_STOP_AFTER == 2) holdBootDiagnostic("sensor-ok");
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = SUPPLY_VOLTAGE;
  if (!driver.init()) {
    driver.disable();
    UsbSerial.println("BOOT-FAIL stage=driver-init; motor disabled, USB recovery active");
    holdBootDiagnostic("driver-failed");
  }
  UsbSerial.println("BOOT-DIAG stage=driver-ok");
  if (BOOT_DIAG_STOP_AFTER == 3) holdBootDiagnostic("driver-ok");
  motor.linkDriver(&driver);

  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller     = MotionControlType::torque;         // we render torque (voltage)
  motor.torque_controller = TorqueControlType::voltage;
  motor.velocity_limit = 1000;                              // rad/s, effectively unlimited here

  // Alignment must NOT depend on the force knob: initFOC needs enough voltage to turn the
  // motor, or pole/direction detection fails and shaft_angle stays 0 (sensor_direction UNKNOWN).
  motor.voltage_sensor_align = 2.0f;                        // same physical align voltage as 4 V / 24 V
  motor.voltage_limit = 6.0f;                               // headroom during init
  motor.useMonitoring(UsbSerial);
  if (!motor.init()) {
    driver.disable();
    UsbSerial.println("BOOT-FAIL stage=motor-init; motor disabled, USB recovery active");
    holdBootDiagnostic("motor-init-failed");
  }
  UsbSerial.println("BOOT-DIAG stage=motor-init-ok");
  if (BOOT_DIAG_STOP_AFTER == 4) holdBootDiagnostic("motor-init-ok");
  int foc_ok = motor.initFOC();                             // pole alignment (brief motion)
  UsbSerial.printf("MOT: initFOC %s, dir=%d\n", foc_ok ? "OK" : "FAILED", (int)motor.sensor_direction);
  if (!foc_ok) {
    driver.disable();
    UsbSerial.println("BOOT-FAIL stage=init-foc; motor disabled, USB recovery active");
    holdBootDiagnostic("init-foc-failed");
  }
  if (BOOT_DIAG_STOP_AFTER == 5) holdBootDiagnostic("init-foc-ok");
  motor.voltage_limit = g_ctrlMode == 2 ? fminf(STOCK_V_LIMIT, V_LINEAR_MAX) : g_forceV;   // normal operation

  // If no stored range yet, centre on the boot position (provisional +/-0.8 rad)
  // so the stick holds sensibly until 'C' / long-press calibration.
  if (!g_calibrated) {
    for (int i = 0; i < 100; i++) { motor.loopFOC(); delay(1); }
    float c = motor.shaft_angle;
    g_cal.angle_center = c;
    g_cal.angle_min    = c - 0.8f;
    g_cal.angle_max    = c + 0.8f;
    UsbSerial.printf("Provisional center=%.3f (auto-homing to stops next)\n", c);
  }

  // Commander (live tuning + diagnostics over USB serial). SimpleFOC's add() has
  // NO bounds check on its 20-slot table: a 21st command corrupts memory at boot.
  command.add('M', cmdMotor,  "motor");
  command.add('F', cmdForce,  "force voltage_limit");
  command.add('D', cmdDamp,   "damping gain");
  command.add('P', cmdKp,     "spring Kp");
  command.add('C', cmdCalib,  "manual sweep calibrate");
  command.add('A', cmdAuto,   "auto-calibrate (home to stops)");
  command.add('S', cmdStatus, "status");
  command.add('I', cmdSensor, "AS5600 magnet diag");
  command.add('K', cmdKnobs,  "raw ADC of knob pins");
  command.add('L', cmdLive,   "toggle live knobs");
  command.add('H', cmdStream, "toggle status stream");
  command.add('X', cmdDiag,   "persistent diagnostics");
  command.add('U', cmdUsbRecover, "local USB recovery test");
  command.add('R', cmdRecord, "record 1500 control passes and dump");
  command.add('Y', cmdSplit,  "damper velocity complementary split Hz");
  command.add('N', cmdVelDead,   "damper velocity dead zone rad/s");
  command.add('W', cmdVelFilter, "damper velocity low-pass Hz");
  command.add('J', cmdKick,      "stand: kick center 3 deg + record");
  command.add('Q', cmdSensorFilter, "AS5600 CONF: Qs SF, Qf FTH, Qh HYST");
  command.add('O', cmdVelMode,   "mode: O0 ours, O1 ours+biquad, O2 factory");   // 20th: Commander table is full

  // Task watchdog: if either task stalls > WDT_TIMEOUT_S -> clean reboot.
#if ESP_IDF_VERSION_MAJOR >= 5
  const esp_task_wdt_config_t wdtConfig = {
    .timeout_ms = WDT_TIMEOUT_S * 1000U,
    .idle_core_mask = 0,
    .trigger_panic = true,
  };
  g_wdtInitResult = esp_task_wdt_init(&wdtConfig);
#else
  g_wdtInitResult = esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif

  xTaskCreatePinnedToCore(motorTask, "motor_task", 4096, nullptr, 4, nullptr, 1);
  xTaskCreatePinnedToCore(hidTask,   "hid_task",   4096, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(supervisorTask, "supervisor", 4096, nullptr, 1, nullptr, 0);

  DiagRecord previous;
  if (loadSavedDiag(previous) && previous.fault != DIAG_NONE) {
    UsbSerial.println("Previous automatic recovery:");
    printDiagRecord(UsbSerial, previous);
  }
}

// Read the two pots continuously so force/damping adjust LIVE (no RESET needed).
void readKnobs() {
  static float fF = -1, fD = -1;
  int rawF = analogRead(PIN_KNOB_FORCE); if (KNOB_FORCE_INVERT) rawF = 4095 - rawF;
  int rawD = analogRead(PIN_KNOB_DAMP);  if (KNOB_DAMP_INVERT)  rawD = 4095 - rawD;
  g_lastRawF = rawF; g_lastRawD = rawD;
  float tF = FORCE_V_MIN + (FORCE_V_MAX - FORCE_V_MIN) * (rawF / 4095.0f);
  float tD = DAMP_MIN   + (DAMP_MAX   - DAMP_MIN)   * (rawD / 4095.0f);
  if (fF < 0) { fF = tF; fD = tD; }        // init on first call
  fF += 0.20f * (tF - fF);                 // EMA smoothing (ADC is noisy)
  fD += 0.20f * (tD - fD);
  g_forceV = fF;
  g_kp = springForForce(fF);                // same EMA as force, no stiffness jumps from ADC noise
  g_dampK  = fD;
  // Factory mapping on the raw (uninverted) ADC, EMA against flicker between integer steps.
  static float rF = -1, rD = -1;
  float physF = (float)analogRead(PIN_KNOB_FORCE), physD = (float)analogRead(PIN_KNOB_DAMP);
  if (rF < 0) { rF = physF; rD = physD; }
  rF += 0.20f * (physF - rF); rD += 0.20f * (physD - rD);
  g_stockVlim = (float)(map((long)(rF + 0.5f), 0, 4095, 20, 100) / 10);
  g_stockKv   = map((long)(rD + 0.5f), 0, 4095, 30, 150) / 100.0f;
  motor.voltage_limit = g_ctrlMode == 2 ? fminf(STOCK_V_LIMIT, V_LINEAR_MAX) : g_forceV;   // stock 12 V capped to the linear 6.9 V; live (guarded by g_calibrating in loop)
}

void loop() {
  g_loopHb++;
  command.run();          // service Commander on the Arduino task
  static uint32_t lastk = 0, last = 0;
  if (g_live_knobs && !g_calibrating && millis() - lastk > 40) { lastk = millis(); readKnobs(); }
  if (g_stream && millis() - last > 300) { last = millis(); cmdStatus(nullptr); }
  if (g_recState == 2) {
    UsbSerial.printf("REC begin n=%d kp=%.2f damp=%.3f force=%.2f mode=%d obs=%.0f split=%.0f cut=%.0f stock_vlim=%.0f stock_kv=%.2f dead=%.2f reg=0x%02X kick_aborted=%d\n",
                     g_recLen, g_kp, g_dampK, g_forceV, (int)g_ctrlMode, g_obsHz, g_splitHz, g_velCutHz, g_stockVlim, g_stockKv,
                     g_velDeadRad, (unsigned)sensor.angle_reg, (int)g_kickAborted);
    for (int i = 0; i < g_recLen; i++) {
      UsbSerial.printf("%lu %.5f %.4f %.4f\n", (unsigned long)g_rec[i].t, g_rec[i].a, g_rec[i].v, g_rec[i].u);
      g_loopHb++;                     // a slow host must not look like a stalled loop to the supervisor
      if ((i & 63) == 63) delay(1);
    }
    UsbSerial.println("REC end");
    g_recState = 0;
  }
  delay(2);
}
