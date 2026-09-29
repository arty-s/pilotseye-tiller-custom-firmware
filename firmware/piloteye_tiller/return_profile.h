#pragma once

#include <math.h>
#include <stdint.h>

class TillerReturnProfile {
public:
  struct Output {
    float springScale;
    float springError;
    float dampingScale;
    float returnBlend;
    bool returning;
  };

  static constexpr float ACTIVE_SPRING_SCALE = 1.00f;
  static constexpr float RETURN_SPRING_SCALE = 1.00f;
  static constexpr float RETURN_DAMPING_SCALE = 1.00f;
  static constexpr float CENTER_QUIET_RAD = 0.040f; // about 2.3 degrees
  static constexpr float CENTER_FULL_RAD = 0.080f;  // full spring from about 4.6 degrees
  static constexpr float MOTION_THRESHOLD_RAD_S = 0.10f;
  static constexpr uint32_t IDLE_DELAY_MS = 700;
  static constexpr uint32_t RETURN_RAMP_MS = 1400;

  void reset(uint32_t nowMs) {
    initialized_ = true;
    returning_ = false;
    lastAwayMotionMs_ = nowMs;
    returnStartMs_ = nowMs;
  }

  Output update(uint32_t nowMs, float error, float velocity) {
    if (!initialized_) reset(nowMs);

    // error = center - angle. A negative product means unambiguous pilot
    // movement away from center. Natural motion toward center must not keep
    // the controller in active mode forever.
    const bool movingAway = fabsf(velocity) >= MOTION_THRESHOLD_RAD_S &&
                            error * velocity < 0.0f;
    if (movingAway) {
      returning_ = false;
      lastAwayMotionMs_ = nowMs;
    } else if (!returning_ && nowMs - lastAwayMotionMs_ >= IDLE_DELAY_MS) {
      returning_ = true;
      returnStartMs_ = nowMs;
    }

    float blend = 0.0f;
    if (returning_) {
      float t = (float)(nowMs - returnStartMs_) / (float)RETURN_RAMP_MS;
      if (t > 1.0f) t = 1.0f;
      blend = t * t * (3.0f - 2.0f * t); // smoothstep: no torque step
    }

    const float absError = fabsf(error);
    float springError = error;
    if (absError <= CENTER_QUIET_RAD) {
      springError = 0.0f;
    } else if (absError < CENTER_FULL_RAD) {
      float t = (absError - CENTER_QUIET_RAD) /
                (CENTER_FULL_RAD - CENTER_QUIET_RAD);
      float centerBlend = t * t * (3.0f - 2.0f * t);
      springError = error * centerBlend;
    }

    Output out;
    out.springScale = ACTIVE_SPRING_SCALE +
      (RETURN_SPRING_SCALE - ACTIVE_SPRING_SCALE) * blend;
    out.springError = springError;
    out.dampingScale = 1.0f + (RETURN_DAMPING_SCALE - 1.0f) * blend;
    out.returnBlend = blend;
    out.returning = returning_;
    return out;
  }

private:
  bool initialized_ = false;
  bool returning_ = false;
  uint32_t lastAwayMotionMs_ = 0;
  uint32_t returnStartMs_ = 0;
};
