#pragma once

#include <math.h>
#include <stdint.h>
#include <string.h>

// Runtime tuning over CAN (CAN_TUNE, 2026-11-17): every parameter change used to cost a reflash of the
// modules - the arm back to vertical, the engines off, a recalibration. The host's POST /tune sends
// one parameter of one joint at a time and waits for the answer.
//
// Request (CAN_TUNE): byte 0 the joint (TUNE_JOINT_*), byte 1 the parameter (TUNE_*), byte 2 bit 0 set
// (else only read), bytes 4-7 the value (float). Answer (the module's *_TUNED id): the same joint and
// parameter, byte 2 bit 0 known, bytes 4-7 the value now in effect - clamped to a safe range, so a typo
// can't do what a faster damping did on 2026-11-12 (see Servo::tune()). The ids and numbers are shared
// with the host (its include/config.h) - change both together.

#define TUNE_JOINT_SHOULDER_Y 1
#define TUNE_JOINT_SHOULDER_Z 2
#define TUNE_JOINT_ELBOW_Y 3
#define TUNE_JOINT_WRIST_Y 4
#define TUNE_JOINT_WRIST_Z 5

#define TUNE_DAMPING_GAIN 1         // s, Servo::setDamping()
#define TUNE_DAMPING_MAX 2          // deg
#define TUNE_DAMPING_DEADBAND 3     // deg/s
#define TUNE_DAMPING_RATE_TAU 4     // s, the module's RateEstimator (the link rate's low-pass)
#define TUNE_DITHER_DEG 5           // Servo::setDither()
#define TUNE_DITHER_HALF_PERIOD_MS 6
#define TUNE_STREAM_GAIN 7          // 1/s, see STREAM_GAIN
#define TUNE_STREAM_ERROR_TAU 8     // s, see STREAM_ERROR_TAU_SEC
#define TUNE_SETTLE_TOLERANCE 9     // deg, see STREAM_SETTLE_TOLERANCE_DEG
#define TUNE_SETTLE_SPEED 10        // deg/s, see STREAM_SETTLE_PLAN_SPEED_DEG_PER_SEC
#define TUNE_CALM_SPAN 11           // deg, see STREAM_CALM_SPAN_DEG
#define TUNE_CALM_BLOCK_MS 12       // see STREAM_CALM_BLOCK_MS
#define TUNE_CALM_WAIT_MAX_MS 13    // see STREAM_CALM_WAIT_MAX_MS
#define TUNE_HOLD_WAKE_DEG 14       // Servo::setHoldHysteresis()
#define TUNE_HOLD_WAKE_MS 15
#define TUNE_HOLD_WAKE_NOW_DEG 16
#define TUNE_PARENT_WAIT 17         // 0/1, Servo::setAncestorsSettling() honoured
#define TUNE_PARENT_FOLLOW 18       // 0/1, Servo::setParentSettleSpeed() honoured
#define TUNE_ENTRY_SEC 19           // s, see STREAM_ENTRY_SEC
#define TUNE_CORRECTION_MAX 20      // deg, see STREAM_CORRECTION_MAX_DEG
#define TUNE_RESET_IMU 21           // set 1: reset the joint's BNO now (a test of the in-place recovery);
                                    // an action, never kept in the host's profile
#define TUNE_FUSION_GATE 22         // deg, see STREAM_FUSION_GATE_DEG (30 = off)
#define TUNE_LEAD_SMALLER 23        // deg, the break-away lead toward smaller angles as learned so far
#define TUNE_LEAD_LARGER 24         // (Servo::setBreakawayLead()); set replaces it - state, not a setting:
                                    // never kept in the host's profile (a reboot starts from the config)
#define TUNE_LEAD_LEARN 25          // 0/1, the lead learned from each stream (0: kept as it is)
#define TUNE_LEAD_FULL_SPEED 26     // deg/s, see STREAM_LEAD_FULL_SPEED_DEG_PER_SEC
#define TUNE_PARAM_COUNT 26

struct TuneRequest {
  uint8_t joint;
  uint8_t param;
  bool set;
  float value;
};

inline TuneRequest decodeTune(const uint8_t data[8]) {
  TuneRequest r;
  r.joint = data[0];
  r.param = data[1];
  r.set = (data[2] & 1) != 0;
  memcpy(&r.value, data + 4, sizeof(float));
  return r;
}

// The answer: value NaN = a parameter this joint doesn't have.
inline uint64_t encodeTuned(uint8_t joint, uint8_t param, float value) {
  uint8_t data[8] = {joint, param, (uint8_t)(isnan(value) ? 0 : 1), 0};
  memcpy(data + 4, &value, sizeof(float));
  uint64_t out;
  memcpy(&out, data, sizeof(out));
  return out;
}
