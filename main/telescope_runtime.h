#pragma once

// Disabled-by-default runtime for the telescoping sonar arm.
//
// This module owns only the telescope pins. It is deliberately inert unless
// the measured hardware configuration, wiring verification, and explicit
// enable flag are all true in firmware_config.h.

#include <Arduino.h>
#include <limits.h>
#include <math.h>

#include "telescope_control.h"

namespace telescope {

class Runtime {
public:
  Runtime()
      : active_(false),
        homed_(false),
        home_requested_(false),
        command_received_(false),
        fatal_fault_latched_(false),
        desired_length_m_(NAN),
        command_received_ms_(0),
        homing_started_ms_(0),
        last_update_ms_(0),
        last_encoder_change_ms_(0),
        min_raw_changed_ms_(0),
        max_raw_changed_ms_(0),
        reversal_blank_until_ms_(0),
        last_encoder_count_(0),
        min_raw_valid_(false),
        max_raw_valid_(false),
        min_raw_pressed_(false),
        max_raw_pressed_(false),
        limits_qualified_(false),
        min_pressed_(false),
        max_pressed_(false),
        reversal_blank_active_(false),
        last_requested_direction_(0),
        motor_duty_(0.0f),
        status_("disabled") {}

  bool begin(uint32_t now_ms) {
    if (!canOperate()) {
      status_ = "disabled_unconfigured";
      return false;
    }

    using namespace ig_handle_firmware_config::telescope;
    // Apply software-safe levels before validation reads. External hard pulls
    // and a hardware driver-disable remain mandatory during MCU reset/high-Z.
    digitalWrite(kMotorRightPwmPin, LOW);
    digitalWrite(kMotorLeftPwmPin, LOW);
    digitalWrite(kMotorRightEnablePin, LOW);
    digitalWrite(kMotorLeftEnablePin, LOW);
    pinMode(kMotorRightPwmPin, OUTPUT);
    pinMode(kMotorLeftPwmPin, OUTPUT);
    pinMode(kMotorRightEnablePin, OUTPUT);
    pinMode(kMotorLeftEnablePin, OUTPUT);
    pinMode(kMotorRightCurrentSensePin, INPUT);
    pinMode(kMotorLeftCurrentSensePin, INPUT);
    pinMode(kEncoderPhaseAPin, INPUT_PULLUP);
    pinMode(kEncoderPhaseBPin, INPUT_PULLUP);
    if (kMinLimitPresent) {
      pinMode(kMinLimitNoPin, INPUT_PULLUP);
      pinMode(kMinLimitNcPin, INPUT_PULLUP);
    }
    if (kMaxLimitPresent) {
      pinMode(kMaxLimitNoPin, INPUT_PULLUP);
      pinMode(kMaxLimitNcPin, INPUT_PULLUP);
    }
    analogWriteFrequency(kMotorRightPwmPin, kPwmFrequencyHz);
    analogWriteFrequency(kMotorLeftPwmPin, kPwmFrequencyHz);
    stopMotor();
    active_ = true;
    sampleRawLimits(&min_raw_valid_, &min_raw_pressed_, &max_raw_valid_, &max_raw_pressed_);
    min_pressed_ = false;
    max_pressed_ = false;
    limits_qualified_ = false;
    last_update_ms_ = now_ms;
    last_encoder_change_ms_ = now_ms;
    min_raw_changed_ms_ = now_ms;
    max_raw_changed_ms_ = now_ms;
    encoder_state_ = encoderState();
    last_encoder_count_ = encoderCount();
    status_ = "limit_qualifying";
    return true;
  }

  void onEncoderEdge() {
    if (!active_ || fatal_fault_latched_) {
      return;
    }
    const uint8_t next_state = encoderState();
    // Gray-code transition table for x4 quadrature decoding. Invalid two-bit
    // jumps contribute zero counts instead of inventing motion.
    static const int8_t kTransition[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
    const int8_t increment = kTransition[(encoder_state_ << 2) | next_state];
    if ((increment > 0 && encoder_count_ < INT32_MAX) || (increment < 0 && encoder_count_ > INT32_MIN)) {
      encoder_count_ += increment;
    }
    encoder_state_ = next_state;
  }

  void setDesiredLength(float desired_length_m, uint32_t now_ms) {
    if (!active_) {
      status_ = "disabled_unconfigured";
      return;
    }
    if (fatal_fault_latched_) {
      stopMotor();
      return;
    }
    const Geometry geometry = geometryFromFirmware();
    if (!isfinite(desired_length_m) || desired_length_m < geometry.min_length_m ||
        desired_length_m > geometry.max_length_m) {
      command_received_ = false;
      stopMotor();
      status_ = "command_out_of_range";
      return;
    }
    const bool new_target =
        !command_received_ || !isfinite(desired_length_m_) || fabs(desired_length_m - desired_length_m_) > 1e-5f;
    desired_length_m_ = desired_length_m;
    command_received_ms_ = now_ms;
    command_received_ = true;
    const bool request_home = !homed_ && fabs(desired_length_m - geometry.min_length_m) <= 1e-5f;
    if (request_home && !home_requested_) {
      homing_started_ms_ = now_ms;
    }
    if (new_target || (request_home && !home_requested_)) {
      // A genuinely new motion intent gets a fresh observation window. Normal
      // rosserial refreshes of the same target do not mask an encoder stall.
      last_encoder_count_ = encoderCount();
      last_encoder_change_ms_ = now_ms;
    }
    home_requested_ = request_home;
    status_ = home_requested_ ? "homing" : "tracking";
  }

  void update(uint32_t now_ms) {
    if (!active_) {
      return;
    }
    if (fatal_fault_latched_) {
      stopMotor();
      return;
    }
    using namespace ig_handle_firmware_config::telescope;
    if (now_ms - last_update_ms_ < kControlUpdatePeriodMs) {
      return;
    }
    const float elapsed_s = static_cast<float>(now_ms - last_update_ms_) / 1000.0f;
    last_update_ms_ = now_ms;

    if (!updateDebouncedLimits(now_ms)) {
      latchFatalFault("limit_disagreement");
      return;
    }
    if (!limits_qualified_) {
      stopMotor();
      status_ = "limit_qualifying";
      return;
    }

    if (min_pressed_) {
      rebaseAtMinimum(now_ms);
      homed_ = true;
      home_requested_ = false;
    }
    if (!command_received_ || now_ms - command_received_ms_ > kCommandTimeoutMs) {
      command_received_ = false;
      home_requested_ = false;
      stopMotor();
      status_ = "command_timeout";
      return;
    }

    const float current_a = motorCurrentA();
    if (isfinite(current_a) && current_a > kMaxMotorCurrentA) {
      latchFatalFault("motor_current_limit");
      return;
    }

    if (!homed_) {
      if (!home_requested_) {
        stopMotor();
        status_ = "needs_home";
        return;
      }
      if (max_pressed_) {
        stopMotor();
        status_ = "homing_blocked_at_maximum";
        return;
      }
      if (now_ms - homing_started_ms_ > kHomingTimeoutMs) {
        latchFatalFault("homing_timeout");
        return;
      }
      const int32_t homing_count = encoderCount();
      if (homing_count != last_encoder_count_) {
        last_encoder_count_ = homing_count;
        last_encoder_change_ms_ = now_ms;
      } else if (now_ms - last_encoder_change_ms_ > kStallTimeoutMs) {
        latchFatalFault("encoder_stall");
        return;
      }
      motor_duty_ =
          static_cast<float>(kHomingDirection) * static_cast<float>(kMotorDutySignForExtension) * kHomingDutyFraction;
      status_ = driveMotor(motor_duty_, now_ms) ? "homing" : "direction_reversal_blank";
      return;
    }

    const Geometry geometry = geometryFromFirmware();
    int32_t target_count = 0;
    if (!encoderForLength(geometry, desired_length_m_, &target_count)) {
      latchFatalFault("invalid_calibration");
      return;
    }
    const int32_t current_count = encoderCount();
    const int64_t count_error = static_cast<int64_t>(target_count) - current_count;
    const bool extending = count_error > 0;
    if ((min_pressed_ && !extending) || (max_pressed_ && extending)) {
      stopMotor();
      status_ = min_pressed_ ? "at_minimum" : "at_maximum";
      return;
    }

    if (count_error >= -1 && count_error <= 1) {
      stopMotor();
      status_ = "at_target";
      return;
    }

    const float normalized_error = static_cast<float>(count_error < 0 ? -count_error : count_error) /
                                   static_cast<float>(geometry.calibrated_count_span_to_max_length);
    float requested_duty = kPositionKp * normalized_error;
    if (requested_duty > kMaxDutyFraction) {
      requested_duty = kMaxDutyFraction;
    }
    const float signed_target_duty =
        (extending ? 1.0f : -1.0f) * static_cast<float>(kMotorDutySignForExtension) * requested_duty;
    const float max_step = kMaxDutyAccelerationPerSec * elapsed_s;
    if (signed_target_duty > motor_duty_ + max_step) {
      motor_duty_ += max_step;
    } else if (signed_target_duty < motor_duty_ - max_step) {
      motor_duty_ -= max_step;
    } else {
      motor_duty_ = signed_target_duty;
    }

    if (current_count != last_encoder_count_) {
      last_encoder_count_ = current_count;
      last_encoder_change_ms_ = now_ms;
    } else if (fabs(motor_duty_) > 0.05f && now_ms - last_encoder_change_ms_ > kStallTimeoutMs) {
      latchFatalFault("encoder_stall");
      return;
    }

    status_ = driveMotor(motor_duty_, now_ms) ? "tracking" : "direction_reversal_blank";
  }

  bool active() const { return active_; }
  bool fatalFaultLatched() const { return fatal_fault_latched_; }
  const char* status() const { return status_; }

  float actualLengthM() const {
    if (!active_ || !homed_) {
      return NAN;
    }
    float length_m = NAN;
    lengthFromEncoder(geometryFromFirmware(), encoderCount(), &length_m);
    return length_m;
  }

  float motorCurrentA() const {
    using namespace ig_handle_firmware_config::telescope;
    if (!active_ || !kCurrentSenseConfigured || kMotorCurrentAPerAdcCount <= 0.0f) {
      return NAN;
    }
    const int raw = max(analogRead(kMotorRightCurrentSensePin), analogRead(kMotorLeftCurrentSensePin));
    const float adjusted = static_cast<float>(raw) - kMotorCurrentZeroAdcCount;
    return adjusted > 0.0f ? adjusted * kMotorCurrentAPerAdcCount : 0.0f;
  }

private:
  static volatile int32_t encoder_count_;
  static volatile uint8_t encoder_state_;

  static Geometry geometryFromFirmware() { return kDummyGeometry; }

  static bool canOperate() {
    using namespace ig_handle_firmware_config::telescope;
    const Geometry geometry = geometryFromFirmware();
    return kEnabled && kConfigured && kWiringVerified && kCurrentSenseConfigured && kMotorCurrentAPerAdcCount > 0.0f &&
           kMaxMotorCurrentA > 0.0f && kHomingDutyFraction > 0.0f && kHomingDutyFraction <= kMaxDutyFraction &&
           kMaxDutyFraction <= 1.0f && kControlUpdatePeriodMs > 0 && kCommandTimeoutMs > 0 && kHomingTimeoutMs > 0 &&
           kStallTimeoutMs > 0 && kLimitDebounceMs > 0 && kDirectionReversalBlankMs > 0 && kMinLimitPresent &&
           kMinLimitNoPin != ig_handle_firmware_config::kInvalidPin &&
           (!kRequireRedundantLimitAgreement || kMinLimitNcPin != ig_handle_firmware_config::kInvalidPin) &&
           (!kMaxLimitPresent ||
            (kMaxLimitNoPin != ig_handle_firmware_config::kInvalidPin &&
             (!kRequireRedundantLimitAgreement || kMaxLimitNcPin != ig_handle_firmware_config::kInvalidPin))) &&
           (kHomingDirection == 1 || kHomingDirection == -1) &&
           (kMotorDutySignForExtension == 1 || kMotorDutySignForExtension == -1) && geometryReady(geometry);
  }

  static uint8_t encoderState() {
    using namespace ig_handle_firmware_config::telescope;
    const uint8_t phase_a = digitalRead(kEncoderPhaseAPin) == HIGH ? 2 : 0;
    const uint8_t phase_b = digitalRead(kEncoderPhaseBPin) == HIGH ? 1 : 0;
    return phase_a | phase_b;
  }

  static int32_t encoderCount() {
    noInterrupts();
    const int32_t raw_count = encoder_count_;
    interrupts();
    using namespace ig_handle_firmware_config::telescope;
    if (kEncoderCountIncreasesOnExtension) {
      return raw_count;
    }
    const int64_t reversed = 2LL * static_cast<int64_t>(kEncoderZeroCountAtMinLength) - raw_count;
    if (reversed > INT32_MAX) {
      return INT32_MAX;
    }
    if (reversed < INT32_MIN) {
      return INT32_MIN;
    }
    return static_cast<int32_t>(reversed);
  }

  void latchFatalFault(const char* status) {
    fatal_fault_latched_ = true;
    command_received_ = false;
    home_requested_ = false;
    stopMotor();
    status_ = status;
  }

  bool updateDebouncedLimits(uint32_t now_ms) {
    bool min_raw_valid = false;
    bool max_raw_valid = false;
    bool min_raw_pressed = false;
    bool max_raw_pressed = false;
    sampleRawLimits(&min_raw_valid, &min_raw_pressed, &max_raw_valid, &max_raw_pressed);
    using namespace ig_handle_firmware_config::telescope;
    if (min_raw_valid != min_raw_valid_ || min_raw_pressed != min_raw_pressed_) {
      min_raw_valid_ = min_raw_valid;
      min_raw_pressed_ = min_raw_pressed;
      min_raw_changed_ms_ = now_ms;
    }
    if (max_raw_valid != max_raw_valid_ || max_raw_pressed != max_raw_pressed_) {
      max_raw_valid_ = max_raw_valid;
      max_raw_pressed_ = max_raw_pressed;
      max_raw_changed_ms_ = now_ms;
    }
    const bool min_stable = now_ms - min_raw_changed_ms_ >= kLimitDebounceMs;
    const bool max_stable = now_ms - max_raw_changed_ms_ >= kLimitDebounceMs;
    limits_qualified_ = min_stable && max_stable && min_raw_valid_ && max_raw_valid_;
    if (min_stable && !min_raw_valid_) {
      return false;
    }
    if (max_stable && !max_raw_valid_) {
      return false;
    }
    if (limits_qualified_) {
      min_pressed_ = min_raw_pressed_;
      max_pressed_ = max_raw_pressed_;
    }
    return !(limits_qualified_ && min_pressed_ && max_pressed_);
  }

  void sampleRawLimits(bool* min_valid, bool* min_pressed, bool* max_valid, bool* max_pressed) const {
    using namespace ig_handle_firmware_config::telescope;
    *min_valid = false;
    *min_pressed = false;
    *max_valid = !kMaxLimitPresent;
    *max_pressed = false;
    if (!kRequireRedundantLimitAgreement) {
      *min_valid = kMinLimitPresent;
      *min_pressed = digitalRead(kMinLimitNoPin) == LOW;
      if (kMaxLimitPresent) {
        *max_valid = true;
        *max_pressed = digitalRead(kMaxLimitNoPin) == LOW;
      }
      return;
    }
    *min_valid = decodeLimit(kMinLimitNoPin, kMinLimitNcPin, min_pressed);
    if (kMaxLimitPresent) {
      *max_valid = decodeLimit(kMaxLimitNoPin, kMaxLimitNcPin, max_pressed);
    }
  }

  static bool decodeLimit(uint8_t no_pin, uint8_t nc_pin, bool* pressed) {
    if (pressed == 0) {
      return false;
    }
    const bool no_active = digitalRead(no_pin) == LOW;
    const bool nc_active = digitalRead(nc_pin) == LOW;
    if (no_active == nc_active) {
      return false;
    }
    *pressed = no_active;
    return true;
  }

  void rebaseAtMinimum(uint32_t now_ms) {
    noInterrupts();
    encoder_count_ = ig_handle_firmware_config::telescope::kEncoderZeroCountAtMinLength;
    interrupts();
    last_encoder_count_ = encoderCount();
    last_encoder_change_ms_ = now_ms;
  }

  bool driveMotor(float duty, uint32_t now_ms) {
    using namespace ig_handle_firmware_config::telescope;
    const float bounded = constrain(fabs(duty), 0.0f, 1.0f);
    const int pwm = static_cast<int>(bounded * 255.0f + 0.5f);
    if (pwm == 0) {
      stopMotor();
      return true;
    }
    const int8_t direction = duty > 0.0f ? 1 : -1;
    if (last_requested_direction_ != 0 && direction != last_requested_direction_) {
      powerOffMotor();
      motor_duty_ = 0.0f;
      last_requested_direction_ = direction;
      reversal_blank_until_ms_ = now_ms + kDirectionReversalBlankMs;
      reversal_blank_active_ = true;
      return false;
    }
    if (last_requested_direction_ == 0) {
      last_requested_direction_ = direction;
    }
    powerOffMotor();
    if (reversal_blank_active_) {
      if (static_cast<int32_t>(now_ms - reversal_blank_until_ms_) < 0) {
        motor_duty_ = 0.0f;
        return false;
      }
      reversal_blank_active_ = false;
    }
    digitalWrite(kMotorRightEnablePin, HIGH);
    digitalWrite(kMotorLeftEnablePin, HIGH);
    if (duty > 0.0f) {
      analogWrite(kMotorRightPwmPin, pwm);
      analogWrite(kMotorLeftPwmPin, 0);
    } else {
      analogWrite(kMotorRightPwmPin, 0);
      analogWrite(kMotorLeftPwmPin, pwm);
    }
    return true;
  }

  void powerOffMotor() {
    using namespace ig_handle_firmware_config::telescope;
    analogWrite(kMotorRightPwmPin, 0);
    analogWrite(kMotorLeftPwmPin, 0);
    digitalWrite(kMotorRightEnablePin, LOW);
    digitalWrite(kMotorLeftEnablePin, LOW);
  }

  void stopMotor() {
    powerOffMotor();
    motor_duty_ = 0.0f;
  }

  volatile bool active_;
  bool homed_;
  bool home_requested_;
  bool command_received_;
  volatile bool fatal_fault_latched_;
  float desired_length_m_;
  uint32_t command_received_ms_;
  uint32_t homing_started_ms_;
  uint32_t last_update_ms_;
  uint32_t last_encoder_change_ms_;
  uint32_t min_raw_changed_ms_;
  uint32_t max_raw_changed_ms_;
  uint32_t reversal_blank_until_ms_;
  int32_t last_encoder_count_;
  bool min_raw_valid_;
  bool max_raw_valid_;
  bool min_raw_pressed_;
  bool max_raw_pressed_;
  bool limits_qualified_;
  bool min_pressed_;
  bool max_pressed_;
  bool reversal_blank_active_;
  int8_t last_requested_direction_;
  float motor_duty_;
  const char* status_;
};

volatile int32_t Runtime::encoder_count_ = 0;
volatile uint8_t Runtime::encoder_state_ = 0;

}  // namespace telescope
