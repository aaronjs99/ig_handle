#pragma once

// Whole-firmware Teensy 4.1 pin contract.
//
// Timing and telescope runtimes deliberately remain separate owners.  This
// validator is the one place that rejects cross-owner aliases, RTC I2C
// collisions, and unsupported PWM/ADC assignments before either owner touches
// enabled hardware.

#include <stdint.h>

#include "firmware_config.h"

namespace firmware_pin_contract {

class PinSet {
public:
  PinSet() : count_(0) {}

  bool add(uint8_t pin) {
    if (!digitalPinValid(pin) || pin == 18 || pin == 19 || count_ >= kCapacity) {
      return false;
    }
    for (uint8_t index = 0; index < count_; ++index) {
      if (pins_[index] == pin) {
        return false;
      }
    }
    pins_[count_++] = pin;
    return true;
  }

private:
  static constexpr uint8_t kCapacity = 32;

  static bool digitalPinValid(uint8_t pin) { return pin != ig_handle_firmware_config::kInvalidPin && pin <= 54; }

  uint8_t pins_[kCapacity];
  uint8_t count_;
};

inline bool pwmPinValid(uint8_t pin) {
  switch (pin) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
    case 18:
    case 19:
    case 22:
    case 23:
    case 24:
    case 25:
    case 28:
    case 29:
    case 33:
    case 36:
    case 37:
    case 42:
    case 43:
    case 44:
    case 45:
    case 46:
    case 47:
    case 51:
    case 54:
      return true;
    default:
      return false;
  }
}

inline bool adcPinValid(uint8_t pin) { return (pin >= 14 && pin <= 27) || (pin >= 38 && pin <= 41); }

inline bool timingPinsValid(PinSet* pins) {
  using namespace ig_handle_firmware_config::timing;
  const bool any_timing = kCameraTriggerEnabled || kCameraFeedbackEnabled || kImuTriggerEnabled ||
                          kImuFeedbackEnabled || kLidarClockEnabled;
  if (!any_timing) {
    return true;
  }
  if (!pins->add(kReferenceInputPin)) {
    return false;
  }
  if (kCameraTriggerEnabled) {
    if (kCameraUseHardwareFanout) {
      if (!pins->add(kCameraFanoutTriggerPin)) {
        return false;
      }
    } else {
      for (uint8_t index = 0; index < kCameraCount; ++index) {
        if (!pins->add(kCameraTriggerPins[index])) {
          return false;
        }
      }
    }
  }
  if (kCameraFeedbackEnabled) {
    for (uint8_t index = 0; index < kCameraCount; ++index) {
      if (!pins->add(kCameraExposurePins[index])) {
        return false;
      }
    }
  }
  if (kImuTriggerEnabled && !pins->add(kImuTriggerPin)) {
    return false;
  }
  if (kImuFeedbackEnabled && !pins->add(kImuSyncPin)) {
    return false;
  }
  if (kLidarClockEnabled) {
    for (uint8_t index = 0; index < kLidarCount; ++index) {
      if (!pins->add(kLidarPpsPins[index])) {
        return false;
      }
    }
  }
  return true;
}

inline bool telescopePinsValid(PinSet* pins) {
  using namespace ig_handle_firmware_config::telescope;
  if (!kEnabled) {
    return true;
  }
  if (!pwmPinValid(kMotorRightPwmPin) || !pwmPinValid(kMotorLeftPwmPin) || !adcPinValid(kMotorRightCurrentSensePin) ||
      !adcPinValid(kMotorLeftCurrentSensePin)) {
    return false;
  }
  const uint8_t required[] = {kMotorRightPwmPin,   kMotorLeftPwmPin,           kMotorRightEnablePin,
                              kMotorLeftEnablePin, kMotorRightCurrentSensePin, kMotorLeftCurrentSensePin,
                              kEncoderPhaseAPin,   kEncoderPhaseBPin,          kMinLimitNoPin};
  for (uint8_t index = 0; index < sizeof(required) / sizeof(required[0]); ++index) {
    if (!pins->add(required[index])) {
      return false;
    }
  }
  if (kRequireRedundantLimitAgreement && !pins->add(kMinLimitNcPin)) {
    return false;
  }
  if (kMaxLimitPresent) {
    if (!pins->add(kMaxLimitNoPin) || (kRequireRedundantLimitAgreement && !pins->add(kMaxLimitNcPin))) {
      return false;
    }
  }
  return true;
}

inline bool valid() {
  PinSet pins;
  return timingPinsValid(&pins) && telescopePinsValid(&pins);
}

}  // namespace firmware_pin_contract
