#pragma once

// Side-effect-free telescope geometry and command conversion.
//
// This header deliberately contains no GPIO, ROS, or motor-driver code. The
// assembled mechanism is not measured yet, so the dummy geometry must reject
// all conversions until a future hardware module supplies real calibration.

#include <stdint.h>
#include "../config/teensy/firmware_config.h"

namespace telescope {

struct Geometry {
  bool enabled;
  bool configured;
  float min_length_m;
  float max_length_m;
  float linear_travel_per_motor_revolution_m;
  int32_t encoder_zero_count_at_min_length;
  int32_t calibrated_count_span_to_max_length;
  int8_t direction_sign;
};

// DUMMY: replace from hardware.yaml after the mechanism is measured.
static const Geometry kDummyGeometry = {
    ig_handle_firmware_config::telescope::kEnabled,
    ig_handle_firmware_config::telescope::kConfigured,
    ig_handle_firmware_config::telescope::kMinLengthM,
    ig_handle_firmware_config::telescope::kMaxLengthM,
    ig_handle_firmware_config::telescope::kLinearTravelPerMotorRevolutionM,
    ig_handle_firmware_config::telescope::kEncoderZeroCountAtMinLength,
    ig_handle_firmware_config::telescope::kEncoderCountSpanToMaxLength,
    ig_handle_firmware_config::telescope::kDirectionSign};

inline bool geometryReady(const Geometry& geometry) {
  return geometry.enabled && geometry.configured &&
         geometry.max_length_m > geometry.min_length_m &&
         geometry.linear_travel_per_motor_revolution_m > 0.0f &&
         geometry.calibrated_count_span_to_max_length > 0;
}

inline bool lengthFromEncoder(const Geometry& geometry,
                              int32_t encoder_count,
                              float* length_m) {
  if (length_m == 0 || !geometryReady(geometry)) {
    return false;
  }

  int32_t delta = encoder_count - geometry.encoder_zero_count_at_min_length;
  if (delta < 0) {
    delta = 0;
  }
  if (delta > geometry.calibrated_count_span_to_max_length) {
    delta = geometry.calibrated_count_span_to_max_length;
  }

  const float fraction =
      static_cast<float>(delta) /
      static_cast<float>(geometry.calibrated_count_span_to_max_length);
  *length_m = geometry.min_length_m +
              fraction * (geometry.max_length_m - geometry.min_length_m);
  return true;
}

inline bool encoderForLength(const Geometry& geometry,
                             float length_m,
                             int32_t* encoder_count) {
  if (encoder_count == 0 || !geometryReady(geometry) ||
      length_m < geometry.min_length_m || length_m > geometry.max_length_m) {
    return false;
  }

  const float fraction =
      (length_m - geometry.min_length_m) /
      (geometry.max_length_m - geometry.min_length_m);
  const float count_delta =
      fraction * static_cast<float>(geometry.calibrated_count_span_to_max_length);
  *encoder_count = geometry.encoder_zero_count_at_min_length +
                   static_cast<int32_t>(count_delta + 0.5f);
  return true;
}

inline bool motorRevolutionsForLength(const Geometry& geometry,
                                      float length_m,
                                      float* motor_revolutions) {
  if (motor_revolutions == 0 || !geometryReady(geometry) ||
      length_m < geometry.min_length_m || length_m > geometry.max_length_m) {
    return false;
  }

  const float travel_m = length_m - geometry.min_length_m;
  *motor_revolutions =
      static_cast<float>(geometry.direction_sign) *
      travel_m / geometry.linear_travel_per_motor_revolution_m;
  return true;
}

}  // namespace telescope
