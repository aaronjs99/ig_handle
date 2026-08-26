#pragma once

// Side-effect-free telescope geometry and command conversion.
//
// This header deliberately contains no GPIO, ROS, or motor-driver code. The
// assembled mechanism is not measured yet, so the uncommissioned geometry
// must reject all conversions until measured calibration is supplied.

#include <limits.h>
#include <stdint.h>

#include "firmware_config.h"

namespace telescope {

struct Geometry {
  bool enabled;
  bool configured;
  float min_length_m;
  float max_length_m;
  int32_t encoder_zero_count_at_min_length;
  int32_t calibrated_count_span_to_max_length;
};

// Compile-time mirror of the disabled measurement-required hardware contract.
static const Geometry kUncommissionedGeometry = {ig_handle_firmware_config::telescope::kEnabled,
                                                 ig_handle_firmware_config::telescope::kConfigured,
                                                 ig_handle_firmware_config::telescope::kMinLengthM,
                                                 ig_handle_firmware_config::telescope::kMaxLengthM,
                                                 ig_handle_firmware_config::telescope::kEncoderZeroCountAtMinLength,
                                                 ig_handle_firmware_config::telescope::kEncoderCountSpanToMaxLength};

inline bool geometryReady(const Geometry& geometry) {
  return geometry.enabled && geometry.configured && geometry.max_length_m > geometry.min_length_m &&
         geometry.calibrated_count_span_to_max_length > 0;
}

inline bool lengthFromEncoder(const Geometry& geometry, int32_t encoder_count, float* length_m) {
  if (length_m == 0 || !geometryReady(geometry)) {
    return false;
  }

  int64_t delta = static_cast<int64_t>(encoder_count) - geometry.encoder_zero_count_at_min_length;
  if (delta < 0) {
    delta = 0;
  }
  if (delta > geometry.calibrated_count_span_to_max_length) {
    delta = geometry.calibrated_count_span_to_max_length;
  }

  const float fraction = static_cast<float>(delta) / static_cast<float>(geometry.calibrated_count_span_to_max_length);
  *length_m = geometry.min_length_m + fraction * (geometry.max_length_m - geometry.min_length_m);
  return true;
}

inline bool encoderForLength(const Geometry& geometry, float length_m, int32_t* encoder_count) {
  if (encoder_count == 0 || !geometryReady(geometry) || length_m < geometry.min_length_m ||
      length_m > geometry.max_length_m) {
    return false;
  }

  const float fraction = (length_m - geometry.min_length_m) / (geometry.max_length_m - geometry.min_length_m);
  const float count_delta = fraction * static_cast<float>(geometry.calibrated_count_span_to_max_length);
  const int64_t rounded_delta = static_cast<int64_t>(count_delta + 0.5f);
  const int64_t result = static_cast<int64_t>(geometry.encoder_zero_count_at_min_length) + rounded_delta;
  if (result < INT32_MIN || result > INT32_MAX) {
    return false;
  }
  *encoder_count = static_cast<int32_t>(result);
  return true;
}

}  // namespace telescope
