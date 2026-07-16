#pragma once

// Firmware-side configuration for the Teensy sketch.
//
// Keep this file synchronized with config/telescope/hardware.yaml. Values
// marked DUMMY are intentionally conservative and do not enable telescope
// motion. The Teensy cannot parse the ROS YAML at compile time.

#include <stdint.h>

// DUMMY/configurable build choice: use the Teensy USB serial transport for
// rosserial unless the firmware build explicitly overrides it.
#ifndef USE_USBCON
#define USE_USBCON
#endif

// DUMMY: verify the transceiver wiring and framing before field use.
#define IG_HANDLE_GPSERIAL_FORMAT SERIAL_8N1_TXINV

namespace ig_handle_firmware_config {

// Serial and timing surfaces.
#define IG_HANDLE_GPSERIAL Serial1 // DUMMY: verify the assembled serial wiring.
static constexpr uint32_t kGpsBaudRate = 9600;
static constexpr uint32_t kPpsPulseWidthMs = 20;
static constexpr uint32_t kPpsNmeaMinSeparationMs = 55;
static constexpr int32_t kTimeZoneOffsetHours = -7; // DUMMY: operator timezone.

// DUMMY: current legacy GPRMC payload values; replace with a real navigation
// source when the upstream sensor contract is changed.
static const char kNmeaPrefix[] = "GPRMC,";
static const char kNmeaStatus[] = "A";
static const char kNmeaLatitude[] = "4365.107,N";
static const char kNmeaLongitude[] = "79347.702,E";
static const char kNmeaSpeedKnots[] = "022.4";
static const char kNmeaCourseDegrees[] = "084.4";
static const char kNmeaMagneticVariation[] = "003.1,W";

// Existing sensor synchronization pins.
static constexpr uint8_t kPpsOutPin = 2;
static constexpr uint8_t kPpsInPin = 3;
static constexpr uint8_t kCameraTriggerOutPin = 10;
static constexpr uint8_t kCameraOpenInPin = 29;
static constexpr uint8_t kCameraCloseInPin = 30;
static constexpr uint8_t kImuTriggerOutPin = 7;
static constexpr uint8_t kImuSyncInPin = 8;
static constexpr float kCameraTriggerFrequencyHz = 20.0f;
static constexpr uint8_t kCameraTriggerDuty = 5;

// Existing rosserial topic names.
static const char kPpsTimeTopic[] = "/pps/time";
static const char kCameraTimeTopic[] = "/cam/time";
static const char kImuTimeTopic[] = "/imu/time";

namespace telescope {

// DUMMY: motion remains disabled until geometry and wiring are measured.
static constexpr bool kEnabled = false;
static constexpr bool kConfigured = false;
static constexpr bool kWiringVerified = false;
static constexpr int8_t kDirectionSign = 1; // DUMMY: verify polarity.
static constexpr float kMinLengthM = 0.0f; // DUMMY: measure.
static constexpr float kMaxLengthM = 0.0f; // DUMMY: measure.
static constexpr float kLinearTravelPerMotorRevolutionM = 0.0f; // DUMMY: measure.
static constexpr int32_t kEncoderZeroCountAtMinLength = 0; // DUMMY: home.
static constexpr int32_t kEncoderCountSpanToMaxLength = 0; // DUMMY: calibrate.

// DUMMY: provisional pins; replace only after wiring is verified.
static constexpr uint8_t kMotorRightPwmPin = 22;
static constexpr uint8_t kMotorLeftPwmPin = 23;
static constexpr uint8_t kMotorRightEnablePin = 24;
static constexpr uint8_t kMotorLeftEnablePin = 25;
static constexpr uint8_t kMotorRightCurrentSensePin = 38;
static constexpr uint8_t kMotorLeftCurrentSensePin = 39;
static constexpr uint8_t kEncoderPhaseAPin = 26;
static constexpr uint8_t kEncoderPhaseBPin = 27;
static constexpr uint8_t kMinLimitNoPin = 31;
static constexpr uint8_t kMinLimitNcPin = 32;
static constexpr uint8_t kMaxLimitNoPin = 33;
static constexpr uint8_t kMaxLimitNcPin = 34;

}  // namespace telescope
}  // namespace ig_handle_firmware_config
