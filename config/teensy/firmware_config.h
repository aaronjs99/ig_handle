#pragma once

// Firmware-side configuration for the Teensy sketch.
//
// Keep this file synchronized with config/telescope/hardware.yaml. Values
// marked DUMMY are intentionally conservative and do not enable telescope
// motion. The Teensy cannot parse the ROS YAML at compile time.

#include <stdint.h>

#if !defined(ARDUINO_TEENSY41)
#error "ig_handle timing and telescope firmware requires a Teensy 4.1 target"
#endif

// DUMMY/configurable build choice: use the Teensy USB serial transport for
// rosserial unless the firmware build explicitly overrides it.
#ifndef USE_USBCON
#define USE_USBCON
#endif

namespace ig_handle_firmware_config {

static constexpr uint8_t kInvalidPin = 0xff;

namespace timing {

static constexpr uint8_t kCameraCount = 4;
static constexpr uint8_t kLidarCount = 2;
static_assert(kCameraCount == 4, "camera count changes require matching ISR handlers and arrays");
static_assert(kLidarCount == 2, "LiDAR count changes require matching PPS arrays");

// The DS3231 square-wave input is the current reference-edge source. It may
// align relative epochs, but it is not a GNSS-disciplined UTC source and must
// not be represented as one. INT/SQW is open-drain: the assembled board must
// use a verified 3.3 V pull-up or a characterized 3.3 V level shifter. A 5 V
// breakout pull-up may destroy the non-5-V-tolerant Teensy input. Firmware
// publishes only a relative monotonic epoch.
static constexpr uint8_t kReferenceInputPin = 3;
static constexpr bool kReferenceWiringVerified = false;
static constexpr bool kReferencePullupTo3V3Verified = false;
static constexpr bool kReferenceActiveHigh = true;
static constexpr uint32_t kReferenceNominalPeriodUs = 1000000;
static constexpr uint32_t kReferenceMinPeriodUs = 900000;
static constexpr uint32_t kReferenceMaxPeriodUs = 1100000;
static constexpr uint8_t kRequiredStableReferenceEdges = 3;
static constexpr uint32_t kReferenceTimeoutUs = 1500000;

// Forge FG-PGE-50S5C-IP trigger/ExposureActive channels. The checked-in host
// inventory's extra "-C-" spelling must be checked against each camera label;
// firmware follows the currently published Teledyne model family name. All
// board pins are
// deliberately unassigned: the camera OPTOIN threshold is not permission to
// drive it directly from a Teensy. The completed board must provide a verified
// 3.3 V-safe buffer/isolator and a defined return path before these gates are
// enabled. If GPIO power is used, each 8-24 V camera branch also requires
// appropriate protection and fusing. Line0 is the intended FrameStart input;
// Line1 is the intended ExposureActive output. A characterized hardware
// fanout/latch is required for simultaneous edges; sequential MCU writes are
// not claimed to be simultaneous without an oscilloscope measurement.
static constexpr bool kCameraTriggerEnabled = false;
static constexpr bool kCameraFeedbackEnabled = false;
static constexpr bool kCameraWiringVerified = false;
static constexpr bool kCameraFrameStartConfigured = false;
static constexpr bool kCameraExposureActiveConfigured = false;
static constexpr bool kCameraUseHardwareFanout = false;
static constexpr uint8_t kCameraFanoutTriggerPin = kInvalidPin;
static constexpr bool kCameraFanoutTriggerActiveHigh = true;
static constexpr uint8_t kCameraTriggerPins[kCameraCount] = {kInvalidPin, kInvalidPin, kInvalidPin, kInvalidPin};
static constexpr uint8_t kCameraExposurePins[kCameraCount] = {kInvalidPin, kInvalidPin, kInvalidPin, kInvalidPin};
static constexpr bool kCameraTriggerActiveHigh[kCameraCount] = {true, true, true, true};
static constexpr bool kCameraExposureActiveHigh[kCameraCount] = {true, true, true, true};
static const char* const kCameraSources[kCameraCount] = {"diagnostic_relative_epoch_not_utc_forge_f1_exposure_mid",
                                                         "diagnostic_relative_epoch_not_utc_forge_f2_exposure_mid",
                                                         "diagnostic_relative_epoch_not_utc_forge_f3_exposure_mid",
                                                         "diagnostic_relative_epoch_not_utc_forge_f4_exposure_mid"};

// A shared 10 Hz trigger epoch matches the checked-in continuous-acquisition
// target, but host camera triggering remains intentionally unchanged here.
// The firmware scheduler remains inert until every enabled receiver and board
// gate is true. The common pulse width and phase must be accepted by every
// enabled receiver; no cross-device timing precision is claimed until measured.
static constexpr uint32_t kSensorTriggerPeriodUs = 100000;
static constexpr uint32_t kSensorTriggerPulseWidthUs = 100;
static constexpr uint32_t kSensorTriggerPhaseUs = 50000;
static constexpr uint32_t kCameraFeedbackTimeoutUs = 50000;
static constexpr uint32_t kSchedulerTickUs = 25;
static constexpr uint32_t kMaximumTriggerLatenessUs = 100;

// The VLP-16s rotate and acquire continuously. Firmware can align their clocks
// and rotational phase through a common PPS edge; it cannot trigger a scan.
// Each PPS output needs a verified >3.0 V, <5.0 V, >=2 mA driver. Teensy GPIO
// is 3.3 V-only and must not be assumed to satisfy the assembled load without
// the board-level buffer being measured.
static constexpr bool kLidarClockEnabled = false;
static constexpr bool kLidarClockWiringVerified = false;
static constexpr uint8_t kLidarPpsPins[kLidarCount] = {kInvalidPin, kInvalidPin};
// Both polarity values are configurable. The current official VLP-16 contract
// uses the rising edge when a positive PPS pulse is physically commissioned.
static constexpr bool kLidarPpsActiveHigh[kLidarCount] = {true, true};
static constexpr uint32_t kLidarPpsPulseWidthUs = 20000;
// No verified UTC/GNSS sentence source currently exists in this firmware, so
// it intentionally contains no NMEA generator. Add one only with a separate,
// verified UTC and reference-phase contract.

// The encased MTi-30 supports SyncIn and SyncOut, but its selected SyncSettings
// are external device configuration. This optional output is only a timing
// event marker for a continuously sampled IMU; it never controls or reduces
// the estimator sample rate. Both directions remain disabled until the pin,
// voltage, polarity, event function, and MT Manager configuration are verified.
static constexpr bool kImuTriggerEnabled = false;
static constexpr bool kImuFeedbackEnabled = false;
static constexpr bool kImuWiringVerified = false;
static constexpr bool kImuSyncInEventConfigured = false;
static constexpr bool kImuSyncOutEventConfigured = false;
static constexpr uint8_t kImuTriggerPin = kInvalidPin;
static constexpr uint8_t kImuSyncPin = kInvalidPin;
static constexpr bool kImuTriggerActiveHigh = true;
static constexpr bool kImuSyncActiveHigh = true;
static constexpr uint32_t kImuFeedbackTimeoutUs = 50000;
static const char kImuSource[] = "diagnostic_relative_epoch_not_utc_xsens_mti30_syncout";

static constexpr uint32_t kStatusPublishPeriodMs = 1000;

// Fail at build time for the most dangerous partial edits. Full timing and
// cross-owner pin/capability validation runs before either runtime initializes.
static_assert(!kCameraTriggerEnabled || kCameraWiringVerified, "camera trigger requires verified wiring");
static_assert(!kCameraTriggerEnabled || kCameraFrameStartConfigured,
              "camera trigger requires verified FrameStart Line0 settings");
static_assert(!kCameraTriggerEnabled || kCameraFeedbackEnabled, "camera trigger requires ExposureActive feedback");
static_assert(!kCameraTriggerEnabled || kCameraExposureActiveConfigured,
              "camera trigger requires verified Line1 ExposureActive settings");
static_assert(!kCameraTriggerEnabled || (kCameraFeedbackTimeoutUs > kSensorTriggerPulseWidthUs &&
                                         kCameraFeedbackTimeoutUs < kSensorTriggerPeriodUs),
              "camera feedback timeout must fit inside one trigger cycle");
static_assert(!kCameraFeedbackEnabled || kCameraWiringVerified, "camera feedback requires verified wiring");
static_assert(!kCameraUseHardwareFanout || kCameraFanoutTriggerPin != kInvalidPin,
              "camera fanout pin must be assigned");
static_assert(!kCameraTriggerEnabled || kCameraUseHardwareFanout ||
                  (kCameraTriggerPins[0] != kInvalidPin && kCameraTriggerPins[1] != kInvalidPin &&
                   kCameraTriggerPins[2] != kInvalidPin && kCameraTriggerPins[3] != kInvalidPin),
              "all camera trigger pins must be assigned");
static_assert(!kCameraFeedbackEnabled ||
                  (kCameraExposurePins[0] != kInvalidPin && kCameraExposurePins[1] != kInvalidPin &&
                   kCameraExposurePins[2] != kInvalidPin && kCameraExposurePins[3] != kInvalidPin),
              "all camera feedback pins must be assigned");
static_assert(!kImuTriggerEnabled || (kImuWiringVerified && kImuFeedbackEnabled && kImuSyncInEventConfigured &&
                                      kImuSyncOutEventConfigured),
              "MTi event marker requires wiring, feedback, and SyncSettings");
static_assert(!kImuTriggerEnabled || kImuTriggerPin != kInvalidPin, "MTi trigger pin must be assigned");
static_assert(!kImuTriggerEnabled || (kImuFeedbackTimeoutUs > 0 && kImuFeedbackTimeoutUs < kSensorTriggerPeriodUs),
              "MTi feedback timeout must fit inside one trigger cycle");
static_assert(!kImuFeedbackEnabled || (kImuWiringVerified && kImuSyncPin != kInvalidPin),
              "MTi feedback requires verified wiring and an assigned pin");
static_assert(!kLidarClockEnabled || kLidarClockWiringVerified, "VLP-16 clock output requires verified wiring");
static_assert(!kLidarClockEnabled || (kLidarPpsPins[0] != kInvalidPin && kLidarPpsPins[1] != kInvalidPin),
              "both VLP-16 PPS pins must be assigned");
static_assert(!(kCameraTriggerEnabled || kImuTriggerEnabled || kLidarClockEnabled) ||
                  (kReferenceWiringVerified && kReferencePullupTo3V3Verified && kReferenceInputPin != kInvalidPin),
              "timed outputs require a verified 3.3 V reference input");

}  // namespace timing

// Existing rosserial topic names.
static const char kPpsTimeTopic[] = "/pps/time";
static const char kCameraTimeTopic[] = "/cam/time";
static const char kImuTimeTopic[] = "/imu/time";
static const char kTimingStatusTopic[] = "/timing/status";

namespace telescope {

// DUMMY: motion remains disabled until geometry and wiring are measured.
static constexpr bool kEnabled = false;
static constexpr bool kConfigured = false;
static constexpr bool kWiringVerified = false;
static constexpr float kMinLengthM = 0.0f;                       // DUMMY: measure.
static constexpr float kMaxLengthM = 0.0f;                       // DUMMY: measure.
static constexpr int32_t kEncoderZeroCountAtMinLength = 0;       // DUMMY: home.
static constexpr int32_t kEncoderCountSpanToMaxLength = 0;       // DUMMY: calibrate.
static constexpr bool kEncoderCountIncreasesOnExtension = true;  // DUMMY: verify.
static constexpr int8_t kMotorDutySignForExtension = 1;          // DUMMY: verify.

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
static constexpr bool kMinLimitPresent = true;
// The reported mechanism has one SPDT retraction switch. An optional maximum
// SPDT switch is supported, but it is not invented in the active contract.
static constexpr bool kMaxLimitPresent = false;
static constexpr uint8_t kMaxLimitNoPin = kInvalidPin;
static constexpr uint8_t kMaxLimitNcPin = kInvalidPin;

// Compile-time mirror of config/runtime_surface.yaml. The telescope is an
// actuator; its carried sonar remains under the separate /sensors/sonar tree.
static const char kCommandLengthTopic[] = "/actuators/telescope/command/length";
static const char kStateLengthTopic[] = "/actuators/telescope/state/length";
static const char kStateStatusTopic[] = "/actuators/telescope/state/status";
static const char kStateMotorCurrentTopic[] = "/actuators/telescope/state/motor_current";

// Compile-time mirrors of telescope/hardware.yaml control values.
static constexpr uint32_t kPwmFrequencyHz = 20000;
static constexpr uint32_t kControlUpdatePeriodMs = 10;
static constexpr uint32_t kTelemetryPublishPeriodMs = 100;
static constexpr uint32_t kLimitDebounceMs = 25;
static constexpr uint32_t kDirectionReversalBlankMs = 20;
static constexpr uint32_t kCommandTimeoutMs = 500;
static constexpr uint32_t kStallTimeoutMs = 1000;
static constexpr uint32_t kHomingTimeoutMs = 30000;
static constexpr int8_t kHomingDirection = -1;
static constexpr float kHomingDutyFraction = 0.10f;
static constexpr float kMaxDutyFraction = 0.25f;
static constexpr float kMaxDutyAccelerationPerSec = 0.50f;
static constexpr float kPositionKp = 1.0f;
// For an installed SPDT endpoint, "redundant" means its NC and NO contacts
// must be complementary. It does not imply that both endpoints are installed.
static constexpr bool kRequireRedundantLimitAgreement = true;

// DUMMY: no current feedback is treated as calibrated until this is measured
// against a trusted meter. Motion remains gated off while this is false.
static constexpr bool kCurrentSenseConfigured = false;
static constexpr float kMotorCurrentAPerAdcCount = 0.0f;
static constexpr float kMotorCurrentZeroAdcCount = 0.0f;
static constexpr float kMaxMotorCurrentA = 0.0f;  // DUMMY: set after calibration.

}  // namespace telescope
}  // namespace ig_handle_firmware_config
