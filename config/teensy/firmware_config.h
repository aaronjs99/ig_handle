#pragma once

// Firmware-side configuration for the Teensy sketch.
//
// Keep this file synchronized with config/telescope/hardware.yaml. Unmeasured
// values remain fail-closed and do not enable telescope motion. The Teensy
// cannot parse the ROS YAML at compile time.

#include <stdint.h>

#if !defined(ARDUINO_TEENSY41)
#error "ig_handle timing and telescope firmware requires a Teensy 4.1 target"
#endif

// Use the Teensy USB serial transport for rosserial unless a reviewed build
// explicitly overrides it.
#ifndef USE_USBCON
#define USE_USBCON
#endif

namespace ig_handle_firmware_config {

static constexpr uint8_t kInvalidPin = 0xff;

namespace timing {

static constexpr uint8_t kCameraCount = 4;
static constexpr uint8_t kLidarCount = 2;
static_assert(kCameraCount == 4, "camera count changes require matching ISR handlers and arrays");
static_assert(kLidarCount == 2, "LiDAR count changes require matching hardware fanout and harness documentation");

// The falling edge of the DS3231 open-drain SQW output triggers the one-shot.
// V6 captures the resulting active-high PPS_MASTER rising edge on D38, the same
// edge physically fanned out to both VLP-16s. Firmware never synthesizes PPS.
// The datasheet places SQW's high transition about 500 ms after seconds data
// transfer, so the following falling edge supports association with the next
// divider boundary. Exact assembled phase still requires scoping SQW against
// RTC register rollover and PPS_MASTER. The RTC is a local epoch, not GNSS UTC.
static constexpr uint8_t kReferenceInputPin = 38;
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
// board pins are assigned to the V6 buffered interfaces, but the gates remain
// false until the assembled paths are verified. The camera OPTOIN threshold is
// not permission to drive it directly from a Teensy. The completed board must
// provide a verified buffer and defined opto return path before these gates are
// enabled. V6 does not power the PoE cameras. Line0 is the intended FrameStart input;
// Line1 is the intended ExposureActive output. A characterized hardware
// fanout/latch is required for simultaneous edges; sequential MCU writes are
// not claimed to be simultaneous without an oscilloscope measurement.
static constexpr bool kCameraTriggerEnabled = false;
static constexpr bool kCameraFeedbackEnabled = false;
static constexpr bool kCameraWiringVerified = false;
static constexpr bool kCameraFrameStartConfigured = false;
static constexpr bool kCameraExposureActiveConfigured = false;
static constexpr bool kCameraUseHardwareFanout = true;
static constexpr uint8_t kCameraFanoutTriggerPin = 34;
static constexpr bool kCameraFanoutTriggerActiveHigh = true;
static constexpr uint8_t kCameraTriggerPins[kCameraCount] = {kInvalidPin, kInvalidPin, kInvalidPin, kInvalidPin};
static constexpr uint8_t kCameraExposurePins[kCameraCount] = {17, 16, 15, 14};
static constexpr bool kCameraTriggerActiveHigh[kCameraCount] = {true, true, true, true};
// The exact SN74LV14APWR feedback receiver inverts active-high OPTOOUT.
static constexpr bool kCameraExposureActiveHigh[kCameraCount] = {false, false, false, false};
static const char* const kCameraSources[kCameraCount] = {"diagnostic_relative_epoch_not_utc_forge_f1_exposure_mid",
                                                         "diagnostic_relative_epoch_not_utc_forge_f2_exposure_mid",
                                                         "diagnostic_relative_epoch_not_utc_forge_f3_exposure_mid",
                                                         "diagnostic_relative_epoch_not_utc_forge_f4_exposure_mid"};

// A shared 5 Hz trigger epoch matches the checked-in Forge acquisition-rate
// target, but host camera triggering remains intentionally unchanged here.
// The firmware scheduler remains inert until every enabled receiver and board
// gate is true. The common pulse width and phase must be accepted by every
// enabled receiver; no cross-device timing precision is claimed until measured.
static constexpr uint32_t kSensorTriggerPeriodUs = 200000;
static constexpr uint32_t kSensorTriggerPulseWidthUs = 100;
static constexpr uint32_t kSensorTriggerPhaseUs = 50000;
static constexpr uint32_t kCameraFeedbackTimeoutUs = 50000;
static constexpr uint32_t kSchedulerTickUs = 25;
static constexpr uint32_t kMaximumTriggerLatenessUs = 100;

// The VLP-16s rotate and acquire continuously; PPS aligns their local clocks
// but does not trigger a scan. V6 hardware shapes and fans out PPS. There is no
// per-LiDAR hardware enable: whenever the common RTC timing source is enabled
// for any receiver, both VLP PPS branches are physically driven while
// FIELD_VALID is high. kLidarTimingEnabled controls LiDAR-specific firmware
// capture/diagnostics, not the physical fanout. Firmware may also send one
// common RTC-backed GPRMC stream over Serial1 after the PPS pulse. A status-V
// sentence supplies date/time but makes no GNSS-position claim. Each VLP must
// be read back with both its PPS and GPS "Require GPS Receiver Valid"
// qualifiers disabled; firmware never fabricates status A or position to
// bypass that device-side requirement.
static constexpr bool kLidarTimingEnabled = false;
static constexpr bool kLidarTimingWiringVerified = false;
static constexpr bool kLidarNmeaEnabled = false;
static constexpr bool kLidarNmeaWiringVerified = false;
static constexpr bool kLidarNmeaPolarityVerified = false;
// Required only for RTC-backed NMEA association: scope this exact DS3231
// module's register-rollover-to-SQW-to-PPS_MASTER phase before enabling NMEA.
static constexpr bool kRtcPpsPhaseVerified = false;
static constexpr bool kLidarQualifierSettingsReadBack[kLidarCount] = {false, false};
static constexpr bool kLidarPpsRequireGpsReceiverValidDisabled[kLidarCount] = {false, false};
static constexpr bool kLidarGpsRequireGpsReceiverValidDisabled[kLidarCount] = {false, false};
static constexpr uint8_t kLidarNmeaTxPin = 1;
static constexpr uint32_t kLidarNmeaBaud = 9600;
// Schedule from the shaped PPS leading edge. With the nominal 10 ms one-shot,
// 100 ms leaves about 90 ms between the PPS trailing edge and NMEA start,
// comfortably above the VLP Rev-F 50 ms minimum.
static constexpr uint32_t kLidarNmeaDelayAfterPpsUs = 100000;
static constexpr uint32_t kLidarNmeaLatestStartAfterPpsUs = 200000;
// Stop enqueueing well before the VLP's required final 300 ms quiet window.
// Even a conservative 80-byte, 10-bit-per-byte sentence started at this
// deadline finishes on wire before the minimum qualified period's final
// 300 ms quiet window at 9600 baud.
static constexpr uint32_t kLidarNmeaLatestEnqueueAfterPpsUs = 500000;
static constexpr uint32_t kLidarNmeaMaximumSentenceBytes = 80;
static constexpr uint32_t kLidarNmeaWorstCaseWireUs =
    (kLidarNmeaMaximumSentenceBytes * 10UL * 1000000UL + kLidarNmeaBaud - 1UL) / kLidarNmeaBaud;
static constexpr bool kLidarNmeaAppliesToNextPps = true;
// The VLP-16 GPS serial input uses inverted UART polarity. V6 performs that
// inversion in the Teensy UART peripheral, avoiding another logic package.
static constexpr bool kLidarNmeaTxInverted = true;
static_assert(kLidarNmeaTxPin == 1, "V6 routes the VLP NMEA source through Teensy Serial1 TX on D1");
static_assert(kLidarNmeaBaud == 9600, "VLP Rev-F GPS serial timing assumes 9600 baud");
static_assert(kLidarNmeaAppliesToNextPps,
              "V6 GPRMC sent between PPS edges must carry the next PPS second");
static_assert(!kLidarNmeaEnabled ||
                  (kLidarQualifierSettingsReadBack[0] && kLidarQualifierSettingsReadBack[1] &&
                   kLidarPpsRequireGpsReceiverValidDisabled[0] &&
                   kLidarPpsRequireGpsReceiverValidDisabled[1] &&
                   kLidarGpsRequireGpsReceiverValidDisabled[0] &&
                   kLidarGpsRequireGpsReceiverValidDisabled[1]),
              "both VLPs must be read back with PPS/GPS Receiver Valid requirements disabled before RTC-only status-V NMEA is enabled");
static_assert(kLidarNmeaDelayAfterPpsUs >= 100000,
              "V6 preserves margin beyond the VLP 50 ms PPS-trailing-edge-to-NMEA minimum");
static_assert(kLidarNmeaLatestStartAfterPpsUs > kLidarNmeaDelayAfterPpsUs &&
                  kLidarNmeaLatestStartAfterPpsUs < 700000,
              "VLP NMEA start window must preserve the final 300 ms before the next PPS");
static_assert(kLidarNmeaLatestEnqueueAfterPpsUs > kLidarNmeaLatestStartAfterPpsUs &&
                  kLidarNmeaLatestEnqueueAfterPpsUs + kLidarNmeaWorstCaseWireUs <
                      kReferenceMinPeriodUs - 300000UL,
              "VLP NMEA enqueue deadline plus worst-case wire time must preserve the final 300 ms at the minimum qualified reference period");

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
static constexpr uint8_t kImuTriggerPin = 30;
static constexpr uint8_t kImuSyncPin = 27;
static constexpr bool kImuTriggerActiveHigh = true;
// The exact SN74LV14APWR inverts the physical active-high SyncOut edge.
static constexpr bool kImuSyncActiveHigh = false;
static constexpr uint32_t kImuFeedbackTimeoutUs = 50000;
static const char kImuSource[] = "diagnostic_relative_epoch_not_utc_xsens_mti30_syncout";

// The RTC/one-shot is a shared physical source. Enabling any timed receiver
// therefore also drives both VLP PPS connectors; both LiDAR branches must be
// commissioned even when only camera or MTi timing is requested in firmware.
static constexpr bool kCommonTimingRequested =
    kCameraTriggerEnabled || kImuTriggerEnabled || kLidarTimingEnabled;

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
static_assert(!kCommonTimingRequested || kLidarTimingWiringVerified,
              "any common timing enable physically drives both VLP PPS branches; verify both before enabling timing");
static_assert(!kLidarNmeaEnabled ||
                  (kLidarTimingEnabled && kLidarNmeaWiringVerified && kLidarNmeaPolarityVerified &&
                   kRtcPpsPhaseVerified && kLidarNmeaTxPin != kInvalidPin),
              "VLP-16 NMEA requires commissioned timing, wiring, polarity, and measured RTC-to-PPS phase");
static_assert(!kCommonTimingRequested ||
                  (kReferenceWiringVerified && kReferencePullupTo3V3Verified && kReferenceInputPin != kInvalidPin),
              "timed outputs require a verified 3.3 V reference input");

}  // namespace timing

// Existing rosserial topic names.
static const char kPpsTimeTopic[] = "/pps/time";
static const char kCameraTimeTopic[] = "/cam/time";
static const char kImuTimeTopic[] = "/imu/time";
static const char kTimingStatusTopic[] = "/timing/status";

namespace telescope {

// Motion remains disabled until geometry, polarity, field power, and wiring
// are measured and commissioned together.
static constexpr bool kEnabled = false;
static constexpr bool kConfigured = false;
static constexpr bool kWiringVerified = false;
// This gate is independent of firmware and may become true only after a
// normally-closed hard E-stop in the 12 V motor-power path is installed and
// physically tested to remove motor power.
static constexpr bool kHardEstopVerified = false;
static_assert(!kEnabled || kHardEstopVerified,
              "telescope motion enable requires a tested independent hard E-stop in the 12 V motor path");
static constexpr float kMinLengthM = 0.0f;                       // Measurement required.
static constexpr float kMaxLengthM = 0.0f;                       // Measurement required.
static constexpr int32_t kEncoderZeroCountAtMinLength = 0;       // Establish during explicit homing.
static constexpr int32_t kEncoderCountSpanToMaxLength = 0;       // Calibrate over measured travel.
static constexpr bool kEncoderCountIncreasesOnExtension = true;  // Verify encoder sign.
static constexpr int8_t kMotorDutySignForExtension = 1;          // Verify motor polarity.

// Fixed V6 Teensy pin contract. Wiring verification gates operation; it does
// not change these assignments.
static constexpr uint8_t kMotorRightPwmPin = 5;
static constexpr uint8_t kMotorLeftPwmPin = 4;
static constexpr uint8_t kMotorEnablePin = 23;
static constexpr uint8_t kEncoderPhaseAPin = 12;
static constexpr uint8_t kEncoderPhaseBPin = 11;
static constexpr uint8_t kMinLimitTripPin = 22;
// V6's exact SN74LV14APWR makes an open/tripped NC loop active-low at the MCU.
static constexpr bool kMinLimitTrippedActiveHigh = false;
static constexpr bool kMinLimitPresent = true;
// TPS3897ADRYT supervises EXT5_FIELD through an 84.5 kOhm / 10.0 kOhm divider.
// Its open-drain output and a 10 kOhm core-3.3-V pull-up create FIELD_VALID on
// D31: HIGH means qualified field power and LOW means invalid/off. Hardware
// FIELD_VALID gating disables U3/U6 independently. As a secondary defense, the
// falling edge asynchronously removes the MCU's shared BTS enable and the
// 10 ms control loop clears requests and zeroes both PWMs.
// Motion remains disabled until the assembled supervisor and polarity have
// been verified.
static constexpr uint8_t kFieldValidPin = 31;
static constexpr bool kFieldValidSupervisorVerified = false;
static constexpr bool kFieldValidActiveHigh = true;

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
// The current IBT-2 clone has no commissioned R_IS/L_IS contract. The retained
// motor-current ROS topic therefore publishes NaN rather than invented data.

}  // namespace telescope
}  // namespace ig_handle_firmware_config
