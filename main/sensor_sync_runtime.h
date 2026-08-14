#pragma once

// Teensy I/O adapter for the reference-qualified trigger scheduler.
//
// The runtime owns only synchronization GPIO. Camera, LiDAR, and IMU payloads
// remain continuous host-side acquisitions until their respective drivers are
// explicitly configured for hardware synchronization.

#include <Arduino.h>

#include "firmware_config.h"
#include "sensor_sync.h"

namespace sensor_sync {

struct CaptureEvent {
  uint8_t channel;
  uint32_t sequence;
  uint32_t sec;
  uint32_t nsec;
};

class Runtime {
public:
  Runtime()
      : scheduler_(schedulerConfig()),
        io_configuration_valid_(false),
        lidar_pulse_active_(false),
        lidar_pulse_release_us_(0),
        imu_feedback_pending_(false),
        imu_feedback_deadline_us_(0),
        imu_event_ready_(false),
        imu_event_sec_(0),
        imu_event_nsec_(0),
        imu_sequence_(0),
        imu_dropped_(0),
        camera_invalid_edges_(0) {
    for (uint8_t i = 0; i < cameraCount(); ++i) {
      camera_open_valid_[i] = false;
      camera_open_sec_[i] = 0;
      camera_open_nsec_[i] = 0;
      camera_event_ready_[i] = false;
      camera_event_sec_[i] = 0;
      camera_event_nsec_[i] = 0;
      camera_sequence_[i] = 0;
      camera_dropped_[i] = 0;
    }
  }

  bool begin() {
    // Best-effort software safe-init happens before validation. The helper
    // touches only assigned pins that have exactly one configured use.
    // External pulls/interlocks remain the independent boot/reset safety layer.
    configureAssignedOutputsInactive();
    io_configuration_valid_ = ioConfigurationValid();
    if (!io_configuration_valid_) {
      scheduler_.forceFault(Fault::kInvalidConfiguration);
      return false;
    }

    configureInputs();
    return scheduler_.begin();
  }

  void onReferenceEdge(uint32_t now_us) {
    scheduler_.onReferenceEdge(now_us);
    using namespace ig_handle_firmware_config::timing;
    if (scheduler_.state() == State::kRunning && kLidarClockEnabled && kLidarClockWiringVerified) {
      setLidarPps(true);
      lidar_pulse_active_ = true;
      lidar_pulse_release_us_ = now_us + kLidarPpsPulseWidthUs;
    }
  }

  void onTimerTick(uint32_t now_us) {
    using namespace ig_handle_firmware_config::timing;

    if (lidar_pulse_active_ && reached(now_us, lidar_pulse_release_us_)) {
      setLidarPps(false);
      lidar_pulse_active_ = false;
    }

    const Actions actions = scheduler_.update(now_us);
    if (actions.release_trigger) {
      setScheduledTrigger(false);
    }
    if (actions.assert_trigger) {
      setScheduledTrigger(true);
      if (kCameraTriggerEnabled && kCameraFeedbackEnabled) {
        camera_feedback_.arm(now_us, kCameraFeedbackTimeoutUs);
      }
      if (kImuTriggerEnabled && kImuFeedbackEnabled) {
        imu_feedback_pending_ = true;
        imu_feedback_deadline_us_ = now_us + kImuFeedbackTimeoutUs;
      }
    }

    if (camera_feedback_.expired(now_us)) {
      camera_feedback_.clear();
      setScheduledTrigger(false);
      scheduler_.forceFault(Fault::kFeedbackTimeout);
    }
    if (imu_feedback_pending_ && reached(now_us, imu_feedback_deadline_us_)) {
      imu_feedback_pending_ = false;
      setScheduledTrigger(false);
      scheduler_.forceFault(Fault::kFeedbackTimeout);
    }

    if (scheduler_.state() == State::kFault) {
      setScheduledTrigger(false);
      setLidarPps(false);
      lidar_pulse_active_ = false;
      clearCaptureState();
    }
  }

  bool timerRequired() const {
    using namespace ig_handle_firmware_config::timing;
    return (kCameraTriggerEnabled || kImuTriggerEnabled || kLidarClockEnabled) && io_configuration_valid_;
  }

  bool cameraFeedbackConfigured(uint8_t channel) const {
    using namespace ig_handle_firmware_config;
    using namespace ig_handle_firmware_config::timing;
    return channel < kCameraCount && kCameraFeedbackEnabled && kCameraWiringVerified &&
           kCameraExposureActiveConfigured && kCameraExposurePins[channel] != kInvalidPin;
  }

  bool cameraInputActive(uint8_t channel) const {
    using namespace ig_handle_firmware_config::timing;
    if (!cameraFeedbackConfigured(channel)) {
      return false;
    }
    const bool high = digitalRead(kCameraExposurePins[channel]) == HIGH;
    return high == kCameraExposureActiveHigh[channel];
  }

  void onCameraExposureEdge(uint8_t channel, bool active, uint32_t now_us, uint32_t sec, uint32_t nsec) {
    if (!cameraFeedbackConfigured(channel) || scheduler_.state() != State::kRunning || sec == 0) {
      return;
    }
    if (camera_feedback_.expired(now_us)) {
      camera_feedback_.clear();
      scheduler_.forceFault(Fault::kFeedbackTimeout);
      clearCaptureState();
      return;
    }
    const ExposureEdgeResult result = camera_feedback_.onEdge(channel, active);
    if (result == ExposureEdgeResult::kInvalid) {
      ++camera_invalid_edges_;
      return;
    }
    if (result == ExposureEdgeResult::kOpened) {
      camera_open_sec_[channel] = sec;
      camera_open_nsec_[channel] = nsec;
      camera_open_valid_[channel] = true;
      return;
    }

    if (result != ExposureEdgeResult::kCompleted || !camera_open_valid_[channel]) {
      ++camera_invalid_edges_;
      return;
    }
    camera_open_valid_[channel] = false;
    const uint64_t open_ns =
        static_cast<uint64_t>(camera_open_sec_[channel]) * 1000000000ULL + camera_open_nsec_[channel];
    const uint64_t close_ns = static_cast<uint64_t>(sec) * 1000000000ULL + nsec;
    if (close_ns < open_ns) {
      ++camera_invalid_edges_;
      return;
    }
    const uint64_t midpoint_ns = open_ns + (close_ns - open_ns) / 2ULL;
    if (camera_event_ready_[channel]) {
      ++camera_dropped_[channel];
    }
    camera_event_sec_[channel] = static_cast<uint32_t>(midpoint_ns / 1000000000ULL);
    camera_event_nsec_[channel] = static_cast<uint32_t>(midpoint_ns % 1000000000ULL);
    ++camera_sequence_[channel];
    camera_event_ready_[channel] = true;
    camera_feedback_.complete(channel);
  }

  bool takeCameraEvent(uint8_t channel, CaptureEvent* event) {
    if (event == 0 || channel >= cameraCount()) {
      return false;
    }
    noInterrupts();
    const bool ready = camera_event_ready_[channel];
    if (ready) {
      event->channel = channel;
      event->sequence = camera_sequence_[channel];
      event->sec = camera_event_sec_[channel];
      event->nsec = camera_event_nsec_[channel];
      camera_event_ready_[channel] = false;
    }
    interrupts();
    return ready;
  }

  bool imuFeedbackConfigured() const {
    using namespace ig_handle_firmware_config;
    using namespace ig_handle_firmware_config::timing;
    return kImuFeedbackEnabled && kImuWiringVerified && kImuSyncOutEventConfigured && kImuSyncPin != kInvalidPin;
  }

  bool imuInputActive() const {
    using namespace ig_handle_firmware_config::timing;
    if (!imuFeedbackConfigured()) {
      return false;
    }
    const bool high = digitalRead(kImuSyncPin) == HIGH;
    return high == kImuSyncActiveHigh;
  }

  void onImuSyncEdge(uint32_t now_us, uint32_t sec, uint32_t nsec) {
    if (!imuFeedbackConfigured() || scheduler_.state() != State::kRunning || sec == 0) {
      return;
    }
    if (!imu_feedback_pending_ || reached(now_us, imu_feedback_deadline_us_)) {
      imu_feedback_pending_ = false;
      scheduler_.forceFault(Fault::kFeedbackTimeout);
      clearCaptureState();
      return;
    }
    if (imu_event_ready_) {
      ++imu_dropped_;
    }
    imu_event_sec_ = sec;
    imu_event_nsec_ = nsec;
    ++imu_sequence_;
    imu_event_ready_ = true;
    imu_feedback_pending_ = false;
  }

  bool takeImuEvent(CaptureEvent* event) {
    if (event == 0) {
      return false;
    }
    noInterrupts();
    const bool ready = imu_event_ready_;
    if (ready) {
      event->channel = 0;
      event->sequence = imu_sequence_;
      event->sec = imu_event_sec_;
      event->nsec = imu_event_nsec_;
      imu_event_ready_ = false;
    }
    interrupts();
    return ready;
  }

  State state() const { return scheduler_.state(); }
  Fault fault() const { return scheduler_.fault(); }
  void forceFault(Fault fault) {
    scheduler_.forceFault(fault);
    setScheduledTrigger(false);
    setLidarPps(false);
    lidar_pulse_active_ = false;
    clearCaptureState();
  }
  uint32_t triggerCount() const { return scheduler_.triggerCount(); }
  uint8_t stableReferenceEdges() const { return scheduler_.stableReferenceEdges(); }
  uint32_t cameraDropped(uint8_t channel) const { return channel < cameraCount() ? camera_dropped_[channel] : 0; }
  uint32_t imuDropped() const { return imu_dropped_; }
  uint32_t cameraInvalidEdges() const { return camera_invalid_edges_; }

  static constexpr uint8_t cameraCount() { return ig_handle_firmware_config::timing::kCameraCount; }

private:
  static Config schedulerConfig() {
    using namespace ig_handle_firmware_config::timing;
    const bool enabled = kCameraTriggerEnabled || kImuTriggerEnabled || kLidarClockEnabled;
    const bool trigger_enabled = kCameraTriggerEnabled || kImuTriggerEnabled;
    const Config config = {
        enabled,
        trigger_enabled,
        enabled,
        kReferenceWiringVerified && kReferencePullupTo3V3Verified &&
            (!kCameraTriggerEnabled || (kCameraWiringVerified && kCameraFrameStartConfigured &&
                                        kCameraFeedbackEnabled && kCameraExposureActiveConfigured)) &&
            (!kImuTriggerEnabled || (kImuWiringVerified && kImuSyncInEventConfigured && kImuSyncOutEventConfigured)),
        kSensorTriggerPeriodUs,
        kSensorTriggerPulseWidthUs,
        kSensorTriggerPhaseUs,
        kMaximumTriggerLatenessUs,
        kReferenceNominalPeriodUs,
        kReferenceMinPeriodUs,
        kReferenceMaxPeriodUs,
        kReferenceTimeoutUs,
        kRequiredStableReferenceEdges};
    return config;
  }

  static bool reached(uint32_t now_us, uint32_t deadline_us) { return static_cast<int32_t>(now_us - deadline_us) >= 0; }

  static bool pinAssigned(uint8_t pin) { return pin != ig_handle_firmware_config::kInvalidPin; }

  static uint8_t pinUseCount(uint8_t target) {
    using namespace ig_handle_firmware_config::timing;
    if (!pinAssigned(target)) {
      return 0;
    }
    uint8_t count = 0;
    const bool any_timing = kCameraTriggerEnabled || kCameraFeedbackEnabled || kImuTriggerEnabled ||
                            kImuFeedbackEnabled || kLidarClockEnabled;
    count += any_timing && kReferenceInputPin == target ? 1 : 0;
    if (kCameraTriggerEnabled) {
      if (kCameraUseHardwareFanout) {
        count += kCameraFanoutTriggerPin == target ? 1 : 0;
      } else {
        for (uint8_t i = 0; i < kCameraCount; ++i) {
          count += kCameraTriggerPins[i] == target ? 1 : 0;
        }
      }
    }
    if (kCameraFeedbackEnabled) {
      for (uint8_t i = 0; i < kCameraCount; ++i) {
        count += kCameraExposurePins[i] == target ? 1 : 0;
      }
    }
    count += kImuTriggerEnabled && kImuTriggerPin == target ? 1 : 0;
    count += kImuFeedbackEnabled && kImuSyncPin == target ? 1 : 0;
    if (kLidarClockEnabled) {
      for (uint8_t i = 0; i < kLidarCount; ++i) {
        count += kLidarPpsPins[i] == target ? 1 : 0;
      }
    }
    return count;
  }

  static bool safeAssignedOutput(uint8_t pin) { return pinAssigned(pin) && pinUseCount(pin) == 1; }

  static bool addUniquePin(uint8_t pin, uint8_t* pins, uint8_t* pin_count) {
    if (!pinAssigned(pin) || pins == 0 || pin_count == 0) {
      return false;
    }
    for (uint8_t i = 0; i < *pin_count; ++i) {
      if (pins[i] == pin) {
        return false;
      }
    }
    pins[*pin_count] = pin;
    ++(*pin_count);
    return true;
  }

  static bool ioConfigurationValid() {
    using namespace ig_handle_firmware_config;
    using namespace ig_handle_firmware_config::timing;

    const bool any_timed_output = kCameraTriggerEnabled || kImuTriggerEnabled || kLidarClockEnabled;
    if (any_timed_output &&
        (!kReferenceWiringVerified || !kReferencePullupTo3V3Verified || !pinAssigned(kReferenceInputPin))) {
      return false;
    }
    if (kCameraTriggerEnabled &&
        (!kCameraWiringVerified || !kCameraFrameStartConfigured || !kCameraFeedbackEnabled ||
         !kCameraExposureActiveConfigured ||
         !feedbackWindowValid(true, kCameraFeedbackTimeoutUs, kSensorTriggerPulseWidthUs, kSensorTriggerPeriodUs))) {
      return false;
    }
    if (kImuTriggerEnabled &&
        (!kImuWiringVerified || !kImuSyncInEventConfigured || !kImuFeedbackEnabled || !kImuSyncOutEventConfigured ||
         !feedbackWindowValid(true, kImuFeedbackTimeoutUs, kSensorTriggerPulseWidthUs, kSensorTriggerPeriodUs))) {
      return false;
    }
    if (kLidarClockEnabled && !kLidarClockWiringVerified) {
      return false;
    }
    uint8_t pins[16] = {0};
    uint8_t pin_count = 0;
    if (any_timed_output && !addUniquePin(kReferenceInputPin, pins, &pin_count)) {
      return false;
    }
    if (kCameraTriggerEnabled) {
      if (kCameraUseHardwareFanout) {
        if (!addUniquePin(kCameraFanoutTriggerPin, pins, &pin_count)) {
          return false;
        }
      } else {
        for (uint8_t i = 0; i < kCameraCount; ++i) {
          if (!addUniquePin(kCameraTriggerPins[i], pins, &pin_count)) {
            return false;
          }
        }
      }
    }
    if (kCameraFeedbackEnabled) {
      for (uint8_t i = 0; i < kCameraCount; ++i) {
        if (!addUniquePin(kCameraExposurePins[i], pins, &pin_count)) {
          return false;
        }
      }
    }
    if (kImuTriggerEnabled && !addUniquePin(kImuTriggerPin, pins, &pin_count)) {
      return false;
    }
    if (kImuFeedbackEnabled && !addUniquePin(kImuSyncPin, pins, &pin_count)) {
      return false;
    }
    if (kLidarClockEnabled) {
      for (uint8_t i = 0; i < kLidarCount; ++i) {
        if (!addUniquePin(kLidarPpsPins[i], pins, &pin_count)) {
          return false;
        }
      }
    }
    return true;
  }

  static void configureOutputInactive(uint8_t pin, bool active_high) {
    digitalWrite(pin, active_high ? LOW : HIGH);
    pinMode(pin, OUTPUT);
  }

  static void writeOutput(uint8_t pin, bool active_high, bool active) {
    digitalWriteFast(pin, active == active_high ? HIGH : LOW);
  }

  static void configureAssignedOutputsInactive() {
    using namespace ig_handle_firmware_config::timing;
    if (kCameraTriggerEnabled) {
      if (kCameraUseHardwareFanout) {
        if (safeAssignedOutput(kCameraFanoutTriggerPin)) {
          configureOutputInactive(kCameraFanoutTriggerPin, kCameraFanoutTriggerActiveHigh);
        }
      } else {
        for (uint8_t i = 0; i < kCameraCount; ++i) {
          if (safeAssignedOutput(kCameraTriggerPins[i])) {
            configureOutputInactive(kCameraTriggerPins[i], kCameraTriggerActiveHigh[i]);
          }
        }
      }
    }
    if (kImuTriggerEnabled && safeAssignedOutput(kImuTriggerPin)) {
      configureOutputInactive(kImuTriggerPin, kImuTriggerActiveHigh);
    }
    if (kLidarClockEnabled) {
      for (uint8_t i = 0; i < kLidarCount; ++i) {
        if (safeAssignedOutput(kLidarPpsPins[i])) {
          configureOutputInactive(kLidarPpsPins[i], kLidarPpsActiveHigh[i]);
        }
      }
    }
  }

  static void configureInputs() {
    using namespace ig_handle_firmware_config::timing;
    if (kCameraFeedbackEnabled) {
      for (uint8_t i = 0; i < kCameraCount; ++i) {
        pinMode(kCameraExposurePins[i], INPUT);
      }
    }
    if (kImuFeedbackEnabled) {
      pinMode(kImuSyncPin, INPUT);
    }
  }

  static void setScheduledTrigger(bool active) {
    using namespace ig_handle_firmware_config::timing;
    if (kCameraTriggerEnabled && kCameraWiringVerified) {
      if (kCameraUseHardwareFanout) {
        writeOutput(kCameraFanoutTriggerPin, kCameraFanoutTriggerActiveHigh, active);
      } else {
        for (uint8_t i = 0; i < kCameraCount; ++i) {
          writeOutput(kCameraTriggerPins[i], kCameraTriggerActiveHigh[i], active);
        }
      }
    }
    if (kImuTriggerEnabled && kImuWiringVerified && kImuSyncInEventConfigured) {
      writeOutput(kImuTriggerPin, kImuTriggerActiveHigh, active);
    }
  }

  static void setLidarPps(bool active) {
    using namespace ig_handle_firmware_config::timing;
    if (!kLidarClockEnabled || !kLidarClockWiringVerified) {
      return;
    }
    for (uint8_t i = 0; i < kLidarCount; ++i) {
      writeOutput(kLidarPpsPins[i], kLidarPpsActiveHigh[i], active);
    }
  }

  void clearCaptureState() {
    camera_feedback_.clear();
    imu_feedback_pending_ = false;
    for (uint8_t i = 0; i < cameraCount(); ++i) {
      camera_open_valid_[i] = false;
      camera_event_ready_[i] = false;
    }
    imu_event_ready_ = false;
  }

  Scheduler scheduler_;
  bool io_configuration_valid_;
  volatile bool lidar_pulse_active_;
  volatile uint32_t lidar_pulse_release_us_;
  ExposureFeedbackTracker<ig_handle_firmware_config::timing::kCameraCount> camera_feedback_;
  volatile bool imu_feedback_pending_;
  volatile uint32_t imu_feedback_deadline_us_;

  volatile bool camera_open_valid_[ig_handle_firmware_config::timing::kCameraCount];
  volatile uint32_t camera_open_sec_[ig_handle_firmware_config::timing::kCameraCount];
  volatile uint32_t camera_open_nsec_[ig_handle_firmware_config::timing::kCameraCount];
  volatile bool camera_event_ready_[ig_handle_firmware_config::timing::kCameraCount];
  volatile uint32_t camera_event_sec_[ig_handle_firmware_config::timing::kCameraCount];
  volatile uint32_t camera_event_nsec_[ig_handle_firmware_config::timing::kCameraCount];
  volatile uint32_t camera_sequence_[ig_handle_firmware_config::timing::kCameraCount];
  volatile uint32_t camera_dropped_[ig_handle_firmware_config::timing::kCameraCount];

  volatile bool imu_event_ready_;
  volatile uint32_t imu_event_sec_;
  volatile uint32_t imu_event_nsec_;
  volatile uint32_t imu_sequence_;
  volatile uint32_t imu_dropped_;
  volatile uint32_t camera_invalid_edges_;
};

}  // namespace sensor_sync
