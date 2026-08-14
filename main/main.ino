#include <RTClib.h>
#include <ros.h>
#include <sensor_msgs/TimeReference.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#include "firmware_config.h"
#include "firmware_build_identity.h"
#include "firmware_pin_contract.h"
#include "sensor_sync_runtime.h"
#include "telescope_control.h"
#include "telescope_runtime.h"

using namespace ig_handle_firmware_config;

void referenceISR();
void sensorTimerISR();
void attachCameraInterrupt(uint8_t channel);
void imuSyncISR();
void telescopeEncoderISR();

ros::NodeHandle nh;
RTC_DS3231 rtc;
sensor_sync::Runtime sensor_sync_runtime;
::telescope::Runtime telescope_runtime;
sensor_sync::RelativeEpoch relative_epoch;

IntervalTimer sensor_sync_timer;
volatile uint32_t reference_sec = 0;
volatile uint32_t reference_nsec = 0;
volatile bool reference_publish_ready = false;

struct ReferenceEdge {
  uint32_t timestamp_us;
};

struct SensorEdge {
  uint32_t timestamp_us;
  bool active;
};

sensor_sync::EdgeMailbox<ReferenceEdge, 4> reference_edges;
sensor_sync::EdgeMailbox<SensorEdge, 8> camera_edges[ig_handle_firmware_config::timing::kCameraCount];
sensor_sync::EdgeMailbox<SensorEdge, 8> imu_edges;

sensor_msgs::TimeReference pps_time_msg;
sensor_msgs::TimeReference camera_time_msg;
sensor_msgs::TimeReference imu_time_msg;
std_msgs::String timing_status_msg;
char timing_status_buffer[256];
ros::Publisher pps_time_pub(kPpsTimeTopic, &pps_time_msg);
ros::Publisher camera_time_pub(kCameraTimeTopic, &camera_time_msg);
ros::Publisher imu_time_pub(kImuTimeTopic, &imu_time_msg);
ros::Publisher timing_status_pub(kTimingStatusTopic, &timing_status_msg);

std_msgs::Float32 telescope_actual_length_msg;
std_msgs::Float32 telescope_motor_current_msg;
std_msgs::String telescope_status_msg;
char telescope_status_buffer[48];
ros::Publisher telescope_actual_length_pub(ig_handle_firmware_config::telescope::kStateLengthTopic,
                                           &telescope_actual_length_msg);
ros::Publisher telescope_motor_current_pub(ig_handle_firmware_config::telescope::kStateMotorCurrentTopic,
                                           &telescope_motor_current_msg);
ros::Publisher telescope_status_pub(ig_handle_firmware_config::telescope::kStateStatusTopic, &telescope_status_msg);

void telescopeDesiredLengthCallback(const std_msgs::Float32& message) {
  telescope_runtime.setDesiredLength(message.data, millis());
}

ros::Subscriber<std_msgs::Float32> telescope_desired_length_sub(
    ig_handle_firmware_config::telescope::kCommandLengthTopic, telescopeDesiredLengthCallback);

bool timestampFromReference(uint32_t now_us, uint32_t* sec, uint32_t* nsec) {
  if (sec == 0 || nsec == 0) {
    return false;
  }
  sensor_sync::RelativeTime stamp;
  if (!relative_epoch.stamp(now_us, &stamp)) {
    return false;
  }
  *sec = stamp.sec;
  *nsec = stamp.nsec;
  return true;
}

void publishReferenceTime() {
  noInterrupts();
  const bool ready = reference_publish_ready;
  const uint32_t sec = reference_sec;
  const uint32_t nsec = reference_nsec;
  if (ready) {
    reference_publish_ready = false;
  }
  interrupts();
  if (!ready) {
    return;
  }
  ros::Time stamp;
  stamp.sec = sec;
  stamp.nsec = nsec;
  pps_time_msg.header.seq++;
  // No verified UTC/ROS phase mapping exists. header.stamp is only the ROS
  // publication receipt; time_ref carries the relative MCU epoch.
  pps_time_msg.header.stamp = nh.now();
  pps_time_msg.time_ref = stamp;
  pps_time_msg.source = "diagnostic_relative_monotonic_epoch_not_utc_unmapped_to_ros";
  pps_time_pub.publish(&pps_time_msg);
}

void publishCaptureEvents() {
  sensor_sync::CaptureEvent event;
  static uint32_t camera_topic_sequence = 0;
  for (uint8_t channel = 0; channel < sensor_sync::Runtime::cameraCount(); ++channel) {
    if (!sensor_sync_runtime.takeCameraEvent(channel, &event)) {
      continue;
    }
    camera_time_msg.header.seq = ++camera_topic_sequence;
    camera_time_msg.header.stamp = nh.now();
    camera_time_msg.time_ref.sec = event.sec;
    camera_time_msg.time_ref.nsec = event.nsec;
    camera_time_msg.source = ig_handle_firmware_config::timing::kCameraSources[channel];
    camera_time_pub.publish(&camera_time_msg);
  }
  if (sensor_sync_runtime.takeImuEvent(&event)) {
    imu_time_msg.header.seq = event.sequence;
    imu_time_msg.header.stamp = nh.now();
    imu_time_msg.time_ref.sec = event.sec;
    imu_time_msg.time_ref.nsec = event.nsec;
    imu_time_msg.source = ig_handle_firmware_config::timing::kImuSource;
    imu_time_pub.publish(&imu_time_msg);
  }
}

void publishTimingStatus(uint32_t now_ms) {
  static uint32_t last_publish_ms = 0;
  if (now_ms - last_publish_ms < ig_handle_firmware_config::timing::kStatusPublishPeriodMs) {
    return;
  }
  last_publish_ms = now_ms;
  uint32_t camera_drops = 0;
  for (uint8_t channel = 0; channel < sensor_sync::Runtime::cameraCount(); ++channel) {
    camera_drops += sensor_sync_runtime.cameraDropped(channel);
  }
  snprintf(timing_status_buffer, sizeof(timing_status_buffer),
           "firmware_build_id=%s state=%s fault=%s reference_edges=%u triggers=%lu "
           "camera_drops=%lu camera_invalid_edges=%lu imu_drops=%lu",
           kFirmwareBuildId, sensor_sync::stateName(sensor_sync_runtime.state()),
           sensor_sync::faultName(sensor_sync_runtime.fault()),
           sensor_sync_runtime.stableReferenceEdges(), static_cast<unsigned long>(sensor_sync_runtime.triggerCount()),
           static_cast<unsigned long>(camera_drops),
           static_cast<unsigned long>(sensor_sync_runtime.cameraInvalidEdges()),
           static_cast<unsigned long>(sensor_sync_runtime.imuDropped()));
  timing_status_msg.data = timing_status_buffer;
  timing_status_pub.publish(&timing_status_msg);
}

void publishTelescopeState(uint32_t now_ms) {
  static uint32_t last_publish_ms = 0;
  if (now_ms - last_publish_ms < ig_handle_firmware_config::telescope::kTelemetryPublishPeriodMs) {
    return;
  }
  last_publish_ms = now_ms;
  telescope_actual_length_msg.data = telescope_runtime.actualLengthM();
  telescope_motor_current_msg.data = telescope_runtime.motorCurrentA();
  snprintf(telescope_status_buffer, sizeof(telescope_status_buffer), "%s", telescope_runtime.status());
  telescope_status_msg.data = telescope_status_buffer;
  telescope_actual_length_pub.publish(&telescope_actual_length_msg);
  telescope_motor_current_pub.publish(&telescope_motor_current_msg);
  telescope_status_pub.publish(&telescope_status_msg);
}

void setup() {
  nh.initNode();
  nh.advertise(pps_time_pub);
  nh.advertise(camera_time_pub);
  nh.advertise(imu_time_pub);
  nh.advertise(timing_status_pub);
  nh.advertise(telescope_actual_length_pub);
  nh.advertise(telescope_motor_current_pub);
  nh.advertise(telescope_status_pub);
  nh.subscribe(telescope_desired_length_sub);

  using namespace ig_handle_firmware_config::timing;
  const bool timing_requested = kCameraTriggerEnabled || kImuTriggerEnabled || kLidarClockEnabled;

  // Configure all verified outputs inactive before any runtime dependency can
  // fault. External hard pull-down/driver-disable circuitry remains required.
  const bool pin_contract_valid = firmware_pin_contract::valid();
  const bool timing_initialized = pin_contract_valid && sensor_sync_runtime.begin();
  if (!pin_contract_valid) {
    sensor_sync_runtime.forceFault(sensor_sync::Fault::kInvalidConfiguration);
    nh.logerror("whole-firmware Teensy pin contract is invalid");
  }
  bool rtc_available = true;
  if (timing_requested) {
    rtc_available = rtc.begin();
  }
  if (timing_requested && !rtc_available) {
    nh.logerror("RTC unavailable; sensor synchronization remains disabled");
    sensor_sync_runtime.forceFault(sensor_sync::Fault::kRuntimeUnavailable);
  } else if (timing_requested) {
    rtc.disable32K();
    rtc.writeSqwPinMode(DS3231_SquareWave1Hz);
  }

  const bool timing_runtime_ready = timing_requested && rtc_available && timing_initialized;
  if (timing_runtime_ready && kReferenceWiringVerified) {
    pinMode(kReferenceInputPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(kReferenceInputPin), referenceISR, kReferenceActiveHigh ? RISING : FALLING);
  }
  if (timing_runtime_ready && sensor_sync_runtime.timerRequired()) {
    sensor_sync_timer.priority(96);
    if (!sensor_sync_timer.begin(sensorTimerISR, kSchedulerTickUs)) {
      sensor_sync_runtime.forceFault(sensor_sync::Fault::kInvalidConfiguration);
    }
  }

  for (uint8_t channel = 0; channel < sensor_sync::Runtime::cameraCount(); ++channel) {
    if (timing_runtime_ready && sensor_sync_runtime.cameraFeedbackConfigured(channel)) {
      attachCameraInterrupt(channel);
    }
  }
  if (timing_runtime_ready && sensor_sync_runtime.imuFeedbackConfigured()) {
    attachInterrupt(digitalPinToInterrupt(ig_handle_firmware_config::timing::kImuSyncPin), imuSyncISR,
                    ig_handle_firmware_config::timing::kImuSyncActiveHigh ? RISING : FALLING);
  }

  if (pin_contract_valid && telescope_runtime.begin(millis())) {
    attachInterrupt(digitalPinToInterrupt(ig_handle_firmware_config::telescope::kEncoderPhaseAPin), telescopeEncoderISR,
                    CHANGE);
    attachInterrupt(digitalPinToInterrupt(ig_handle_firmware_config::telescope::kEncoderPhaseBPin), telescopeEncoderISR,
                    CHANGE);
  }
}

void loop() {
  nh.spinOnce();
  publishReferenceTime();
  publishCaptureEvents();
  publishTimingStatus(millis());
  telescope_runtime.update(millis());
  publishTelescopeState(millis());
  nh.spinOnce();
}

void referenceISR() {
  const ReferenceEdge edge = {micros()};
  reference_edges.pushFromIsr(edge);
}

void processReferenceEdges() {
  ReferenceEdge edge;
  if (reference_edges.takeOverflowFromOwner()) {
    sensor_sync_runtime.forceFault(sensor_sync::Fault::kEventQueueOverflow);
  }
  while (reference_edges.popFromOwner(&edge)) {
    sensor_sync_runtime.onReferenceEdge(edge.timestamp_us);
    if (sensor_sync_runtime.state() != sensor_sync::State::kRunning) {
      relative_epoch.reset();
      reference_sec = 0;
      reference_nsec = 0;
      reference_publish_ready = false;
      continue;
    }
    const sensor_sync::RelativeTime reference = relative_epoch.onQualifiedReference(edge.timestamp_us);
    reference_sec = reference.sec;
    reference_nsec = reference.nsec;
    reference_publish_ready = true;
  }
}

void processCameraEdges() {
  for (uint8_t channel = 0; channel < ig_handle_firmware_config::timing::kCameraCount; ++channel) {
    if (camera_edges[channel].takeOverflowFromOwner()) {
      sensor_sync_runtime.forceFault(sensor_sync::Fault::kEventQueueOverflow);
    }
    SensorEdge edge;
    while (camera_edges[channel].popFromOwner(&edge)) {
      uint32_t sec = 0;
      uint32_t nsec = 0;
      if (!timestampFromReference(edge.timestamp_us, &sec, &nsec)) {
        continue;
      }
      sensor_sync_runtime.onCameraExposureEdge(channel, edge.active, edge.timestamp_us, sec, nsec);
    }
  }
}

void processImuEdges() {
  if (imu_edges.takeOverflowFromOwner()) {
    sensor_sync_runtime.forceFault(sensor_sync::Fault::kEventQueueOverflow);
  }
  SensorEdge edge;
  while (imu_edges.popFromOwner(&edge)) {
    uint32_t sec = 0;
    uint32_t nsec = 0;
    if (!edge.active || !timestampFromReference(edge.timestamp_us, &sec, &nsec)) {
      continue;
    }
    sensor_sync_runtime.onImuSyncEdge(edge.timestamp_us, sec, nsec);
  }
}

void sensorTimerISR() {
  // Drain sensor feedback against the most recent qualified reference before
  // advancing that reference. This keeps just-before-edge samples in the old
  // interval instead of unsigned-wrapping them into the next epoch.
  processCameraEdges();
  processImuEdges();
  processReferenceEdges();
  sensor_sync_runtime.onTimerTick(micros());
  if (sensor_sync_runtime.state() != sensor_sync::State::kRunning) {
    relative_epoch.reset();
    reference_sec = 0;
    reference_nsec = 0;
    reference_publish_ready = false;
  }
}

void handleCameraEdge(uint8_t channel) {
  if (channel >= ig_handle_firmware_config::timing::kCameraCount) {
    return;
  }
  const SensorEdge edge = {micros(), sensor_sync_runtime.cameraInputActive(channel)};
  camera_edges[channel].pushFromIsr(edge);
}

void camera0ISR() { handleCameraEdge(0); }
void camera1ISR() { handleCameraEdge(1); }
void camera2ISR() { handleCameraEdge(2); }
void camera3ISR() { handleCameraEdge(3); }

void attachCameraInterrupt(uint8_t channel) {
  using namespace ig_handle_firmware_config::timing;
  void (*handlers[kCameraCount])() = {camera0ISR, camera1ISR, camera2ISR, camera3ISR};
  attachInterrupt(digitalPinToInterrupt(kCameraExposurePins[channel]), handlers[channel], CHANGE);
}

void imuSyncISR() {
  const SensorEdge edge = {micros(), sensor_sync_runtime.imuInputActive()};
  imu_edges.pushFromIsr(edge);
}

void telescopeEncoderISR() { telescope_runtime.onEncoderEdge(); }
