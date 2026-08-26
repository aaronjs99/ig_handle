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
void telescopeFieldInvalidISR();

ros::NodeHandle nh;
RTC_DS3231 rtc;
sensor_sync::Runtime sensor_sync_runtime;
::telescope::Runtime telescope_runtime;
sensor_sync::RelativeEpoch relative_epoch;

IntervalTimer sensor_sync_timer;
volatile uint32_t reference_sec = 0;
volatile uint32_t reference_nsec = 0;
volatile bool reference_publish_ready = false;
volatile uint32_t lidar_nmea_due_us = 0;
volatile uint32_t lidar_nmea_latest_start_us = 0;
volatile uint32_t lidar_nmea_latest_enqueue_us = 0;
volatile bool lidar_nmea_pending = false;
volatile bool lidar_nmea_abort_requested = false;
bool rtc_time_valid = false;
char lidar_nmea_sentence[80] = {0};
static_assert(sizeof(lidar_nmea_sentence) <= ig_handle_firmware_config::timing::kLidarNmeaMaximumSentenceBytes,
              "configured VLP NMEA wire-time bound must cover the sentence buffer");
uint8_t lidar_nmea_sentence_length = 0;
uint8_t lidar_nmea_sentence_offset = 0;
bool lidar_nmea_transmitting = false;
uint32_t lidar_nmea_tx_latest_enqueue_us = 0;
uint32_t lidar_nmea_sent = 0;
uint32_t lidar_nmea_suppressed = 0;

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

void serviceLidarNmea() {
  using namespace ig_handle_firmware_config::timing;
  if (!kLidarNmeaEnabled) {
    return;
  }

  const uint32_t now_us = micros();
  noInterrupts();
  const bool abort_requested = lidar_nmea_abort_requested;
  lidar_nmea_abort_requested = false;
  interrupts();
  if (abort_requested && lidar_nmea_transmitting) {
    lidar_nmea_transmitting = false;
    lidar_nmea_sentence_length = 0;
    lidar_nmea_sentence_offset = 0;
    ++lidar_nmea_suppressed;
  }

  // Never let a 9600-baud sentence block camera, IMU, motor, or rosserial
  // service. Feed only as many bytes as the hardware UART can accept now, and
  // never enqueue a late tail that could violate the VLP's final 300 ms quiet
  // window before the next PPS.
  if (lidar_nmea_transmitting) {
    if (static_cast<int32_t>(now_us - lidar_nmea_tx_latest_enqueue_us) > 0) {
      lidar_nmea_transmitting = false;
      lidar_nmea_sentence_length = 0;
      lidar_nmea_sentence_offset = 0;
      ++lidar_nmea_suppressed;
      return;
    }
    const int writable = Serial1.availableForWrite();
    if (writable <= 0) {
      return;
    }
    const uint8_t remaining = lidar_nmea_sentence_length - lidar_nmea_sentence_offset;
    const uint8_t chunk = writable < remaining ? static_cast<uint8_t>(writable) : remaining;
    const size_t written = Serial1.write(
        reinterpret_cast<const uint8_t*>(lidar_nmea_sentence + lidar_nmea_sentence_offset), chunk);
    lidar_nmea_sentence_offset += static_cast<uint8_t>(written);
    if (lidar_nmea_sentence_offset >= lidar_nmea_sentence_length) {
      lidar_nmea_transmitting = false;
      ++lidar_nmea_sent;
    }
    return;
  }

  noInterrupts();
  const bool due = lidar_nmea_pending && static_cast<int32_t>(now_us - lidar_nmea_due_us) >= 0;
  const uint32_t latest_start_us = lidar_nmea_latest_start_us;
  const uint32_t latest_enqueue_us = lidar_nmea_latest_enqueue_us;
  if (due) {
    lidar_nmea_pending = false;
  }
  interrupts();
  if (!due) {
    return;
  }
  if (static_cast<int32_t>(now_us - latest_start_us) > 0) {
    ++lidar_nmea_suppressed;
    return;
  }
  if (!rtc_time_valid) {
    ++lidar_nmea_suppressed;
    return;
  }

  const DateTime rtc_now = rtc.now();
  if (rtc_now.year() < 2024 || rtc_now.year() > 2099) {
    ++lidar_nmea_suppressed;
    return;
  }
  const DateTime nmea_time = kLidarNmeaAppliesToNextPps ? rtc_now + TimeSpan(1) : rtc_now;
  char payload[64];
  // Deliberately preserve receiver-invalid status V and blank position fields:
  // this RTC-only source provides local date/time, not GNSS validity. Enabling
  // this path is compile-time gated on read-back of both VLP qualifier settings;
  // never substitute status A merely to make a sensor accept the sentence.
  snprintf(payload, sizeof(payload), "GPRMC,%02u%02u%02u.00,V,,,,,,,%02u%02u%02u,,,N", nmea_time.hour(),
           nmea_time.minute(), nmea_time.second(), nmea_time.day(), nmea_time.month(), nmea_time.year() % 100);
  uint8_t checksum = 0;
  for (const char* cursor = payload; *cursor != '\0'; ++cursor) {
    checksum ^= static_cast<uint8_t>(*cursor);
  }
  const int sentence_length =
      snprintf(lidar_nmea_sentence, sizeof(lidar_nmea_sentence), "$%s*%02X\r\n", payload, checksum);
  if (sentence_length <= 0 || sentence_length >= static_cast<int>(sizeof(lidar_nmea_sentence))) {
    ++lidar_nmea_suppressed;
    return;
  }
  lidar_nmea_sentence_length = static_cast<uint8_t>(sentence_length);
  lidar_nmea_sentence_offset = 0;
  lidar_nmea_tx_latest_enqueue_us = latest_enqueue_us;
  lidar_nmea_transmitting = true;
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
           "camera_drops=%lu camera_invalid_edges=%lu imu_drops=%lu nmea_sent=%lu nmea_suppressed=%lu",
           kFirmwareBuildId, sensor_sync::stateName(sensor_sync_runtime.state()),
           sensor_sync::faultName(sensor_sync_runtime.fault()),
           sensor_sync_runtime.stableReferenceEdges(), static_cast<unsigned long>(sensor_sync_runtime.triggerCount()),
           static_cast<unsigned long>(camera_drops),
           static_cast<unsigned long>(sensor_sync_runtime.cameraInvalidEdges()),
           static_cast<unsigned long>(sensor_sync_runtime.imuDropped()), static_cast<unsigned long>(lidar_nmea_sent),
           static_cast<unsigned long>(lidar_nmea_suppressed));
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
  const bool timing_requested = kCommonTimingRequested;

  // Configure all verified outputs inactive before any runtime dependency can
  // fault. External hard pull-down/driver-disable circuitry remains required.
  const bool pin_contract_valid = firmware_pin_contract::valid();
  const bool timing_initialized = pin_contract_valid && sensor_sync_runtime.begin();
  if (!pin_contract_valid) {
    sensor_sync_runtime.forceFault(sensor_sync::Fault::kInvalidConfiguration);
    nh.logerror("whole-firmware Teensy pin contract is invalid");
  }
  // The battery-backed DS3231 can retain a previously selected square-wave
  // mode. Probe it on every boot, even when every timing gate is false, so the
  // disabled configuration actively requests SQW off instead of trusting
  // persistent RTC state.
  const bool rtc_available = rtc.begin();
  if (!rtc_available) {
    if (timing_requested) {
      nh.logerror("RTC unavailable; sensor synchronization remains disabled");
      sensor_sync_runtime.forceFault(sensor_sync::Fault::kRuntimeUnavailable);
    } else {
      nh.logwarn("RTC unavailable; persistent SQW state could not be forced off");
    }
  } else {
    rtc.disable32K();
    if (!timing_requested) {
      rtc.writeSqwPinMode(DS3231_OFF);
    } else {
      rtc_time_valid = !rtc.lostPower();
      // The DS3231 SQW falling edge triggers the hardware one-shot. D12
      // observes the shaped active-high PPS rising edge; firmware does not
      // synthesize PPS. The datasheet's approximate phase relationship is not
      // bench evidence: scope this module's SQW against register rollover and
      // PPS_MASTER before claiming phase.
      rtc.writeSqwPinMode(DS3231_SquareWave1Hz);
      if (kLidarNmeaEnabled) {
        Serial1.begin(kLidarNmeaBaud, kLidarNmeaTxInverted ? SERIAL_8N1_TXINV : SERIAL_8N1);
        if (!rtc_time_valid) {
          nh.logerror("RTC lost-power flag set; VLP NMEA remains suppressed");
        }
      }
    }
  }

  const bool timing_runtime_ready = timing_requested && rtc_available && timing_initialized;
  if (timing_runtime_ready && kReferenceWiringVerified) {
    pinMode(kReferenceInputPin, INPUT);
    // kReferenceActiveHigh is true for the shaped PPS_MASTER edge on D12.
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
    attachInterrupt(digitalPinToInterrupt(ig_handle_firmware_config::telescope::kFieldValidPin),
                    telescopeFieldInvalidISR, FALLING);
  }
}

void loop() {
  nh.spinOnce();
  publishReferenceTime();
  serviceLidarNmea();
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
      lidar_nmea_pending = false;
      lidar_nmea_abort_requested = true;
      continue;
    }
    const sensor_sync::RelativeTime reference = relative_epoch.onQualifiedReference(edge.timestamp_us);
    reference_sec = reference.sec;
    reference_nsec = reference.nsec;
    reference_publish_ready = true;
    if (ig_handle_firmware_config::timing::kLidarNmeaEnabled) {
      lidar_nmea_due_us = edge.timestamp_us + ig_handle_firmware_config::timing::kLidarNmeaDelayAfterPpsUs;
      lidar_nmea_latest_start_us =
          edge.timestamp_us + ig_handle_firmware_config::timing::kLidarNmeaLatestStartAfterPpsUs;
      lidar_nmea_latest_enqueue_us =
          edge.timestamp_us + ig_handle_firmware_config::timing::kLidarNmeaLatestEnqueueAfterPpsUs;
      lidar_nmea_pending = true;
    }
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
    lidar_nmea_pending = false;
    lidar_nmea_abort_requested = true;
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

void telescopeFieldInvalidISR() { telescope_runtime.onFieldInvalidEdge(); }
