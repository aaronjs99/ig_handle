#include "../config/teensy/firmware_config.h"
#include <ros.h>
#include <sensor_msgs/TimeReference.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <RTClib.h>
#include <TimeLib.h>
#include "telescope_control.h"
#include "telescope_runtime.h"

using namespace ig_handle_firmware_config;

// Keep the existing serial alias readable while making its selection
// configurable from config/teensy/firmware_config.h.
#define GPSERIAL IG_HANDLE_GPSERIAL

// ROS nodehandle
ros::NodeHandle nh;

// PPS signal source
RTC_DS3231 rtc;

// messages and time topics for camera and imu
sensor_msgs::TimeReference pps_time_msg;
sensor_msgs::TimeReference cam_time_msg;
sensor_msgs::TimeReference imu_time_msg;
ros::Publisher pps_time_pub(kPpsTimeTopic, &pps_time_msg);
ros::Publisher cam_time_pub(kCameraTimeTopic, &cam_time_msg);
ros::Publisher imu_time_pub(kImuTimeTopic, &imu_time_msg);

// Telescope topics use metres for desired/actual length and amperes for
// motor_current. The runtime itself keeps all motor pins inert until its
// measured firmware configuration is explicitly enabled.
telescope::Runtime telescope_runtime;
std_msgs::Float32 telescope_actual_length_msg;
std_msgs::Float32 telescope_motor_current_msg;
std_msgs::String telescope_status_msg;
char telescope_status_buffer[48];
ros::Publisher telescope_actual_length_pub(
    ig_handle_firmware_config::telescope::kStateLengthTopic,
                                           &telescope_actual_length_msg);
ros::Publisher telescope_motor_current_pub(
    ig_handle_firmware_config::telescope::kStateMotorCurrentTopic,
                                           &telescope_motor_current_msg);
ros::Publisher telescope_status_pub(
    ig_handle_firmware_config::telescope::kStateStatusTopic,
    &telescope_status_msg);

// time-sync indicators
volatile time_t rtc_time{0};
elapsedMillis nmea_delay;
elapsedMicros micros_since_pps;
unsigned long cam_open_t_sec, cam_mid_t_sec, cam_close_t_sec;
unsigned long cam_open_t_nsec, cam_mid_t_nsec, cam_close_t_nsec;
volatile uint32_t pps_stamp_sec = 0;
volatile uint32_t pps_stamp_nsec = 0;
volatile uint32_t cam_mid_stamp_sec = 0;
volatile uint32_t cam_mid_stamp_nsec = 0;
volatile uint32_t imu_stamp_sec = 0;
volatile uint32_t imu_stamp_nsec = 0;
volatile uint32_t pps_edge_count = 0;
volatile bool pps_time_initialized = false;
volatile bool pps_stamp_valid = false;
volatile bool camera_open_valid = false;
volatile bool pub_pps_time = false, send_nmea = false;
volatile bool cam_captured = false, imu_sampled = false;

ros::Time snapshotPpsStamp() {
  noInterrupts();
  const uint32_t sec = pps_stamp_sec;
  const uint32_t nsec = pps_stamp_nsec;
  interrupts();
  ros::Time stamp;
  stamp.sec = sec;
  stamp.nsec = nsec;
  return stamp;
}

bool takePpsStamp(ros::Time* stamp) {
  noInterrupts();
  const bool ready = pub_pps_time;
  if (ready) {
    stamp->sec = pps_stamp_sec;
    stamp->nsec = pps_stamp_nsec;
    pub_pps_time = false;
  }
  interrupts();
  return ready;
}

bool takeCameraStamp(ros::Time* stamp) {
  noInterrupts();
  const bool ready = cam_captured;
  if (ready) {
    stamp->sec = cam_mid_stamp_sec;
    stamp->nsec = cam_mid_stamp_nsec;
    cam_captured = false;
  }
  interrupts();
  return ready;
}

bool takeImuStamp(ros::Time* stamp) {
  noInterrupts();
  const bool ready = imu_sampled;
  if (ready) {
    stamp->sec = imu_stamp_sec;
    stamp->nsec = imu_stamp_nsec;
    imu_sampled = false;
  }
  interrupts();
  return ready;
}

void initializeRtcTimeFromRos() {
  noInterrupts();
  const uint32_t edge_count_before = pps_edge_count;
  const bool already_initialized = pps_time_initialized;
  interrupts();
  if (already_initialized) {
    return;
  }
  const ros::Time ros_now = nh.now();
  if (ros_now.sec == 0) {
    return;
  }
  // Only arm after an interval without a PPS edge. If an edge arrives while
  // ROS time is sampled, retry on the next loop so the counter still denotes
  // the next physical PPS edge.
  const time_t next_pps_sec = ros_now.sec + 1;
  noInterrupts();
  if (!pps_time_initialized && pps_edge_count == edge_count_before) {
    rtc_time = next_pps_sec;
    pps_time_initialized = true;
  }
  interrupts();
}

void telescopeDesiredLengthCallback(const std_msgs::Float32& message) {
  telescope_runtime.setDesiredLength(message.data, millis());
}

ros::Subscriber<std_msgs::Float32> telescope_desired_length_sub(
    ig_handle_firmware_config::telescope::kCommandLengthTopic,
    telescopeDesiredLengthCallback);

void telescopeEncoderISR(void) { telescope_runtime.onEncoderEdge(); }

void publishTelescopeState(uint32_t now_ms) {
  static uint32_t last_telescope_publish_ms = 0;
  if (now_ms - last_telescope_publish_ms <
      ig_handle_firmware_config::telescope::kTelemetryPublishPeriodMs) {
    return;
  }
  last_telescope_publish_ms = now_ms;
  telescope_actual_length_msg.data = telescope_runtime.actualLengthM();
  telescope_motor_current_msg.data = telescope_runtime.motorCurrentA();
  snprintf(telescope_status_buffer, sizeof(telescope_status_buffer), "%s",
           telescope_runtime.status());
  telescope_status_msg.data = telescope_status_buffer;
  telescope_actual_length_pub.publish(&telescope_actual_length_msg);
  telescope_motor_current_pub.publish(&telescope_motor_current_msg);
  telescope_status_pub.publish(&telescope_status_msg);
}

/*
   Initial setup for the arduino sketch
   This function:
    - Configures timers for LiDAR PPS and camera triggering
    - Advertises and subscribes to ROS topics
    - UART Serial setup for NMEA messages
    - Holds until rosserial is connected
*/
void setup() {
  /* Lidar */

  // set GPSERIAL baud rate and transmission inversion for TTL RS-232
  // transmission
  GPSERIAL.begin(kGpsBaudRate, IG_HANDLE_GPSERIAL_FORMAT);

  // set PPS synch pin
  pinMode(kPpsOutPin, OUTPUT);

  /* Camera and IMU */

  // node initialization
  nh.initNode();
  nh.advertise(pps_time_pub);
  nh.advertise(cam_time_pub);
  nh.advertise(imu_time_pub);
  nh.advertise(telescope_actual_length_pub);
  nh.advertise(telescope_motor_current_pub);
  nh.advertise(telescope_status_pub);
  nh.subscribe(telescope_desired_length_sub);
  while (!nh.connected()) {
    nh.spinOnce();
  }

  // configure input and output pins
  pinMode(kCameraOpenInPin, INPUT_PULLUP);
  pinMode(kCameraCloseInPin, INPUT_PULLUP);
  pinMode(kCameraTriggerOutPin, OUTPUT);
  pinMode(kImuSyncInPin, INPUT);
  pinMode(kImuTriggerOutPin, OUTPUT);

  // enable interrupts
  attachInterrupt(digitalPinToInterrupt(kCameraOpenInPin), camOpenISR, RISING);
  attachInterrupt(digitalPinToInterrupt(kCameraCloseInPin), camCloseISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(kImuSyncInPin), imuISR, RISING);

  // set write frequency
  analogWriteFrequency(kCameraTriggerOutPin, kCameraTriggerFrequencyHz);
  digitalWrite(kImuTriggerOutPin, LOW);

  // enable triggers
  analogWrite(kCameraTriggerOutPin, kCameraTriggerDuty);
  digitalWrite(kImuTriggerOutPin, HIGH);

  /* RTC */

  // initialize
  if (!rtc.begin()) {
    nh.loginfo("Couldn't find RTC");
    while (1) delay(10);
  }
  rtc.disable32K();

  // set PPS input pin and write signal
  pinMode(kPpsInPin, INPUT_PULLUP);
  rtc.writeSqwPinMode(DS3231_SquareWave1Hz);

  // enable interrupt
  attachInterrupt(digitalPinToInterrupt(kPpsInPin), ppsISR, RISING);

  // This only configures telescope pins when every measured safety gate is
  // true. The committed dummy configuration returns false and stays inert.
  if (telescope_runtime.begin(millis())) {
    attachInterrupt(
        digitalPinToInterrupt(ig_handle_firmware_config::telescope::kEncoderPhaseAPin),
        telescopeEncoderISR, CHANGE);
    attachInterrupt(
        digitalPinToInterrupt(ig_handle_firmware_config::telescope::kEncoderPhaseBPin),
        telescopeEncoderISR, CHANGE);
  }
}

/*
   Main loop
   This function:
    - Transmits NMEA messages over GPSERIAL
    - Publishes the camera capture timestamp to /cam_time
    - Publishes the IMU sample timestamp to /imu_time
*/
void loop() {
  nh.spinOnce(); // Handle ROS communication at the start
  initializeRtcTimeFromRos();

  // publish PPS time as a reference for soft-synch
  ros::Time pps_snapshot;
  if (takePpsStamp(&pps_snapshot)) {
    pps_time_msg.header.seq = pps_snapshot.sec;
    pps_time_msg.header.stamp = pps_snapshot;
    pps_time_msg.time_ref = pps_snapshot;
    pps_time_pub.publish(&pps_time_msg);
  }

  // ensure PPS width satisfied
  if (send_nmea && nmea_delay >= kPpsPulseWidthMs) {
    // set PPS pin to low
    digitalWriteFast(kPpsOutPin, LOW);

    // ensure min 50 ms width between end of PPS and start of NMEA message
    if (nmea_delay >= kPpsNmeaMinSeparationMs) {
      if (!kNmeaPayloadEnabled) {
        send_nmea = false;
      } else {
      // Snapshot the ISR-owned PPS value before using it in the foreground.
      const time_t t_sec_gmt =
          snapshotPpsStamp().sec - kTimeZoneOffsetHours * 3600;

      // create GPRMC sentence
      char time_now[7], date_now[7];
      sprintf(time_now, "%02i%02i%02i", hour(t_sec_gmt), minute(t_sec_gmt),
              second(t_sec_gmt));
      sprintf(date_now, "%02i%02i%02i", day(t_sec_gmt), month(t_sec_gmt),
              year(t_sec_gmt) % 100);
      String gprmc_sentence = String(kNmeaPrefix) + String(time_now) + "," +
                              String(kNmeaStatus) + "," +
                              String(kNmeaLatitude) + "," +
                              String(kNmeaLongitude) + "," +
                              String(kNmeaSpeedKnots) + "," +
                              String(kNmeaCourseDegrees) + "," +
                              String(date_now) + "," +
                              String(kNmeaMagneticVariation);
      String chk = checksum(gprmc_sentence);
      gprmc_sentence = "$" + gprmc_sentence + "*" + chk + "\n";

      // print GPRMC sentence to serial as an NMEA message
      GPSERIAL.print(gprmc_sentence);
      // nh.loginfo(gprmc_sentence.c_str());  // DEBUG

      // reset send
      send_nmea = false;
      }
    }
  }

  ros::Time camera_snapshot;
  if (takeCameraStamp(&camera_snapshot)) {
    cam_time_msg.time_ref = camera_snapshot;
    cam_time_msg.header.seq++;
    cam_time_msg.header.stamp = camera_snapshot;
    cam_time_pub.publish(&cam_time_msg);
  }

  ros::Time imu_snapshot;
  if (takeImuStamp(&imu_snapshot)) {
    imu_time_msg.time_ref = imu_snapshot;
    imu_time_msg.header.seq++;
    imu_time_msg.header.stamp = imu_snapshot;
    imu_time_pub.publish(&imu_time_msg);
  }

  telescope_runtime.update(millis());
  publishTelescopeState(millis());

  nh.spinOnce();
}

// Timestamp creation interrupts
void ppsISR(void) {
  ++pps_edge_count;
  if (!pps_time_initialized) {
    return;
  }
  const time_t pps_time_sec = rtc_time;
  if (pps_time_sec == 0) {
    return;
  }

  // reset elapsed microseconds
  micros_since_pps = 0;

  // set time of PPS according to RTC clock
  pps_stamp_sec = pps_time_sec;
  pps_stamp_nsec = 0;
  pps_stamp_valid = true;

  // toggle to HIGH
  digitalToggleFast(kPpsOutPin);

  // counters and resets
  rtc_time = pps_time_sec + 1;  // increment time
  pub_pps_time = true;  // enable pps time ref publication
  send_nmea = true;     // enable nmea send
  nmea_delay = 0;       // reset delay counter
}

void camOpenISR(void) {
  if (!pps_stamp_valid) {
    return;
  }
  cam_open_t_sec = pps_stamp_sec;
  cam_open_t_nsec = micros_since_pps * 1000;
  camera_open_valid = true;
  // ros::Time cam_open_stamp(cam_open_t_sec, cam_open_t_nsec);  // DEBUG
  // printROSTime("CAM OPN Time:", cam_open_stamp);              // DEBUG
}

void camCloseISR(void) {
  if (!pps_stamp_valid || !camera_open_valid) {
    return;
  }
  camera_open_valid = false;
  cam_close_t_sec = pps_stamp_sec;
  cam_close_t_nsec = micros_since_pps * 1000;

  // Compute the midpoint in absolute nanoseconds so a capture crossing a
  // PPS boundary is handled without a one-second timestamp error.
  const uint64_t open_ns = static_cast<uint64_t>(cam_open_t_sec) * 1000000000ULL +
                           static_cast<uint64_t>(cam_open_t_nsec);
  const uint64_t close_ns = static_cast<uint64_t>(cam_close_t_sec) * 1000000000ULL +
                            static_cast<uint64_t>(cam_close_t_nsec);
  if (close_ns < open_ns) {
    // Do not publish a wrapped timestamp when the ISR state is incoherent or
    // the shutter-open event was missed.
    cam_captured = false;
    return;
  }
  const uint64_t mid_ns = open_ns + (close_ns - open_ns) / 2ULL;
  cam_mid_t_sec = static_cast<unsigned long>(mid_ns / 1000000000ULL);
  cam_mid_t_nsec = static_cast<unsigned long>(mid_ns % 1000000000ULL);

  cam_mid_stamp_sec = cam_mid_t_sec;
  cam_mid_stamp_nsec = cam_mid_t_nsec;
  cam_captured = true;

  // ros::Time cam_close_stamp(cam_close_t_sec, cam_close_t_nsec);  // DEBUG
  // printROSTime("CAM MID Time:", cam_mid_stamp);                  // DEBUG
  // printROSTime("CAM CLD Time:", cam_close_stamp);                // DEBUG
}

void imuISR(void) {
  if (!pps_stamp_valid) {
    return;
  }
  imu_stamp_sec = pps_stamp_sec;
  imu_stamp_nsec = micros_since_pps * 1000;
  imu_sampled = true;
  // printROSTime("IMU Time:", imu_stamp);  // DEBUG
}

// Computes XOR checksum of GPRMC sentence
String checksum(String msg) {
  byte chksum = 0;
  int l = msg.length();
  for (int i = 0; i < l; i++) {
    chksum ^= msg[i];
  }

  String result = String(chksum, HEX);
  result.toUpperCase();
  if (result.length() < 2) {
    result = "0" + result;
  }
  return result;
}

// Print for debugging
void printROSTime(const String& msg, const ros::Time& ros_time) {
  // get seconds and nano seconds
  const time_t& t_sec = ros_time.sec;
  const time_t& t_nsec = ros_time.nsec;

  // convert ros time to string
  char t_sec_string[11], t_nsec_string[10];
  sprintf(t_sec_string, "%lld", (long long)t_sec);
  sprintf(t_nsec_string, "%lld", (long long)t_nsec);
  String ros_time_string =
      "sec: " + String(t_sec_string) + " nsec: " + String(t_nsec_string);

  // print
  nh.loginfo(msg.c_str());
  nh.loginfo(ros_time_string.c_str());
}
