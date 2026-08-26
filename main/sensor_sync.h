#pragma once

// Side-effect-free reference-qualified trigger scheduler.
//
// Hardware I/O is deliberately kept out of this file so wrap-around,
// qualification, deadline, and watchdog behavior can be checked on a host.

#include <stdint.h>

namespace sensor_sync {

enum class State : uint8_t {
  kDisabled = 0,
  kWaitingForReference,
  kRunning,
  kFault,
};

enum class Fault : uint8_t {
  kNone = 0,
  kInvalidConfiguration,
  kReferencePeriod,
  kReferenceTimeout,
  kTriggerDeadline,
  kFeedbackTimeout,
  kEventQueueOverflow,
  kRuntimeUnavailable,
  kFieldPowerInvalid,
};

struct Config {
  bool enabled;
  bool trigger_enabled;
  bool configured;
  bool wiring_verified;
  uint32_t trigger_period_us;
  uint32_t pulse_width_us;
  uint32_t phase_us;
  uint32_t maximum_lateness_us;
  uint32_t reference_nominal_period_us;
  uint32_t reference_min_period_us;
  uint32_t reference_max_period_us;
  uint32_t reference_timeout_us;
  uint8_t required_stable_reference_edges;
};

struct Actions {
  bool assert_trigger;
  bool release_trigger;
};

struct RelativeTime {
  uint32_t sec;
  uint32_t nsec;
};

// Continuous relative time derived only from qualified MCU reference edges.
// Unsigned edge deltas preserve micros() rollover semantics. This is explicitly
// not UTC and has no relationship to ROS time until separately calibrated.
class RelativeEpoch {
public:
  RelativeEpoch() : initialized_(false), reference_us_(0), epoch_ns_(0) {}

  void reset() {
    initialized_ = false;
    reference_us_ = 0;
    epoch_ns_ = 0;
  }

  RelativeTime onQualifiedReference(uint32_t edge_us) {
    if (!initialized_) {
      initialized_ = true;
      reference_us_ = edge_us;
      epoch_ns_ = 1000000000ULL;
    } else {
      epoch_ns_ += static_cast<uint64_t>(edge_us - reference_us_) * 1000ULL;
      reference_us_ = edge_us;
    }
    return split(epoch_ns_);
  }

  bool stamp(uint32_t edge_us, RelativeTime* result) const {
    if (!initialized_ || result == 0) {
      return false;
    }
    const uint64_t event_ns = epoch_ns_ + static_cast<uint64_t>(edge_us - reference_us_) * 1000ULL;
    *result = split(event_ns);
    return true;
  }

  bool initialized() const { return initialized_; }

private:
  static RelativeTime split(uint64_t nanoseconds) {
    const RelativeTime result = {static_cast<uint32_t>(nanoseconds / 1000000000ULL),
                                 static_cast<uint32_t>(nanoseconds % 1000000000ULL)};
    return result;
  }

  bool initialized_;
  uint32_t reference_us_;
  uint64_t epoch_ns_;
};

// Single-producer/single-consumer ISR mailbox.  The lower-priority GPIO ISR
// writes the record before committing write_index_; the higher-priority timer
// therefore sees either the complete old queue or the complete new record.
// Overflow is explicit and must latch the synchronization runtime off.
template <typename Event, uint8_t Capacity>
class EdgeMailbox {
public:
  EdgeMailbox() : read_index_(0), write_index_(0), overflow_(false) {
    static_assert(Capacity >= 2, "mailbox requires at least two slots");
  }

  bool pushFromIsr(const Event& event) {
    const uint8_t write = write_index_;
    const uint8_t next = increment(write);
    if (next == read_index_) {
      overflow_ = true;
      return false;
    }
    records_[write] = event;
    compilerFence();
    write_index_ = next;
    return true;
  }

  bool popFromOwner(Event* event) {
    if (event == 0) {
      return false;
    }
    const uint8_t read = read_index_;
    const uint8_t write = write_index_;
    compilerFence();
    if (read == write) {
      return false;
    }
    *event = records_[read];
    compilerFence();
    read_index_ = increment(read);
    return true;
  }

  bool takeOverflowFromOwner() {
    const bool overflow = overflow_;
    overflow_ = false;
    return overflow;
  }

private:
  static uint8_t increment(uint8_t value) { return static_cast<uint8_t>((value + 1U) % Capacity); }

  static void compilerFence() { __asm__ volatile("" ::: "memory"); }

  Event records_[Capacity];
  volatile uint8_t read_index_;
  volatile uint8_t write_index_;
  volatile bool overflow_;
};

inline bool feedbackWindowValid(bool enabled, uint32_t timeout_us, uint32_t pulse_width_us, uint32_t period_us) {
  return !enabled || (timeout_us > pulse_width_us && timeout_us < period_us);
}

enum class ExposureEdgeResult : uint8_t {
  kOpened = 0,
  kCompleted,
  kInvalid,
};

// Tracks the full ExposureActive pulse for each camera. A trigger remains
// pending until a valid close edge, so an open-only pulse cannot satisfy the
// watchdog. This class is hardware-free for host checking.
template <uint8_t CameraCount>
class ExposureFeedbackTracker {
public:
  ExposureFeedbackTracker() : pending_mask_(0), open_mask_(0), deadline_us_(0) {}

  void arm(uint32_t now_us, uint32_t timeout_us) {
    pending_mask_ = allMask();
    open_mask_ = 0;
    deadline_us_ = now_us + timeout_us;
  }

  ExposureEdgeResult onEdge(uint8_t channel, bool active) {
    if (channel >= CameraCount || (pending_mask_ & static_cast<uint8_t>(1u << channel)) == 0) {
      return ExposureEdgeResult::kInvalid;
    }
    const uint8_t bit = static_cast<uint8_t>(1u << channel);
    if (active) {
      if ((open_mask_ & bit) != 0) {
        return ExposureEdgeResult::kInvalid;
      }
      open_mask_ |= bit;
      return ExposureEdgeResult::kOpened;
    }
    if ((open_mask_ & bit) == 0) {
      return ExposureEdgeResult::kInvalid;
    }
    open_mask_ &= static_cast<uint8_t>(~bit);
    return ExposureEdgeResult::kCompleted;
  }

  void complete(uint8_t channel) {
    if (channel < CameraCount) {
      pending_mask_ &= static_cast<uint8_t>(~(1u << channel));
    }
  }

  bool expired(uint32_t now_us) const { return pending_mask_ != 0 && reached(now_us, deadline_us_); }

  bool pending() const { return pending_mask_ != 0; }

  void clear() {
    pending_mask_ = 0;
    open_mask_ = 0;
    deadline_us_ = 0;
  }

private:
  static_assert(CameraCount > 0 && CameraCount <= 8, "feedback mask supports one to eight cameras");

  static constexpr uint8_t allMask() { return static_cast<uint8_t>((1u << CameraCount) - 1u); }

  static bool reached(uint32_t now_us, uint32_t deadline_us) { return static_cast<int32_t>(now_us - deadline_us) >= 0; }

  volatile uint8_t pending_mask_;
  volatile uint8_t open_mask_;
  volatile uint32_t deadline_us_;
};

class Scheduler {
public:
  explicit Scheduler(const Config& config)
      : config_(config),
        state_(State::kDisabled),
        fault_(Fault::kNone),
        have_reference_(false),
        pulse_active_(false),
        stable_reference_edges_(0),
        last_reference_us_(0),
        next_trigger_us_(0),
        pulse_release_us_(0),
        trigger_count_(0) {}

  bool begin() {
    resetState();
    if (!config_.enabled) {
      state_ = State::kDisabled;
      return true;
    }
    if (!configurationValid(config_)) {
      latchFault(Fault::kInvalidConfiguration);
      return false;
    }
    state_ = State::kWaitingForReference;
    return true;
  }

  void onReferenceEdge(uint32_t now_us) {
    if (state_ == State::kDisabled || state_ == State::kFault) {
      return;
    }

    if (!have_reference_) {
      have_reference_ = true;
      last_reference_us_ = now_us;
      stable_reference_edges_ = 1;
      return;
    }

    const uint32_t period_us = now_us - last_reference_us_;
    last_reference_us_ = now_us;
    if (period_us < config_.reference_min_period_us || period_us > config_.reference_max_period_us) {
      if (state_ == State::kRunning) {
        latchFault(Fault::kReferencePeriod);
      } else {
        stable_reference_edges_ = 1;
      }
      return;
    }

    if (stable_reference_edges_ < config_.required_stable_reference_edges) {
      ++stable_reference_edges_;
    }
    if (stable_reference_edges_ >= config_.required_stable_reference_edges) {
      if (pulse_active_) {
        latchFault(Fault::kTriggerDeadline);
        return;
      }
      next_trigger_us_ = now_us + config_.phase_us;
      state_ = State::kRunning;
    }
  }

  Actions update(uint32_t now_us) {
    Actions actions = {false, false};
    if (state_ == State::kDisabled || state_ == State::kFault) {
      return actions;
    }

    if (have_reference_ && now_us - last_reference_us_ > config_.reference_timeout_us) {
      if (pulse_active_) {
        pulse_active_ = false;
        actions.release_trigger = true;
      }
      latchFault(Fault::kReferenceTimeout);
      return actions;
    }

    if (state_ != State::kRunning) {
      return actions;
    }

    // LiDAR-only operation qualifies and watches the shaped PPS edge captured
    // from the DS3231-triggered one-shot. Hardware fans that edge directly to
    // both VLP-16s; firmware does not synthesize or gate it. LiDAR-only mode
    // must not create a fictitious camera/IMU trigger stream or accrue
    // scheduler deadline faults.
    if (!config_.trigger_enabled) {
      return actions;
    }

    if (pulse_active_ && reached(now_us, pulse_release_us_)) {
      pulse_active_ = false;
      actions.release_trigger = true;
    }

    if (!pulse_active_ && reached(now_us, next_trigger_us_)) {
      const uint32_t lateness_us = now_us - next_trigger_us_;
      if (lateness_us > config_.maximum_lateness_us) {
        latchFault(Fault::kTriggerDeadline);
        return actions;
      }
      pulse_active_ = true;
      pulse_release_us_ = now_us + config_.pulse_width_us;
      next_trigger_us_ += config_.trigger_period_us;
      ++trigger_count_;
      actions.assert_trigger = true;
    }
    return actions;
  }

  void forceFault(Fault fault) {
    if (fault != Fault::kNone) {
      latchFault(fault);
    }
  }

  State state() const { return state_; }
  Fault fault() const { return fault_; }
  bool pulseActive() const { return pulse_active_; }
  uint32_t triggerCount() const { return trigger_count_; }
  uint8_t stableReferenceEdges() const { return stable_reference_edges_; }

  static bool configurationValid(const Config& config) {
    if (!config.enabled) {
      return true;
    }
    const bool reference_valid =
        config.reference_nominal_period_us > 0 && config.reference_min_period_us < config.reference_nominal_period_us &&
        config.reference_max_period_us > config.reference_nominal_period_us &&
        config.reference_timeout_us > config.reference_max_period_us && config.required_stable_reference_edges >= 2;
    const bool trigger_valid =
        !config.trigger_enabled || (reference_valid && config.trigger_period_us >= 1000 && config.pulse_width_us > 0 &&
                                    config.pulse_width_us < config.trigger_period_us &&
                                    config.phase_us + config.pulse_width_us < config.trigger_period_us &&
                                    config.maximum_lateness_us < config.trigger_period_us &&
                                    config.reference_nominal_period_us % config.trigger_period_us == 0);
    return config.configured && config.wiring_verified && reference_valid && trigger_valid;
  }

private:
  static bool reached(uint32_t now_us, uint32_t deadline_us) { return static_cast<int32_t>(now_us - deadline_us) >= 0; }

  void resetState() {
    state_ = State::kDisabled;
    fault_ = Fault::kNone;
    have_reference_ = false;
    pulse_active_ = false;
    stable_reference_edges_ = 0;
    last_reference_us_ = 0;
    next_trigger_us_ = 0;
    pulse_release_us_ = 0;
    trigger_count_ = 0;
  }

  void latchFault(Fault fault) {
    fault_ = fault;
    state_ = State::kFault;
    pulse_active_ = false;
  }

  const Config config_;
  volatile State state_;
  volatile Fault fault_;
  volatile bool have_reference_;
  volatile bool pulse_active_;
  volatile uint8_t stable_reference_edges_;
  volatile uint32_t last_reference_us_;
  volatile uint32_t next_trigger_us_;
  volatile uint32_t pulse_release_us_;
  volatile uint32_t trigger_count_;
};

inline const char* stateName(State state) {
  switch (state) {
    case State::kDisabled:
      return "disabled";
    case State::kWaitingForReference:
      return "waiting_reference";
    case State::kRunning:
      return "running";
    case State::kFault:
      return "fault";
  }
  return "unknown";
}

inline const char* faultName(Fault fault) {
  switch (fault) {
    case Fault::kNone:
      return "none";
    case Fault::kInvalidConfiguration:
      return "invalid_configuration";
    case Fault::kReferencePeriod:
      return "reference_period";
    case Fault::kReferenceTimeout:
      return "reference_timeout";
    case Fault::kTriggerDeadline:
      return "trigger_deadline";
    case Fault::kFeedbackTimeout:
      return "feedback_timeout";
    case Fault::kEventQueueOverflow:
      return "event_queue_overflow";
    case Fault::kRuntimeUnavailable:
      return "runtime_unavailable";
    case Fault::kFieldPowerInvalid:
      return "field_power_invalid";
  }
  return "unknown";
}

}  // namespace sensor_sync
