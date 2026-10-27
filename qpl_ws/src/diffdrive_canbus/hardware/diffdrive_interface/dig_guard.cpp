#include <cmath>
#include <string>
#include <rclcpp/logging.hpp>

#include "diffdrive_canbus/diffdrive_interface.hpp"

namespace diffdrive_canbus {
  bool DigGuard::enabled = true;
  double DigGuard::pause_below_ratio = 0.8;
  double DigGuard::resume_above_ratio = 0.9;

  bool DigGuard::update(const CANDevice &drum)
  {
    if (!enabled) {
      lowering_allowed_ = true;
      return true;
    }

    // Drum velocity commands are a proportion of max RPM, the same unit Motor::write() scales by.
    const double set_rpm = drum.commanded_velocity() * MAX_MOTOR_RPM;
    const auto & telemetry = drum.telemetry();
    const bool feedback_fresh = telemetry.has_encoder_velocity &&
      std::chrono::steady_clock::now() - telemetry.encoder_velocity_time < DIG_GUARD_FEEDBACK_TIMEOUT;

    if (std::abs(set_rpm) < DIG_GUARD_MIN_SET_RPM) {
      lowering_allowed_ = false;
      reason_ = "drum is not set to spin";
    }
    else if (!feedback_fresh) {
      lowering_allowed_ = false;
      reason_ = "no recent drum speed feedback";
    }
    else {
      // Signed, so a drum turning the wrong way (or stalled backwards) never counts as at speed.
      const double ratio = telemetry.encoder_velocity_rpm / set_rpm;
      // Hysteresis: once paused, the drum must recover past a higher threshold before lowering resumes.
      lowering_allowed_ = ratio >= (lowering_allowed_ ? pause_below_ratio : resume_above_ratio);
      reason_ = "drum at " + std::to_string(static_cast<int>(std::round(ratio * 100.0))) + "% of set speed";
    }

    return lowering_allowed_;
  }

  void DigGuard::report(bool lowering_paused)
  {
    if (lowering_paused == reported_paused_) return;
    reported_paused_ = lowering_paused;

    if (lowering_paused) {
      RCLCPP_WARN(logger_, "Dig guard: lift lowering paused, %s", reason_.c_str());
    } else if (lowering_allowed_) {
      RCLCPP_INFO(logger_, "Dig guard: lift lowering resumed, %s", reason_.c_str());
    } else {
      RCLCPP_INFO(logger_, "Dig guard: lift no longer commanded down");
    }
  }
}
