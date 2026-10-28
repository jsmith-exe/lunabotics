#include <string>
#include <iostream>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>

#include "diffdrive_canbus/can_device.hpp"
#include "diffdrive_canbus/diffdrive_interface.hpp"

constexpr double RAW_MIN = 7.0;
constexpr double RAW_MAX = 694.0;
constexpr double DISTANCE_MIN_MM = 30.0;
constexpr double DISTANCE_MAX_MM = 220.0;
constexpr double ACTUATOR_SETPOINT_OFFSET_MM = -27.9; // Controller setpoints are offset from feedback
constexpr double ACTUATOR_STOP_TOLERANCE_MM = 3.0; // Actuator is stopped when within this distance of its target

// Protects against spikes.
double low_pass_filter(double prev_val, double new_val, double alpha) {
  return alpha * prev_val + (1 - alpha) * new_val;
}

namespace diffdrive_canbus {
  double Actuator::min_lift_mm = 0.0;
  double Actuator::max_lift_mm = 0.0;

  void Actuator::setup_ros_state_interfaces(std::vector<hardware_interface::StateInterface> &state_interfaces) {
    state_interfaces.emplace_back(
      this->name_,
      hardware_interface::HW_IF_POSITION,
      &position_);
  }

  void Actuator::setup_ros_command_interfaces(std::vector<hardware_interface::CommandInterface> &command_interfaces) {
    command_interfaces.emplace_back(
      this->name_,
      hardware_interface::HW_IF_POSITION,
      &commanded_pos_);
  }

  void Actuator::configure() {
    CANDevice::configure();
    set_status_period(2, ACTUATOR_STATUS2_PERIOD_MS);
    set_status_period(4, UNUSED_STATUS_PERIOD_MS);
  }

  void Actuator::write() {
    go_to_position(commanded_pos_);
  }

  void Actuator::go_to_position(double setpoint_m)
  {
    // Wait for feedback first. This is to prevent restarted motor controllers from retracting actuators
    if (!received_position_) return;

    // Periodically resend whichever command is current, in case the last one was lost on the bus.
    const auto now = std::chrono::steady_clock::now();
    if (now - last_resend_time_ >= ACTUATOR_RESEND_PERIOD) {
      last_resend_time_ = now;
      if (stop_sent_) {
        stop_sent_ = false;
      } else {
        resend_setpoint_ = true;
      }
    }

    // Stop moving if 0 sent
    if (setpoint_m == 0.0) {
      stop();
      resend_setpoint_ = true;
      return;
    }

    const double target_mm = std::clamp(setpoint_m * 1000.0, min_lift_mm, max_lift_mm);

    // Only send position command if new position sent, to reduce bandwidth use.
    if (resend_setpoint_ || target_mm != prev_target_mm_) {
      // On failure the flags stay set, so the setpoint is retried next write.
      if (set_position(static_cast<float>(target_mm + ACTUATOR_SETPOINT_OFFSET_MM))) {
        stop_sent_ = false;
        resend_setpoint_ = false;
        prev_target_mm_ = target_mm;
      }
    }
    // If the actuator has reached the commanded position, send a stop command to hold it in place.
    else if (std::abs(target_mm - filtered_position_mm_) < ACTUATOR_STOP_TOLERANCE_MM) {
      stop();
    }
  }

  void Actuator::stop()
  {
    if (stop_sent_) return;
    stop_sent_ = set_duty_cycle(0.0); // Retried next write if the send failed.
  }

  double Actuator::feedback_to_distance(uint16_t raw_voltage_feedback)
  {
    const double normalised = (static_cast<double>(raw_voltage_feedback) - RAW_MIN) / (RAW_MAX - RAW_MIN);

    const double clamped = std::clamp(normalised, 0.0, 1.0);

    return DISTANCE_MIN_MM + clamped * (DISTANCE_MAX_MM - DISTANCE_MIN_MM);
  }

  void Actuator::update_joint_state(const can_frame & frame)
  {
    // CAN ID must match this device's and the frame must be status3.
    if (get_frc_device_id_from_can_id(frame.can_id) != can_id_
        || !is_actuator_status3_id(frame.can_id, can_id_))
    {
      return;
    }

    const uint16_t packed = le_u16_from_frame_data(frame.data, 0);
    const uint16_t raw_feedback = packed & 0x03FF;
    position_ = feedback_to_distance(raw_feedback) / 1000.0; // convert to meters

    const double position_mm = position_ * 1000.0;

    // Seed the filter with the first reading, so it doesn't start from 0.
    if (!received_position_) {
      filtered_position_mm_ = position_mm;

      // After a power cycle the controllers' setpoint is 0, so hold the current position to stop the actuators retracting.
      commanded_pos_ = position_;
      received_position_ = true;
    }
    else {
      filtered_position_mm_ = low_pass_filter(filtered_position_mm_, position_mm, ACTUATOR_POSITION_LOW_PASS_ALPHA);
    }
  }





  double SynchronisedActuator::max_distance_between_points_metres;

  void SynchronisedActuator::write()
  {
    // The dominant actuator commands both actuators.
    if (!is_dominant_actuator) return;

    if (other_actuator_ == nullptr) {
      RCLCPP_ERROR(logger_, "Other actuator not set for dominant synchronised actuator %s", name_.c_str());
      return;
    }

    if (prev_commanded_pos_ != commanded_pos_) {
      checkpoints_ = generate_resync_points(position_, commanded_pos_, max_distance_between_points_metres);
      checkpoint_index_ = 0;
      prev_commanded_pos_ = commanded_pos_;
    }
    // Move on to the next checkpoint once both actuators have reached the current one.
    else if (stop_sent_ && other_actuator_->stop_sent_ && checkpoint_index_ + 1 < checkpoints_.size()) {
      ++checkpoint_index_;
    }

    if (!checkpoints_.empty()) {
      go_to_position(checkpoints_[checkpoint_index_]);
      other_actuator_->go_to_position(checkpoints_[checkpoint_index_]);
    }
  }

  std::vector<double> SynchronisedActuator::generate_resync_points(const double from, const double to, double max_distance_between_points) {
    if (to == 0.0) {
      return {0.0};
    }
    if (max_distance_between_points <= 0.0 || from == to) {
      return {from, to};
    }

    std::vector<double> points;
    double distance = to - from;
    double direction = (distance >= 0) ? 1.0 : -1.0;
    double step = std::abs(max_distance_between_points) * direction;

    double checkpoint = from + step;
    while (std::abs(checkpoint - from) < std::abs(distance)) {
      points.push_back(checkpoint);
      checkpoint += step;
    }

    points.push_back(to);
    return points;
  }
}
