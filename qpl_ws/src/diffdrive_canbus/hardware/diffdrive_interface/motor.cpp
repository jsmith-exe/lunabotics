#include <iostream>
#include <string>
#include <thread>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/logging.hpp>

#include "diffdrive_canbus/can_comms.hpp"
#include "diffdrive_canbus/diffdrive_interface.hpp"
#include "diffdrive_canbus/can_device.hpp"


namespace diffdrive_canbus {
  void Motor::setup_ros_state_interfaces(std::vector<hardware_interface::StateInterface> &state_interfaces) {
    state_interfaces.emplace_back(
      name_,
      hardware_interface::HW_IF_POSITION,
      &rotation_position_);

    state_interfaces.emplace_back(
      name_,
      hardware_interface::HW_IF_VELOCITY,
      &velocity_);
  }

  void Motor::setup_ros_command_interfaces(std::vector<hardware_interface::CommandInterface> &command_interfaces) {
    command_interfaces.emplace_back(
      name_,
      hardware_interface::HW_IF_VELOCITY,
      &commanded_velocity_);
  }

  void Motor::update_joint_state(const can_frame &frame)
  {
    handle_status_frame(frame);
    const auto & tel = this->telemetry();

    if (tel.has_encoder_velocity)
    {
      velocity_ = static_cast<double>(tel.wheel_rad_per_sec);
    }

    if (tel.has_encoder_position)
    {
      const double absolute_position_rad =
        static_cast<double>(tel.wheel_position_rotations) * TWO_PI;

      if (!position_offset_valid_)
      {
        position_offset_rad_ = absolute_position_rad;
        position_offset_valid_ = true;
      }

      rotation_position_ = absolute_position_rad - position_offset_rad_;
    }
  }

  void Motor::write()
  {
    // Calculate time since last write
    const auto now = std::chrono::system_clock::now();
    const auto time_since_last_write = now - last_write_time_;
    last_write_time_ = now;
    // To ms
    const auto ms_since_last_write = std::chrono::duration_cast<std::chrono::milliseconds>(time_since_last_write);
    const double seconds_since_last_write = ms_since_last_write.count() / 1000.0;
    
    const double max_velocity_change = seconds_since_last_write * rate_of_velocity_change_;
    // Set velocity to commanded if it's within the max velocity change
    if (max_velocity_change >= abs(commanded_velocity_ - smoothed_velocity_))
    {
      smoothed_velocity_ = commanded_velocity_;
    }
    else // Otherwise add/subtract
    {
      if (commanded_velocity_ < smoothed_velocity_) { // If we need to slow down to meet commanded vel
        smoothed_velocity_ -= max_velocity_change;
      }
      else { // We need to speed up to meet commanded vel
        smoothed_velocity_ += max_velocity_change;
      }
    }
    const double velocity_to_write = smoothed_velocity_;

    // Limit writes to significant changes
    const bool new_zero_sent = velocity_to_write == 0.0 && prev_commanded_velocity_ != 0.0;
    const bool insignificant_change = abs(velocity_to_write - prev_commanded_velocity_) < MIN_VELOCITY_CHANGE;
    if (!new_zero_sent && insignificant_change)
    {
      return;
    }
    prev_commanded_velocity_ = velocity_to_write;

    if (velocity_to_write == 0.0) {
      this->set_duty_cycle(0.0);
    }
    else {
      this->set_velocity_rad_per_sec(static_cast<float>(velocity_to_write));
    }
  }

  // Returns true if decoded
  bool Motor::handle_status_frame(const can_frame &frame)
  {
    const auto fields = parse_frc_extended_can_id(frame.can_id);

    if (fields.device_id != can_id_ || fields.api_class != API_CLASS_PERIODIC_STATUS)
    {
      return false;
    }

    if (fields.api_index == API_INDEX_STATUS_0)
    {
      if (frame.can_dlc < 2) return false;
      const int16_t raw_output = le_bytes_to_i16(frame.data, 0);
      telemetry_.applied_output = static_cast<float>(raw_output) / 32767.0f;
    }

    else if (fields.api_index == API_INDEX_STATUS_1)
    {
      // Firmware 24:
      // Status 1, bytes 0-3 = encoder velocity RPM.
      if (frame.can_dlc < 4) return false;
      const float encoder_velocity_rpm = le_bytes_to_float(frame.data, 0);

      if (!std::isfinite(encoder_velocity_rpm)) return false;
      telemetry_.encoder_velocity_rpm = encoder_velocity_rpm;
      telemetry_.motor_rad_per_sec = rpm_to_rad_per_sec(encoder_velocity_rpm);
      telemetry_.wheel_rad_per_sec = telemetry_.motor_rad_per_sec / gear_ratio_;
      telemetry_.has_encoder_velocity = true;
    }

    else if (fields.api_index == API_INDEX_STATUS_2)
    {
      // Firmware 24:
      // Status 2, bytes 0-3 = encoder position rotations.
      if (frame.can_dlc < 4) return false;
      const float encoder_position_rotations = le_bytes_to_float(frame.data, 0);

      if (!std::isfinite(encoder_position_rotations)) return false;
      telemetry_.encoder_position_rotations = encoder_position_rotations;
      telemetry_.wheel_position_rotations = encoder_position_rotations / gear_ratio_;
      telemetry_.has_encoder_position = true;
    }
    else {
      return false;
    }

    return true;
  }
}