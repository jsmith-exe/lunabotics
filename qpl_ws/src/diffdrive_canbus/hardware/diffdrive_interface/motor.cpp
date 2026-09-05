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
  constexpr double SAFE_STOPPED_VELOCITY = 0.1;

  // Firmware 24 status frame layout:
  //   Status 0: 0x2051800 + device_id
  //   Status 1: 0x2051840 + device_id
  //   Status 2: 0x2051880 + device_id
  //
  // Firmware 25+ status frame layout seen in earlier logs:
  //   Status 0: 0x205B800 + device_id
  //   Status 1: 0x205B840 + device_id
  //   Status 2: 0x205B880 + device_id
  constexpr uint8_t API_CLASS_PERIODIC_STATUS_FIRMWARE_24 = 6;
  constexpr uint8_t API_CLASS_PERIODIC_STATUS_FIRMWARE_25_PLUS = 46;

  constexpr int TELEMETRY_EMPTY_READ_RETRIES = 8;
  constexpr auto TELEMETRY_EMPTY_READ_DELAY = std::chrono::milliseconds(1);

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

  void Motor::update_joint_state(const CANFrame & frame)
  {
    handle_status_frame(frame, false);
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

  bool Motor::handle_status_frame(const CANFrame & frame, bool print_status_frame)
  {
    const auto fields = parse_frc_extended_can_id(frame.id);

    if (fields.device_type != DEVICE_TYPE_MOTOR_CONTROLLER ||
        fields.manufacturer != MANUFACTURER_REV ||
        fields.device_id != can_id_)
    {
      return false;
    }

    const bool is_firmware24_status =
      fields.api_class == API_CLASS_PERIODIC_STATUS_FIRMWARE_24;

    const bool is_firmware25_status =
      fields.api_class == API_CLASS_PERIODIC_STATUS_FIRMWARE_25_PLUS;

    if (!is_firmware24_status && !is_firmware25_status)
    {
      return false;
    }

    if (print_status_frame)
    {
      std::cout << "RX SPARK STATUS: "
                << frame_to_string(frame)
                << " | api_class=" << static_cast<int>(fields.api_class)
                << " api_index=" << static_cast<int>(fields.api_index)
                << " device_id=" << static_cast<int>(fields.device_id);

      if (is_firmware24_status)
      {
        std::cout << " | style=firmware24";
      }
      else
      {
        std::cout << " | style=firmware25+";
      }

      std::cout << "\n";
    }

    if (fields.api_index == API_INDEX_STATUS_0)
    {
      if (frame.dlc >= 2)
      {
        const int16_t raw_output =
          le_bytes_to_i16(frame.data, 0);

        telemetry_.applied_output =
          static_cast<float>(raw_output) / 32767.0f;

        telemetry_.has_applied_output = true;
      }

      return true;
    }

    if (is_firmware24_status && fields.api_index == API_INDEX_STATUS_1)
    {
      // Firmware 24:
      // Status 1, bytes 0-3 = encoder velocity RPM.
      //
      // Example from your logs:
      //   DATA=[0x39 0x8A 0xF8 0x43 ...] -> about 497 RPM
      // while commanding 500 RPM.
      if (frame.dlc >= 4)
      {
        const float encoder_velocity_rpm =
          le_bytes_to_float(frame.data, 0);

        if (std::isfinite(encoder_velocity_rpm))
        {
          telemetry_.encoder_velocity_rpm =
            encoder_velocity_rpm;

          telemetry_.motor_rad_per_sec =
            rpm_to_rad_per_sec(encoder_velocity_rpm);

          telemetry_.wheel_rad_per_sec =
            telemetry_.motor_rad_per_sec / gear_ratio_;

          telemetry_.has_encoder_velocity = true;
        }
      }

      return true;
    }

    if (is_firmware24_status && fields.api_index == API_INDEX_STATUS_2)
    {
      // Firmware 24:
      // Status 2, bytes 0-3 = encoder position rotations.
      //
      // Example from your logs:
      //   DATA=[0xB8 0xC2 0xB0 0x44 ...] -> about 1414 rotations
      // and this value increases over time.
      if (frame.dlc >= 4)
      {
        const float encoder_position_rotations =
          le_bytes_to_float(frame.data, 0);

        if (std::isfinite(encoder_position_rotations))
        {
          telemetry_.encoder_position_rotations =
            encoder_position_rotations;

          telemetry_.wheel_position_rotations =
            encoder_position_rotations / gear_ratio_;

          telemetry_.has_encoder_position = true;
        }
      }

      return true;
    }

    if (is_firmware25_status && fields.api_index == API_INDEX_STATUS_1)
    {
      // Not decoding firmware 25 status 1 yet.
      return true;
    }

    if (is_firmware25_status && fields.api_index == API_INDEX_STATUS_2)
    {
      // Firmware 25+ assumption from your earlier logs:
      // Status 2, bytes 0-3 = velocity RPM
      // Status 2, bytes 4-7 = position rotations
      if (frame.dlc >= 8)
      {
        const float encoder_velocity_rpm =
          le_bytes_to_float(frame.data, 0);

        const float encoder_position_rotations =
          le_bytes_to_float(frame.data, 4);

        if (std::isfinite(encoder_velocity_rpm))
        {
          telemetry_.encoder_velocity_rpm =
            encoder_velocity_rpm;

          telemetry_.motor_rad_per_sec =
            rpm_to_rad_per_sec(encoder_velocity_rpm);

          telemetry_.wheel_rad_per_sec =
            telemetry_.motor_rad_per_sec / gear_ratio_;

          telemetry_.has_encoder_velocity = true;
        }

        if (std::isfinite(encoder_position_rotations))
        {
          telemetry_.encoder_position_rotations =
            encoder_position_rotations;

          telemetry_.wheel_position_rotations =
            encoder_position_rotations / gear_ratio_;

          telemetry_.has_encoder_position = true;
        }
      }

      return true;
    }

    // Any other status frame from this SPARK MAX is valid traffic,
    // but we are not decoding it yet.
    return true;
  }
}