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
    // TODO use clamp_and_apply_deadband_if_finite

    const double target_motor_rpm = commanded_velocity_ * gear_ratio_ * 60.0 / TWO_PI;

    if (detect_runaway(target_motor_rpm))
    {
      RCLCPP_WARN(logger_, "Runaway detected, enabling latch.");
      CANSystem::runaway_latched_ = true;
      this->set_duty_cycle(0.0f);
      return;
    }
    if (CANSystem::runaway_latched_ && velocity_ <= SAFE_STOPPED_VELOCITY)
    {
      RCLCPP_WARN(logger_, "Velocity safe, disabling latch.");
      this->set_duty_cycle(0.0f);
      CANSystem::runaway_latched_ = false;
      return;
    }
    if (CANSystem::runaway_latched_) { return; }

    this->set_velocity_rad_per_sec(static_cast<float>(commanded_velocity_));
  }

  bool Motor::detect_runaway(double target_motor_rpm)
  {
    const auto & tel = this->telemetry();

    if (!tel.has_encoder_velocity)
    {
      return false;
    }

    const double measured_rpm = tel.encoder_velocity_rpm;

    const double rpm_error =
      measured_rpm - target_motor_rpm;

    const bool target_small =
      std::fabs(target_motor_rpm) < RUNAWAY_SMALL_TARGET_RPM;

    const bool measured_large =
      std::fabs(measured_rpm) > RUNAWAY_MIN_MEASURED_RPM;

    const bool huge_error =
      std::fabs(rpm_error) > RUNAWAY_ALLOWED_RPM_ERROR;

    const bool sign_opposed =
      std::fabs(target_motor_rpm) > RUNAWAY_SIGN_TARGET_MIN_RPM &&
      std::fabs(measured_rpm) > RUNAWAY_MIN_MEASURED_RPM &&
      ((target_motor_rpm > 0.0 && measured_rpm < 0.0) ||
       (target_motor_rpm < 0.0 && measured_rpm > 0.0));

    const bool high_applied =
      tel.has_applied_output &&
      std::fabs(static_cast<double>(tel.applied_output)) >
        RUNAWAY_HIGH_APPLIED_OUTPUT;

    const bool runaway =
      (target_small && measured_large) ||
      (huge_error && measured_large && high_applied) ||
      sign_opposed;

    return runaway;
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
                << CANComms::frame_to_string(frame)
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