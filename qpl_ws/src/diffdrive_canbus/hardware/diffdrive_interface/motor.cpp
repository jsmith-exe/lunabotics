#include <iostream>
#include <string>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "diffdrive_canbus/can_comms.hpp"
#include "diffdrive_canbus/diffdrive_canbus_system.hpp"
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

  void Motor::update_joint_state_from_telemetry()
  {
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

  void Motor::write_one_motor_native_velocity()
  {
    // TODO validate?
    const double target_motor_rpm = commanded_velocity_ * gear_ratio_ * 60.0 / TWO_PI;
    // if (detect_runaway(target_motor_rpm))
    // {
    //   CANSystem::runaway_latched_ = true;
    //   this->set_duty_cycle(0.0f);
    //   return;
    // }
    //
    // if (CANSystem::runaway_latched_)
    // {
    //   return;
    // }

    this->send_heartbeat_before_motor_command();
    this->set_velocity_rad_per_sec(static_cast<float>(commanded_velocity_), false);
    sleep_bus_gap();
  }

  void Motor::send_heartbeat_before_motor_command()
  {
    this->send_heartbeats(false);
    sleep_bus_gap();
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
}