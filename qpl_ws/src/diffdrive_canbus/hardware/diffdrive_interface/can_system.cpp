#include <map>
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

  void Motor::update_joint_state_from_telemetry(
    double & position_offset_rad,
    bool & position_offset_valid)
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

      if (!position_offset_valid)
      {
        position_offset_rad = absolute_position_rad;
        position_offset_valid = true;
      }

      rotation_position_ = absolute_position_rad - position_offset_rad;
    }
  }




  CANSystem::CANSystem(CANComms &comms) : comms_(comms) {
  }

  void CANSystem::add_device(const std::unique_ptr<CANDevice> &device) {
    devices_[device->can_id()] = device.get();
  }

  void CANSystem::setup_ros_state_interfaces(std::vector<hardware_interface::StateInterface> &state_interfaces) {
    for (auto & [can_id, device] : devices_) {
      device->setup_ros_state_interfaces(state_interfaces);
    }
  }

  void CANSystem::setup_ros_command_interfaces(std::vector<hardware_interface::CommandInterface> &command_interfaces) {
    for (auto & [can_id, device] : devices_) {
      device->setup_ros_command_interfaces(command_interfaces);
    }
  }
}
