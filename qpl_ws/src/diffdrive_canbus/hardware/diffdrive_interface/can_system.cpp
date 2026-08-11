#include <map>
#include <hardware_interface/handle.hpp>

#include "diffdrive_canbus/can_comms.hpp"
#include "diffdrive_canbus/diffdrive_interface.hpp"
#include "diffdrive_canbus/can_device.hpp"


namespace diffdrive_canbus {
  bool CANSystem::runaway_latched_ = false;

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

  void CANSystem::update_joint_state(CANFrame &frame) {
    for (auto & [can_id, device] : devices_) {
      device->update_joint_state(frame);
    }
  }

  void CANSystem::send_zero_duty_all() {
    for (auto & [can_id, device] : devices_) {
      device->set_duty_cycle(0.0f);
    }
  }

  void CANSystem::send_heartbeat() {
    // Send heartbeat from arbitrary device - in this case, whichever was added first.
    devices_.begin()->second->send_heartbeats(false);
  }
}
