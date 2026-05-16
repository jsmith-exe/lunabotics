#ifndef DIFFDRIVE_CANBUS__DIFFDRIVE_CANBUS_SYSTEM_HPP_
#define DIFFDRIVE_CANBUS__DIFFDRIVE_CANBUS_SYSTEM_HPP_

#include "diffdrive_canbus/CAN_comms.hpp"
#include "diffdrive_canbus/spark_max.hpp"
#include "diffdrive_canbus/visibility_control.h"
#include "diffdrive_canbus/wheel.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace diffdrive_canbus
{

class DIFFDRIVE_CANBUS_PUBLIC DiffDriveCanbusHardware
  : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  struct Config
  {
    std::string device;

    int serial_baud_rate = 2000000;
    int can_baud_rate = 1000000;
    int timeout_ms = 1;

    uint8_t front_left_can_id = 1;
    uint8_t front_right_can_id = 1;
    uint8_t rear_left_can_id = 1;
    uint8_t rear_right_can_id = 1;

    double gear_ratio = 1.0;

    bool invert_front_left = false;
    bool invert_front_right = false;
    bool invert_rear_left = false;
    bool invert_rear_right = false;

    std::size_t telemetry_frames_per_read = 50;
  };

  static double sign(bool inverted);

  Config cfg_;

  Wheel front_left_wheel_;
  Wheel front_right_wheel_;
  Wheel rear_left_wheel_;
  Wheel rear_right_wheel_;

  CANComms can_;

  // Temporary single-motor bring-up:
  // only the front-left ros2_control joint is connected to a real SPARK MAX.
  std::unique_ptr<SparkMax> front_left_spark_;
};

}  // namespace diffdrive_canbus

#endif  // DIFFDRIVE_CANBUS__DIFFDRIVE_CANBUS_SYSTEM_HPP_