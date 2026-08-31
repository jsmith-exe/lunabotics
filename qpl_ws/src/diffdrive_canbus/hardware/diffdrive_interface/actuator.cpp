#include <string>
#include <iomanip>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>

#include "diffdrive_canbus/can_device.hpp"
#include "diffdrive_canbus/diffdrive_interface.hpp"


constexpr float ACTUATOR_TEST_POSITION_SETPOINT_MM = 200.0f;

constexpr double ACTUATOR_STOP_TOLERANCE_MM = 20.0;
constexpr double ACTUATOR_RESUME_TOLERANCE_MM = 40.0;

constexpr double RAW_MIN = 46.0;
constexpr double RAW_MAX = 318.0;
constexpr double DISTANCE_MIN_MM = 22.6;
constexpr double DISTANCE_MAX_MM = 228.0;

namespace diffdrive_canbus {
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
      &commanded_pos_mm_);
  }

  void Actuator::configure() {
    CANDevice::configure();
    set_status_period(2, STATUS3_PERIOD_MS);
  }

  void Actuator::write()
  {
      constexpr double ACTUATOR_POSITION_CONSTANT = 14.0f;
      const double setpoint_mm = ACTUATOR_TEST_POSITION_SETPOINT_MM + ACTUATOR_POSITION_CONSTANT;
      const double abs_error_mm = std::fabs(setpoint_mm - position_);

      if (!reached_position_ && abs_error_mm <= ACTUATOR_STOP_TOLERANCE_MM) {
          reached_position_ = true;
      }
      else if (reached_position_ && abs_error_mm >= ACTUATOR_RESUME_TOLERANCE_MM) {
          reached_position_ = false;
      }

      if (reached_position_ && !stop_sent_) {
          set_duty_cycle(0.0f);
          stop_sent_ = true;
          sleep_bus_gap();
      }
      else if (!reached_position_) {
          set_position(static_cast<float>(setpoint_mm));
          stop_sent_ = false;
          sleep_bus_gap();
      }
  }

  double Actuator::feedback_to_distance(uint16_t raw_voltage_feedback)
  {
      const double normalised = (static_cast<double>(raw_voltage_feedback) - RAW_MIN) / (RAW_MAX - RAW_MIN);

      const double clamped = std::clamp(normalised, 0.0, 1.0);

      return DISTANCE_MIN_MM + clamped * (DISTANCE_MAX_MM - DISTANCE_MIN_MM);
  }

  void Actuator::update_joint_state(const CANFrame & frame)
  {
      // CAN ID must match this device's and the frame must be status3.
      if (get_frc_device_id_from_can_id(frame.id) != can_id_
          || !is_actuator_status3_id(frame.id, can_id_))
      {
          return;
      }

      const uint16_t packed = le_u16_from_frame_data(frame.data, 0);
      const uint16_t raw_feedback = packed & 0x03FF;
      position_ = feedback_to_distance(raw_feedback);
  }
}
