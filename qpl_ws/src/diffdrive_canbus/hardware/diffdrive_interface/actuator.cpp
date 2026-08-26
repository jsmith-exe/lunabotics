#include <string>
#include <iomanip>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>

#include "diffdrive_canbus/can_device.hpp"
#include "diffdrive_canbus/diffdrive_interface.hpp"


// Temporary bench-test target. Both linear actuators receive this position
// setpoint. Change this value to the desired extension in millimetres.
constexpr float ACTUATOR_TEST_POSITION_SETPOINT_MM = 45.0f;
constexpr float ACTUATOR_TEST_POSITION_MM = ACTUATOR_TEST_POSITION_SETPOINT_MM + 14.0f;

constexpr double ACTUATOR_STOP_TOLERANCE_MM = 20.0;
constexpr double ACTUATOR_RESUME_TOLERANCE_MM = 40.0;

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
      &command_);
  }


  void Actuator::request_actuator_status3_period(CANComms & can_)
  {
    const uint32_t status3_id = SPARKMAX_PERIODIC_STATUS_3_BASE_ID + static_cast<uint32_t>(can_id_);

    std::vector<uint8_t> data(2, 0x00);
    data[0] = static_cast<uint8_t>(LINEAR_ACTUATOR_STATUS3_PERIOD_MS & 0xFF);
    data[1] = static_cast<uint8_t>((LINEAR_ACTUATOR_STATUS3_PERIOD_MS >> 8) & 0xFF);

    can_.send_extended_frame(status3_id, data, false);

    sleep_bus_gap();
  }

  void Actuator::write()
  {
      const double setpoint_mm = ACTUATOR_TEST_POSITION_MM;

      const double error_mm = setpoint_mm - position_;
      const double abs_error_mm = std::fabs(error_mm);

      if (!holding_position_)
      {
          // Enter the deadband
          if (abs_error_mm <= ACTUATOR_STOP_TOLERANCE_MM)
          {
              holding_position_ = true;

              // Stop driving the actuator
              set_duty_cycle(0.0f);
          }
          else
          {
              // Continue using the SPARK MAX position controller
              set_position(static_cast<float>(setpoint_mm));
          }
      }
      else
      {
          // Already stopped near the setpoint.
          // Only start correcting again if we move significantly away.
          if (abs_error_mm >= ACTUATOR_RESUME_TOLERANCE_MM)
          {
              holding_position_ = false;

              set_position(static_cast<float>(setpoint_mm));
          }
          else
          {
              set_duty_cycle(0.0f);
          }
      }

      sleep_bus_gap();
  }

  double Actuator::feedback_to_distance(uint16_t raw_feedback)
  {
      constexpr double RAW_MIN = 46.0;
      constexpr double RAW_MAX = 318.0;

      constexpr double DISTANCE_MIN_MM = 22.6;
      constexpr double DISTANCE_MAX_MM = 228.0;

      const double normalised =
          (static_cast<double>(raw_feedback) - RAW_MIN) /
          (RAW_MAX - RAW_MIN);

      const double clamped =
          std::clamp(normalised, 0.0, 1.0);

      return DISTANCE_MIN_MM +
          clamped * (DISTANCE_MAX_MM - DISTANCE_MIN_MM);
  }

  void Actuator::update_joint_state(const CANFrame & frame)
  {
      if (get_frc_device_id_from_can_id(frame.id) != can_id_)
      {
          return;
      }

      if (!is_actuator_status3_id(frame.id, can_id_))
      {
          return;
      }

      const uint16_t packed =
          le_u16_from_frame_data(frame.data, 0);

      const uint16_t raw_feedback =
          packed & 0x03FF;

      position_ = feedback_to_distance(raw_feedback);
  }
}
