#include <string>
#include <iomanip>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>

#include "diffdrive_canbus/can_device.hpp"
#include "diffdrive_canbus/diffdrive_interface.hpp"

constexpr double ACTUATOR_MIN_VOLTAGE = 0.279;
constexpr double ACTUATOR_MAX_VOLTAGE = 1.85;
constexpr double ACTUATOR_DEADBAND = 0.02;

// Temporary bench-test target. Both linear actuators receive this position
// setpoint. Change this value to the desired extension in millimetres.
constexpr float ACTUATOR_TEST_POSITION_MM = 75.0f + 14.0f;

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
    set_position(ACTUATOR_TEST_POSITION_MM);
    sleep_bus_gap();
  }

  double Actuator::normalise_actuator_voltage(double voltage, double min_voltage, double max_voltage)
  {
    const double span = max_voltage - min_voltage;

    if (!std::isfinite(voltage) || std::fabs(span) < 1e-9)
    {
      return 0.0;
    }

    return std::clamp((voltage - min_voltage) / span,0.0, 1.0);
  }

  void Actuator::update_joint_state(const CANFrame & frame)
  {
    if (get_frc_device_id_from_can_id(frame.id) != can_id_)
    {
      return;
    }

    // SPARK MAX analogue sensor data is expected on Periodic Status 3.
    // Use the exact Status 3 CAN ID instead of only the API index so Status 0/1/2
    // frames cannot be mistaken for analogue feedback.
    if (!is_actuator_status3_id(frame.id, can_id_))
    {
      return;
    }

    // Per the non-FRC SPARK MAX CAN reference, Periodic Status 3 starts with
    // adcVoltage in 2q8 fixed-point format. That means the raw 16-bit
    // little-endian value is voltage * 256.
    const uint16_t raw_adc_voltage_2q8 = le_u16_from_frame_data(frame.data, 0);
    const double analog_voltage = static_cast<double>(raw_adc_voltage_2q8) / 256.0;
    voltage_ = analog_voltage;
    position_ = normalise_actuator_voltage(analog_voltage, ACTUATOR_MIN_VOLTAGE, ACTUATOR_MAX_VOLTAGE);
  }
}
