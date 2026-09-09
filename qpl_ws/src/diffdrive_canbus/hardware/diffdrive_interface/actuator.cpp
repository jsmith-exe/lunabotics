#include <string>
#include <iomanip>
#include <iostream>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>

#include "diffdrive_canbus/can_device.hpp"
#include "diffdrive_canbus/diffdrive_interface.hpp"

constexpr double RAW_MIN = 46.0;
constexpr double RAW_MAX = 318.0;
constexpr double DISTANCE_MIN_MM = 22.6;
constexpr double DISTANCE_MAX_MM = 228.0;
constexpr double ACTUATOR_POSITION_CONSTANT = 14.0f; // Commands to the actuators must be offset

// The feedback is off by about 12-13mm (possibly the ACTUATOR_POSITION_CONSTANT above).
// Actuator should be stopped at STOP_POSITION +/- STOP_TOLERANCE.
constexpr double ACTUATOR_STOP_POSITION_MM = 13.0;
constexpr double ACTUATOR_STOP_TOLERANCE_MM = 3.0;

// Protects against spikes.
double low_pass_filter(double prev_val, double new_val, double alpha) {
  return alpha * prev_val + (1 - alpha) * new_val;
}

namespace diffdrive_canbus {
  double Actuator::default_lift_mm = 0.0;

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
      &commanded_pos_);
  }

  void Actuator::configure() {
    CANDevice::configure();
    set_status_period(2, STATUS3_PERIOD_MS);
  }

  void Actuator::write()
  {
    if (commanded_pos_ == 0.0) {
      commanded_pos_ = Actuator::default_lift_mm / 1000.0; // convert to meters
    }

    const double setpoint_mm = commanded_pos_ * 1000.0 + ACTUATOR_POSITION_CONSTANT;
    const double error_mm = setpoint_mm - filtered_position_mm_; // Positive if setpoint is greater than current position (i.e., need to go up)

    bool reached_position = error_mm > ACTUATOR_STOP_POSITION_MM - ACTUATOR_STOP_TOLERANCE_MM
      && error_mm < ACTUATOR_STOP_POSITION_MM + ACTUATOR_STOP_TOLERANCE_MM;

    // Only send position command if new position sent, to reduce bandwidth use.
    if (prev_commanded_pos_ != commanded_pos_) {
      set_position(static_cast<float>(setpoint_mm));
      stop_sent_ = false;
    }
    // If the actuator has reached the commanded position, send a stop command to hold it in place.
    else if (reached_position && !stop_sent_) {
      set_duty_cycle(0.0f);
      stop_sent_ = true;
    }

    prev_commanded_pos_ = commanded_pos_;
  }

  double Actuator::feedback_to_distance(uint16_t raw_voltage_feedback)
  {
    const double normalised = (static_cast<double>(raw_voltage_feedback) - RAW_MIN) / (RAW_MAX - RAW_MIN);

    const double clamped = std::clamp(normalised, 0.0, 1.0);

    return DISTANCE_MIN_MM + clamped * (DISTANCE_MAX_MM - DISTANCE_MIN_MM);
  }

  void Actuator::update_joint_state(const can_frame & frame)
  {
    // CAN ID must match this device's and the frame must be status3.
    if (get_frc_device_id_from_can_id(frame.can_id) != can_id_
        || !is_actuator_status3_id(frame.can_id, can_id_))
    {
      return;
    }

    const uint16_t packed = le_u16_from_frame_data(frame.data, 0);
    const uint16_t raw_feedback = packed & 0x03FF;
    previous_position_ = position_;
    position_ = feedback_to_distance(raw_feedback) / 1000.0; // convert to meters
    filtered_position_mm_ = low_pass_filter(previous_position_, position_, ACTUATOR_POSITION_LOW_PASS_ALPHA) * 1000.0;
  }
}
