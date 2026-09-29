#include <string>
#include <iostream>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>

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
  double Actuator::min_lift_mm = 0.0;
  double Actuator::max_lift_mm = 0.0;

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

  void Actuator::write() {
    const double setpoint_mm = preprocess_pos(commanded_pos_);
    go_to_position(setpoint_mm);
  }

  void Actuator::go_to_position(double setpoint_mm)
  {
    const double error_mm = setpoint_mm - filtered_position_mm_; // Positive if setpoint is greater than current position (i.e., need to go up)

    bool reached_position = error_mm > ACTUATOR_STOP_POSITION_MM - ACTUATOR_STOP_TOLERANCE_MM
      && error_mm < ACTUATOR_STOP_POSITION_MM + ACTUATOR_STOP_TOLERANCE_MM;

    // Only send position command if new position sent, to reduce bandwidth use.
    if (prev_setpoint_mm_ != setpoint_mm) {
      set_position(static_cast<float>(setpoint_mm));
      stop_sent_ = false;
    }
    // If the actuator has reached the commanded position, send a stop command to hold it in place.
    else if (reached_position && !stop_sent_) {
      set_duty_cycle(0.0f);
      stop_sent_ = true;
    }

    prev_setpoint_mm_ = setpoint_mm;
  }

  // Returns the commanded position in mm, clamped, and offset by the actuator position constant
  double Actuator::preprocess_pos(double position_m) {
    if (position_m == 0.0) {
      position_m = default_lift_mm / 1000.0; // convert to meters
    }
    position_m = std::clamp(position_m, min_lift_mm / 1000, max_lift_mm / 1000);

    return position_m * 1000.0 + ACTUATOR_POSITION_CONSTANT;
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



  double SynchronisedActuator::max_distance_between_points_metres;

  void SynchronisedActuator::write()
  {
    if (other_actuator_ == nullptr) {
      RCLCPP_ERROR(logger_, "Other actuator not set for synchronised actuator %s", name_.c_str());
      return;
    }

    if (is_dominant_actuator) {
      bool position_met = stop_sent_ && other_actuator_->stop_sent_;

      // Note that on startup, this condition isn't true until commands are sent. The default position therefore never kicks in.
      if (prev_commanded_pos_ != commanded_pos_) {
        resync_checkpoints = generate_resync_points(position_, commanded_pos_,
          SynchronisedActuator::max_distance_between_points_metres);
        iterator_ = resync_checkpoints.begin();
        iterator_initialised_ = true;
        prev_commanded_pos_ = commanded_pos_;
      }
      else if (position_met && std::next(iterator_) != resync_checkpoints.end()) {
        ++iterator_;
      }
    }
    else if (!other_actuator_->iterator_initialised_) {
      return;
    }

    const SynchronisedActuator* leader = is_dominant_actuator ? this : other_actuator_.get();
    if (leader->iterator_initialised_) {
      go_to_position(preprocess_pos(*leader->iterator_));
    }
  }

  std::vector<double> SynchronisedActuator::generate_resync_points(const double from, const double to, double max_distance_between_points) {
    if (max_distance_between_points <= 0.0 || from == to) {
      return {from, to};
    }

    std::vector<double> points;
    double distance = to - from;
    double direction = (distance >= 0) ? 1.0 : -1.0;
    double step = std::abs(max_distance_between_points) * direction;

    double checkpoint = from + step;
    while (std::abs(checkpoint - from) < std::abs(distance)) {
      points.push_back(checkpoint);
      checkpoint += step;
    }

    points.push_back(to);
    return points;
  }
}
