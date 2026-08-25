#pragma once

#include "diffdrive_canbus/can_comms.hpp"

#include <cstddef>
#include <hardware_interface/handle.hpp>
#include <rclcpp/logger.hpp>

namespace diffdrive_canbus
{

struct SparkMaxTelemetry
{
  bool has_encoder_velocity = false;
  float encoder_velocity_rpm = 0.0f;
  float motor_rad_per_sec = 0.0f;
  float wheel_rad_per_sec = 0.0f;

  bool has_encoder_position = false;
  float encoder_position_rotations = 0.0f;
  float wheel_position_rotations = 0.0f;

  bool has_applied_output = false;
  float applied_output = 0.0f;
};

class CANDevice
{
public:
  CANDevice(std::string name, const uint8_t &can_id, CANComms &can, float gear_ratio, rclcpp::Logger &logger);
  virtual ~CANDevice() = default; // TODO why do we need this?

  void configure();

  void virtual setup_ros_state_interfaces(std::vector<hardware_interface::StateInterface> &state_interfaces) {
    throw std::runtime_error("base setup_ros_state_interfaces used");
  }
  void virtual setup_ros_command_interfaces(std::vector<hardware_interface::CommandInterface> &command_interfaces) {
    throw std::runtime_error("base setup_ros_command_interfaces used");
  }

  std::string name() const { return name_; }
  uint8_t can_id() const { return can_id_; }
  float gear_ratio() const { return gear_ratio_; }

  const SparkMaxTelemetry & telemetry() const;
  float encoder_velocity_rpm() const;
  float motor_rad_per_sec() const;
  float wheel_rad_per_sec() const;
  float encoder_position_rotations() const;
  float wheel_position_rotations() const;
  float applied_output() const;

  virtual void write() { throw std::runtime_error("base write used"); }

  // TODO hopefully temporary declarations of motor functions; these can be removed once there is clearer separation of concerns
  virtual double commanded_velocity() const { return 0.0; }
  virtual double velocity() const { return 0.0; }
  virtual double rotation_position() const { return 0.0; }
  // TODO temporary declarations for actuators (see prev todo)
  virtual void request_actuator_status3_period(CANComms & can) { throw std::runtime_error("base request_actuator_status3_period used"); }
  virtual void update_joint_state(const CANFrame & frame) { throw std::runtime_error("base update_joint_state used"); }

  double clamp_and_apply_deadband_if_finite(double value, double deadband, double min = -1.0, double max = 1.0);
  bool send_heartbeats(bool print = false);
  bool clear_faults(bool print = false);

  bool set_duty_cycle(float duty);
  bool set_velocity_rad_per_sec(float target_wheel_rad_per_sec);

  void set_native_velocity_pid_slot(uint8_t pid_slot);

protected:
  static constexpr float PI = 3.14159265358979323846f;

  static constexpr uint8_t DEVICE_TYPE_MOTOR_CONTROLLER = 2;
  static constexpr uint8_t MANUFACTURER_REV = 5;

  static constexpr uint8_t API_CLASS_PERIODIC_STATUS = 46;
  static constexpr uint8_t API_INDEX_STATUS_0 = 0;
  static constexpr uint8_t API_INDEX_STATUS_1 = 1;
  static constexpr uint8_t API_INDEX_STATUS_2 = 2;
  static constexpr uint8_t API_INDEX_STATUS_5 = 5;

  static constexpr uint8_t API_CLASS_CLEAR_FAULTS = 6;
  static constexpr uint8_t API_INDEX_CLEAR_FAULTS = 14;

  static constexpr uint8_t API_CLASS_ROBORIO = 9;
  static constexpr uint8_t API_INDEX_ROBORIO_HEARTBEAT = 2;

  static constexpr uint8_t API_CLASS_NON_RIO = 11;
  static constexpr uint8_t API_INDEX_NON_RIO_HEARTBEAT = 2;

  static constexpr uint8_t HEARTBEAT_DEVICE_ID = 0;

  rclcpp::Logger logger_;

  std::string name_;
  uint8_t can_id_;
  CANComms & can_;
  float gear_ratio_;

  SparkMaxTelemetry telemetry_;

  uint8_t native_velocity_pid_slot_ = 0;

  static uint32_t make_sparkmax_id(
    uint8_t api_class,
    uint8_t api_index,
    uint8_t device_id);

  static uint32_t make_sparkmax_id_from_api_id(
    uint8_t api_id,
    uint8_t device_id);

  static float rpm_to_rad_per_sec(float rpm);
  static float rad_per_sec_to_rpm(float rad_per_sec);

  static void float_to_le_bytes(float value, uint8_t bytes[4]);
  static float le_bytes_to_float(const uint8_t data[8], size_t offset);
  static int16_t le_bytes_to_i16(const uint8_t data[8], size_t offset);

  bool set_status_period(uint16_t status_parameter_id, uint32_t period);

  bool send_setpoint(
    uint8_t api_class,
    uint8_t api_index,
    float setpoint,
    uint8_t pid_slot,
    bool print);

  bool send_simple_setpoint(
    uint8_t api_id,
    float setpoint);

  bool send_setpoint_with_control_type(
    uint8_t api_class,
    uint8_t api_index,
    float setpoint,
    uint8_t control_type,
    uint8_t pid_slot,
    bool print);
};

}  // namespace diffdrive_canbus