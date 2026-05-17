#pragma once

#include "diffdrive_canbus/CAN_comms.hpp"

#include <cstddef>
#include <cstdint>

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

class SparkMax
{
public:
  SparkMax(CANComms & can, uint8_t device_id, float gear_ratio = 1.0f);

  uint8_t device_id() const;
  float gear_ratio() const;
  void set_gear_ratio(float gear_ratio);

  bool send_heartbeats(bool print = false);
  bool clear_faults(bool print = false);

  bool set_duty_cycle(float duty, bool print = false);

  bool stop(bool print = false);

  bool read_telemetry(int max_frames = 20, bool print_status_frames = false);

  bool handle_status_frame(
    const CANFrame & frame,
    bool print_status_frame);

  const SparkMaxTelemetry & telemetry() const;

  float encoder_velocity_rpm() const;
  float motor_rad_per_sec() const;
  float wheel_rad_per_sec() const;
  float encoder_position_rotations() const;
  float wheel_position_rotations() const;
  float applied_output() const;

  void set_native_velocity_pid_slot(uint8_t pid_slot);

  bool set_velocity_rpm(
    float target_motor_rpm,
    bool print = false);

  bool set_velocity_rad_per_sec(
    float target_wheel_rad_per_sec,
    bool print = false);

  bool set_native_velocity_rpm(
    float target_motor_rpm,
    bool print = false);

  bool set_native_velocity_rad_per_sec(
    float target_wheel_rad_per_sec,
    bool print = false);

  bool debug_send_setpoint_api(
    uint8_t api_class,
    uint8_t api_index,
    float setpoint,
    uint8_t pid_slot = 0,
    bool print = false);

  void reset_velocity_controller();

  float velocity_controller_output() const;

private:
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

  CANComms & can_;
  uint8_t device_id_;
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
  static float clamp_duty(float duty);

  static void float_to_le_bytes(float value, uint8_t bytes[4]);
  static float le_bytes_to_float(const uint8_t data[8], size_t offset);
  static int16_t le_bytes_to_i16(const uint8_t data[8], size_t offset);

  bool send_setpoint(
    uint8_t api_class,
    uint8_t api_index,
    float setpoint,
    uint8_t pid_slot,
    bool print);

  bool send_simple_setpoint(
    uint8_t api_id,
    float setpoint,
    bool print);

  bool send_setpoint_with_control_type(
    uint8_t api_class,
    uint8_t api_index,
    float setpoint,
    uint8_t control_type,
    uint8_t pid_slot,
    bool print);

  bool parse_status_frame(
    const CANFrame & frame,
    bool print_status_frame);
};

}  // namespace diffdrive_canbus