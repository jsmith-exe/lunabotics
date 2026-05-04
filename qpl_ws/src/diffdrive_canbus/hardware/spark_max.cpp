#include "diffdrive_canbus/spark_max.hpp"

#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

namespace diffdrive_canbus
{

SparkMax::SparkMax(CANComms & can, uint8_t device_id, float gear_ratio)
: can_(can),
  device_id_(device_id),
  gear_ratio_(gear_ratio),
  velocity_controller_(
    0.0015f,    // Kp
    0.00002f,   // Ki
    0.0f,       // Kd
    -1.0f,      // integral reset disabled
    -1.0f,      // min output duty
    1.0f,       // max output duty
    0.0f)       // derivative filter tau
{
  if (gear_ratio_ <= 0.0f)
  {
    throw std::runtime_error("SparkMax gear ratio must be greater than zero");
  }
}

uint8_t SparkMax::device_id() const
{
  return device_id_;
}

float SparkMax::gear_ratio() const
{
  return gear_ratio_;
}

void SparkMax::set_gear_ratio(float gear_ratio)
{
  if (gear_ratio <= 0.0f)
  {
    throw std::runtime_error("SparkMax gear ratio must be greater than zero");
  }

  gear_ratio_ = gear_ratio;
}

bool SparkMax::send_heartbeats(bool print)
{
  const std::vector<uint8_t> data = {
    0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF
  };

  const uint32_t non_rio_id = make_sparkmax_id(
    API_CLASS_NON_RIO,
    API_INDEX_NON_RIO_HEARTBEAT,
    HEARTBEAT_DEVICE_ID);

  const uint32_t rio_id = make_sparkmax_id(
    API_CLASS_ROBORIO,
    API_INDEX_ROBORIO_HEARTBEAT,
    HEARTBEAT_DEVICE_ID);

  const bool ok1 = can_.send_extended_frame(non_rio_id, data, print);
  const bool ok2 = can_.send_extended_frame(rio_id, data, print);

  return ok1 && ok2;
}

bool SparkMax::clear_faults(bool print)
{
  const uint32_t id = make_sparkmax_id(
    API_CLASS_CLEAR_FAULTS,
    API_INDEX_CLEAR_FAULTS,
    device_id_);

  return can_.send_extended_frame(id, {}, print);
}

bool SparkMax::send_setpoint(
  uint8_t api_class,
  uint8_t api_index,
  float setpoint,
  uint8_t pid_slot,
  bool print)
{
  const uint32_t id = make_sparkmax_id(
    api_class,
    api_index,
    device_id_);

  std::vector<uint8_t> data(8, 0x00);

  uint8_t target[4];
  float_to_le_bytes(setpoint, target);

  data[0] = target[0];
  data[1] = target[1];
  data[2] = target[2];
  data[3] = target[3];

  data[4] = 0x00;
  data[5] = 0x00;
  data[6] = static_cast<uint8_t>(pid_slot & 0x03);
  data[7] = 0x00;

  return can_.send_extended_frame(id, data, print);
}

bool SparkMax::send_setpoint_with_control_type(
  uint8_t api_class,
  uint8_t api_index,
  float setpoint,
  uint8_t control_type,
  uint8_t pid_slot,
  bool print)
{
  const uint32_t id = make_sparkmax_id(
    api_class,
    api_index,
    device_id_);

  std::vector<uint8_t> data(8, 0x00);

  uint8_t target[4];
  float_to_le_bytes(setpoint, target);

  data[0] = target[0];
  data[1] = target[1];
  data[2] = target[2];
  data[3] = target[3];

  data[4] = control_type;
  data[5] = 0x00;
  data[6] = static_cast<uint8_t>(pid_slot & 0x03);
  data[7] = 0x00;

  return can_.send_extended_frame(id, data, print);
}

bool SparkMax::set_duty_cycle(float duty, bool print)
{
  duty = clamp_duty(duty);

  return send_setpoint(
    API_CLASS_DUTY_CYCLE_SETPOINT,
    API_INDEX_DUTY_CYCLE,
    duty,
    0,
    print);
}

bool SparkMax::stop(bool print)
{
  velocity_controller_.reset();
  velocity_controller_.setOutputState(0.0f);

  send_heartbeats(false);
  return set_duty_cycle(0.0f, print);
}

bool SparkMax::read_telemetry(int max_frames, bool print_status_frames)
{
  bool parsed_any = false;

  for (int i = 0; i < max_frames; ++i)
  {
    CANFrame frame;

    if (!can_.read_can_frame(frame, false))
    {
      return parsed_any;
    }

    if (parse_status_frame(frame, print_status_frames))
    {
      parsed_any = true;
    }
  }

  return parsed_any;
}

const SparkMaxTelemetry & SparkMax::telemetry() const
{
  return telemetry_;
}

float SparkMax::encoder_velocity_rpm() const
{
  return telemetry_.encoder_velocity_rpm;
}

float SparkMax::motor_rad_per_sec() const
{
  return telemetry_.motor_rad_per_sec;
}

float SparkMax::wheel_rad_per_sec() const
{
  return telemetry_.wheel_rad_per_sec;
}

float SparkMax::encoder_position_rotations() const
{
  return telemetry_.encoder_position_rotations;
}

float SparkMax::wheel_position_rotations() const
{
  return telemetry_.wheel_position_rotations;
}

float SparkMax::applied_output() const
{
  return telemetry_.applied_output;
}

void SparkMax::configure_velocity_pid(
  float Kp,
  float Ki,
  float Kd,
  float integral_reset,
  float d_tau)
{
  velocity_controller_.setGains(Kp, Ki, Kd);
  velocity_controller_.setIntegralReset(integral_reset);
  velocity_controller_.setDerivativeFilterTau(d_tau);
}

void SparkMax::set_velocity_command_mode(VelocityCommandMode mode)
{
  velocity_command_mode_ = mode;

  velocity_controller_.reset();
  velocity_controller_.setOutputState(0.0f);
}

SparkMax::VelocityCommandMode SparkMax::velocity_command_mode() const
{
  return velocity_command_mode_;
}

void SparkMax::set_native_velocity_pid_slot(uint8_t pid_slot)
{
  native_velocity_pid_slot_ = static_cast<uint8_t>(pid_slot & 0x03);
}

bool SparkMax::set_native_velocity_rpm(
  float target_motor_rpm,
  bool print)
{
  return send_setpoint(
    API_CLASS_NATIVE_VELOCITY_SETPOINT,
    API_INDEX_NATIVE_VELOCITY,
    target_motor_rpm,
    native_velocity_pid_slot_,
    print);
}

bool SparkMax::set_native_velocity_rad_per_sec(
  float target_wheel_rad_per_sec,
  bool print)
{
  const float target_motor_rad_per_sec =
    target_wheel_rad_per_sec * gear_ratio_;

  const float target_motor_rpm =
    rad_per_sec_to_rpm(target_motor_rad_per_sec);

  return set_native_velocity_rpm(target_motor_rpm, print);
}

bool SparkMax::debug_send_setpoint_api(
  uint8_t api_class,
  uint8_t api_index,
  float setpoint,
  uint8_t pid_slot,
  bool print)
{
  return send_setpoint(
    api_class,
    api_index,
    setpoint,
    pid_slot,
    print);
}

bool SparkMax::set_velocity_rad_per_sec(
  float target_wheel_rad_per_sec,
  bool print)
{
  if (velocity_command_mode_ == VelocityCommandMode::NativeSparkMaxPid)
  {
    return set_native_velocity_rad_per_sec(
      target_wheel_rad_per_sec,
      print);
  }

  update_velocity_controller_only(target_wheel_rad_per_sec);
  return send_velocity_controller_output(print);
}

bool SparkMax::set_velocity_rpm(
  float target_motor_rpm,
  bool print)
{
  if (velocity_command_mode_ == VelocityCommandMode::NativeSparkMaxPid)
  {
    return set_native_velocity_rpm(target_motor_rpm, print);
  }

  const float target_motor_rad_per_sec =
    rpm_to_rad_per_sec(target_motor_rpm);

  const float target_wheel_rad_per_sec =
    target_motor_rad_per_sec / gear_ratio_;

  return set_velocity_rad_per_sec(
    target_wheel_rad_per_sec,
    print);
}

void SparkMax::update_velocity_controller_only(float target_wheel_rad_per_sec)
{
  constexpr float STOP_TARGET_RAD_PER_SEC = 0.001f;
  constexpr float MIN_MOVING_DUTY = 0.07f;

  if (std::fabs(target_wheel_rad_per_sec) < STOP_TARGET_RAD_PER_SEC)
  {
    velocity_controller_.reset();
    velocity_controller_.setOutputState(0.0f);
    return;
  }

  float duty = clamp_duty(velocity_controller_.getOutputState());

  if (!telemetry_.has_encoder_velocity)
  {
    if (duty == 0.0f)
    {
      duty = target_wheel_rad_per_sec > 0.0f ?
        MIN_MOVING_DUTY :
        -MIN_MOVING_DUTY;
    }

    velocity_controller_.setOutputState(duty);
    return;
  }

  const float measured_wheel_rad_per_sec =
    telemetry_.wheel_rad_per_sec;

  const auto [delta_output, latency_ms] =
    velocity_controller_.update(
      target_wheel_rad_per_sec,
      measured_wheel_rad_per_sec);

  (void)delta_output;
  (void)latency_ms;

  duty = clamp_duty(velocity_controller_.getOutputState());

  if (duty > 0.0f && duty < MIN_MOVING_DUTY)
  {
    duty = MIN_MOVING_DUTY;
  }
  else if (duty < 0.0f && duty > -MIN_MOVING_DUTY)
  {
    duty = -MIN_MOVING_DUTY;
  }

  if (duty == 0.0f)
  {
    duty = target_wheel_rad_per_sec > 0.0f ?
      MIN_MOVING_DUTY :
      -MIN_MOVING_DUTY;
  }

  velocity_controller_.setOutputState(duty);
}

bool SparkMax::send_velocity_controller_output(bool print)
{
  const float duty =
    clamp_duty(velocity_controller_.getOutputState());

  return set_duty_cycle(duty, print);
}

void SparkMax::reset_velocity_controller()
{
  velocity_controller_.reset();
  velocity_controller_.setOutputState(0.0f);
}

float SparkMax::velocity_controller_output() const
{
  return velocity_controller_.getOutputState();
}

uint32_t SparkMax::make_sparkmax_id(
  uint8_t api_class,
  uint8_t api_index,
  uint8_t device_id)
{
  return make_frc_extended_can_id(
    DEVICE_TYPE_MOTOR_CONTROLLER,
    MANUFACTURER_REV,
    api_class,
    api_index,
    device_id);
}

float SparkMax::rpm_to_rad_per_sec(float rpm)
{
  return rpm * 2.0f * PI / 60.0f;
}

float SparkMax::rad_per_sec_to_rpm(float rad_per_sec)
{
  return rad_per_sec * 60.0f / (2.0f * PI);
}

float SparkMax::clamp_duty(float duty)
{
  if (duty > 1.0f)
  {
    return 1.0f;
  }

  if (duty < -1.0f)
  {
    return -1.0f;
  }

  return duty;
}

void SparkMax::float_to_le_bytes(float value, uint8_t bytes[4])
{
  static_assert(sizeof(float) == 4, "float must be 32-bit");
  std::memcpy(bytes, &value, sizeof(float));
}

float SparkMax::le_bytes_to_float(const uint8_t data[8], size_t offset)
{
  if (offset + 4 > 8)
  {
    return std::numeric_limits<float>::quiet_NaN();
  }

  float value = 0.0f;
  std::memcpy(&value, data + offset, sizeof(float));
  return value;
}

int16_t SparkMax::le_bytes_to_i16(const uint8_t data[8], size_t offset)
{
  if (offset + 2 > 8)
  {
    return 0;
  }

  int16_t value = 0;
  std::memcpy(&value, data + offset, sizeof(int16_t));
  return value;
}

bool SparkMax::parse_status_frame(
  const CANFrame & frame,
  bool print_status_frame)
{
  const auto fields = parse_frc_extended_can_id(frame.id);

  if (fields.device_type != DEVICE_TYPE_MOTOR_CONTROLLER ||
      fields.manufacturer != MANUFACTURER_REV ||
      fields.device_id != device_id_)
  {
    return false;
  }

  if (fields.api_class != API_CLASS_PERIODIC_STATUS)
  {
    return false;
  }

  if (print_status_frame)
  {
    std::cout << "RX SPARK STATUS: "
              << CANComms::frame_to_string(frame)
              << " | api_class=" << static_cast<int>(fields.api_class)
              << " api_index=" << static_cast<int>(fields.api_index)
              << " device_id=" << static_cast<int>(fields.device_id)
              << "\n";
  }

  if (fields.api_index == API_INDEX_STATUS_0)
  {
    if (frame.dlc >= 2)
    {
      const int16_t raw_output =
        le_bytes_to_i16(frame.data, 0);

      telemetry_.applied_output =
        static_cast<float>(raw_output) / 32767.0f;

      telemetry_.has_applied_output = true;
    }

    return true;
  }

  if (fields.api_index == API_INDEX_STATUS_1)
  {
    return true;
  }

  if (fields.api_index == API_INDEX_STATUS_2)
  {
    if (frame.dlc >= 8)
    {
      const float encoder_velocity_rpm =
        le_bytes_to_float(frame.data, 0);

      const float encoder_position_rotations =
        le_bytes_to_float(frame.data, 4);

      if (std::isfinite(encoder_velocity_rpm))
      {
        telemetry_.encoder_velocity_rpm =
          encoder_velocity_rpm;

        telemetry_.motor_rad_per_sec =
          rpm_to_rad_per_sec(encoder_velocity_rpm);

        telemetry_.wheel_rad_per_sec =
          telemetry_.motor_rad_per_sec / gear_ratio_;

        telemetry_.has_encoder_velocity = true;
      }

      if (std::isfinite(encoder_position_rotations))
      {
        telemetry_.encoder_position_rotations =
          encoder_position_rotations;

        telemetry_.wheel_position_rotations =
          encoder_position_rotations / gear_ratio_;

        telemetry_.has_encoder_position = true;
      }
    }

    return true;
  }

  if (fields.api_index == API_INDEX_STATUS_5)
  {
    return true;
  }

  return false;
}

}  // namespace diffdrive_canbus