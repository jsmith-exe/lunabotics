#include "diffdrive_canbus/spark_max.hpp"

#include <algorithm>
#include <cstring>
#include <iostream>
#include <thread>
#include <vector>

namespace diffdrive_canbus
{

SparkMax::SparkMax(CANComms & can, uint8_t device_id)
: can_(can),
  device_id_(device_id)
{
}

uint8_t SparkMax::device_id() const
{
  return device_id_;
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

std::vector<uint8_t> SparkMax::float_to_le_bytes(float value)
{
  static_assert(sizeof(float) == 4, "float must be 32-bit");

  std::vector<uint8_t> bytes(4, 0x00);
  std::memcpy(bytes.data(), &value, sizeof(float));
  return bytes;
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
    device_id_);

  const uint32_t rio_id = make_sparkmax_id(
    API_CLASS_ROBORIO,
    API_INDEX_ROBORIO_HEARTBEAT,
    device_id_);

  const bool ok1 = can_.send_extended_frame(non_rio_id, data, print);
  const bool ok2 = can_.send_extended_frame(rio_id, data, print);

  return ok1 && ok2;
}

bool SparkMax::clear_faults(bool print)
{
  const uint32_t id = make_sparkmax_id(
    API_CLASS_PERIODIC_STATUS,
    API_INDEX_CLEAR_FAULTS,
    device_id_);

  return can_.send_extended_frame(id, {}, print);
}

bool SparkMax::send_setpoint_frame(
  uint8_t api_class,
  uint8_t api_index,
  float setpoint,
  bool print)
{
  const uint32_t id = make_sparkmax_id(
    api_class,
    api_index,
    device_id_);

  std::vector<uint8_t> data(8, 0x00);

  const auto target = float_to_le_bytes(setpoint);

  // Data[0..3] = setpoint as IEEE float32, little-endian.
  data[0] = target[0];
  data[1] = target[1];
  data[2] = target[2];
  data[3] = target[3];

  // Data[4..5] = arbitrary feedforward.
  // Leave as zero for now.
  data[4] = 0x00;
  data[5] = 0x00;

  // Data[6] bits [1:0] = PID slot.
  // Slot 0 for now.
  data[6] = 0x00;

  // Data[7] reserved.
  data[7] = 0x00;

  return can_.send_extended_frame(id, data, print);
}

bool SparkMax::set_duty_cycle(float duty, bool print)
{
  if (duty < -1.0f || duty > 1.0f)
  {
    std::cerr << "SPARK MAX duty cycle must be between -1.0 and +1.0\n";
    return false;
  }

  return send_setpoint_frame(
    API_CLASS_SETPOINT,
    API_INDEX_DUTY_CYCLE,
    duty,
    print);
}

bool SparkMax::set_velocity_rpm(float rpm, bool print)
{
  return send_setpoint_frame(
    API_CLASS_VELOCITY,
    API_INDEX_VELOCITY,
    rpm,
    print);
}

bool SparkMax::update_duty(float duty, bool print)
{
  const bool heartbeat_ok = send_heartbeats(false);
  const bool command_ok = set_duty_cycle(duty, print);

  return heartbeat_ok && command_ok;
}

bool SparkMax::stop(bool print)
{
  const bool heartbeat_ok = send_heartbeats(false);
  const bool command_ok = set_duty_cycle(0.0f, print);

  return heartbeat_ok && command_ok;
}

void SparkMax::run_duty_for(
  float duty,
  std::chrono::milliseconds duration,
  std::chrono::milliseconds period)
{
  const auto start = std::chrono::steady_clock::now();

  while (std::chrono::steady_clock::now() - start < duration)
  {
    update_duty(duty, false);
    std::this_thread::sleep_for(period);
  }
}

void SparkMax::stop_for(
  std::chrono::milliseconds duration,
  std::chrono::milliseconds period)
{
  const auto start = std::chrono::steady_clock::now();

  while (std::chrono::steady_clock::now() - start < duration)
  {
    stop(false);
    std::this_thread::sleep_for(period);
  }
}

}  // namespace diffdrive_canbus