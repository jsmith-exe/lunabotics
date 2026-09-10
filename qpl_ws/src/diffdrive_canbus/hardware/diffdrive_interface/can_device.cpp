#include "diffdrive_canbus/can_device.hpp"

#include <cmath>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>
#include <functional>

#include "diffdrive_canbus/diffdrive_interface.hpp"

namespace diffdrive_canbus
{
constexpr uint8_t SPARKMAX_API_DUTY_CYCLE_SET = 0x02;
constexpr uint8_t SPARKMAX_API_VELOCITY_SET   = 0x12;
constexpr uint8_t SPARKMAX_API_POSITION_SET   = 0x32;

CANDevice::CANDevice(std::string name, const uint8_t &can_id, SocketCanInterface &can, float gear_ratio, rclcpp::Logger &logger)
: name_(std::move(name)),
  can_id_(can_id),
  can_(can),
  gear_ratio_(gear_ratio),
  logger_(logger)
{
  if (can_id_ == 0)
  {
    throw std::runtime_error(
      "Refusing to construct SparkMax with CAN ID 0.");
  }

  if (gear_ratio_ <= 0.0f)
  {
    throw std::runtime_error("SparkMax gear ratio must be greater than zero");
  }
}

void CANDevice::configure() {
  set_status_period(0, STATUS0_PERIOD_MS);
  set_status_period(1, STATUS1_PERIOD_MS);
}

double CANDevice::clamp_and_apply_deadband_if_finite(double value, double deadband, double min, double max)
{
  if (!std::isfinite(value)) {
    return 0.0;
  }

  value = std::clamp(value, min, max);
  if (std::fabs(value) <= deadband)
  {
    return 0.0;
  }
  return value;
}

bool CANDevice::send_heartbeats(bool print)
{
  const std::vector<uint8_t> data = {
    0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF
  };

  const uint32_t non_rio_id = make_sparkmax_id(
    API_CLASS_NON_RIO,
    API_INDEX_NON_RIO_HEARTBEAT,
    HEARTBEAT_DEVICE_ID);

  if (print)
  {
    std::cout << "DEBUG TX HEARTBEAT:"
              << " heartbeat_device_id=" << static_cast<int>(HEARTBEAT_DEVICE_ID)
              << " can_id=0x"
              << std::hex << std::uppercase << non_rio_id
              << std::dec
              << "\n";
  }

  return can_.send_extended_frame(non_rio_id, data);
}

bool CANDevice::clear_faults(bool print)
{
  const uint32_t id = make_sparkmax_id(
    API_CLASS_CLEAR_FAULTS,
    API_INDEX_CLEAR_FAULTS,
    can_id_);

  if (print)
  {
    std::cout << "DEBUG TX CLEAR_FAULTS:"
              << " device_id_=" << static_cast<int>(can_id_)
              << " can_id=0x"
              << std::hex << std::uppercase << id
              << std::dec
              << "\n";
  }

  if (can_id_ == 0)
  {
    std::cerr << "WARNING: clear_faults is targeting SPARK MAX device ID 0\n";
  }

  return can_.send_extended_frame(id, {});
}

bool CANDevice::set_status_period(uint8_t status_frame_index, uint16_t period_ms)
{
  // status_frame_index: 0-4, corresponding to Periodic Status 0-4
  std::vector<uint8_t> data(2, 0x00);

  const uint32_t id = make_sparkmax_id(0x06, status_frame_index, can_id_);

  data[0] = static_cast<uint8_t>(period_ms & 0xFF);
  data[1] = static_cast<uint8_t>((period_ms >> 8) & 0xFF);

  return can_.send_extended_frame(id, data);
}

bool CANDevice::send_setpoint(
  uint8_t api_class,
  uint8_t api_index,
  float setpoint,
  uint8_t pid_slot)
{
  const uint32_t id = make_sparkmax_id(api_class, api_index, can_id_);

  std::cout << "DEBUG TX SETPOINT:"
            << " api_class=" << static_cast<int>(api_class)
            << " api_index=" << static_cast<int>(api_index)
            << " device_id_=" << static_cast<int>(can_id_)
            << " can_id=0x"
            << std::hex << std::uppercase << id
            << std::dec
            << " setpoint=" << setpoint
            << " pid_slot=" << static_cast<int>(pid_slot)
            << "\n";

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

  return can_.send_extended_frame(id, data);
}

bool CANDevice::send_simple_setpoint(
  uint8_t api_id,
  float setpoint)
{
  const uint32_t id = make_sparkmax_id_from_api_id(api_id, can_id_);
  std::vector<uint8_t> data(8, 0x00);

  uint8_t target[4];
  float_to_le_bytes(setpoint, target);

  data[0] = target[0];
  data[1] = target[1];
  data[2] = target[2];
  data[3] = target[3];

  data[4] = 0x00;
  data[5] = 0x00;
  data[6] = static_cast<uint8_t>(native_velocity_pid_slot_ & 0x03);
  data[7] = 0x00;

  return can_.send_extended_frame(id, data);
}

const SparkMaxTelemetry & CANDevice::telemetry() const
{
  return telemetry_;
}

float CANDevice::encoder_velocity_rpm() const
{
  return telemetry_.encoder_velocity_rpm;
}

float CANDevice::motor_rad_per_sec() const
{
  return telemetry_.motor_rad_per_sec;
}

float CANDevice::wheel_rad_per_sec() const
{
  return telemetry_.wheel_rad_per_sec;
}

float CANDevice::encoder_position_rotations() const
{
  return telemetry_.encoder_position_rotations;
}

float CANDevice::wheel_position_rotations() const
{
  return telemetry_.wheel_position_rotations;
}

float CANDevice::applied_output() const
{
  return telemetry_.applied_output;
}

void CANDevice::set_native_velocity_pid_slot(uint8_t pid_slot)
{
  native_velocity_pid_slot_ = static_cast<uint8_t>(pid_slot & 0x03);
}

bool CANDevice::set_velocity_rad_per_sec(float target_wheel_rad_per_sec)
{
  const float target_motor_rad_per_sec = target_wheel_rad_per_sec * gear_ratio_;
  const float target_motor_rpm = rad_per_sec_to_rpm(target_motor_rad_per_sec);
  return send_simple_setpoint(SPARKMAX_API_VELOCITY_SET, target_motor_rpm);
}

bool CANDevice::set_position(float position)
{
  // The SPARK MAX interprets this value using its configured position
  // conversion factor. For the linear actuators that unit is millimetres.
  return send_simple_setpoint(SPARKMAX_API_POSITION_SET, position);
}

// --- All below here can go into comms
uint32_t CANDevice::make_sparkmax_id(
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

uint32_t CANDevice::make_sparkmax_id_from_api_id(
  uint8_t api_id,
  uint8_t device_id)
{
  return make_frc_extended_can_id(
    DEVICE_TYPE_MOTOR_CONTROLLER,
    MANUFACTURER_REV,
    static_cast<uint8_t>((api_id >> 4) & 0x3F),
    static_cast<uint8_t>(api_id & 0x0F),
    device_id);
}

float CANDevice::rpm_to_rad_per_sec(float rpm)
{
  return rpm * 2.0f * PI / 60.0f;
}

float CANDevice::rad_per_sec_to_rpm(float rad_per_sec)
{
  return rad_per_sec * 60.0f / (2.0f * PI);
}

void CANDevice::float_to_le_bytes(float value, uint8_t bytes[4])
{
  static_assert(sizeof(float) == 4, "float must be 32-bit");
  std::memcpy(bytes, &value, sizeof(float));
}

float CANDevice::le_bytes_to_float(const uint8_t data[8], size_t offset)
{
  if (offset + 4 > 8)
  {
    return std::numeric_limits<float>::quiet_NaN();
  }

  float value = 0.0f;
  std::memcpy(&value, data + offset, sizeof(float));
  return value;
}

int16_t CANDevice::le_bytes_to_i16(const uint8_t data[8], size_t offset)
{
  if (offset + 2 > 8)
  {
    return 0;
  }

  int16_t value = 0;
  std::memcpy(&value, data + offset, sizeof(int16_t));
  return value;
}

bool CANDevice::set_duty_cycle(float duty)
{
  duty = std::clamp(duty, -1.0f, 1.0f);
  return send_simple_setpoint(SPARKMAX_API_DUTY_CYCLE_SET, duty);
}

}  // namespace diffdrive_canbus
