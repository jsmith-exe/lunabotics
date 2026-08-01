#include "diffdrive_canbus/can_device.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include "diffdrive_canbus/diffdrive_canbus_system.hpp"

namespace diffdrive_canbus
{

namespace
{

constexpr uint8_t SPARKMAX_API_DUTY_CYCLE_SET = 0x02;
constexpr uint8_t SPARKMAX_API_VELOCITY_SET   = 0x12;

// Firmware 24 status frame layout:
//   Status 0: 0x2051800 + device_id
//   Status 1: 0x2051840 + device_id
//   Status 2: 0x2051880 + device_id
//
// Firmware 25+ status frame layout seen in earlier logs:
//   Status 0: 0x205B800 + device_id
//   Status 1: 0x205B840 + device_id
//   Status 2: 0x205B880 + device_id
constexpr uint8_t API_CLASS_PERIODIC_STATUS_FIRMWARE_24 = 6;
constexpr uint8_t API_CLASS_PERIODIC_STATUS_FIRMWARE_25_PLUS = 46;

constexpr int TELEMETRY_EMPTY_READ_RETRIES = 8;
constexpr auto TELEMETRY_EMPTY_READ_DELAY = std::chrono::milliseconds(1);
}  // namespace

CANDevice::CANDevice(std::string name, const uint8_t &can_id, CANComms &can, float gear_ratio)
: name_(std::move(name)),
  can_id_(can_id),
  can_(can),
  gear_ratio_(gear_ratio)
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

std::string CANDevice::name() const
{
  return name_;
}

uint8_t CANDevice::can_id() const
{
  return can_id_;
}

float CANDevice::gear_ratio() const
{
  return gear_ratio_;
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

  return can_.send_extended_frame(non_rio_id, data, print);
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

  return can_.send_extended_frame(id, {}, print);
}

bool CANDevice::send_setpoint(
  uint8_t api_class,
  uint8_t api_index,
  float setpoint,
  uint8_t pid_slot,
  bool print)
{
  const uint32_t id = make_sparkmax_id(
    api_class,
    api_index,
    can_id_);

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

  if (can_id_ == 0)
  {
    std::cerr << "WARNING: send_setpoint is targeting SPARK MAX device ID 0\n";
  }

  if (can_id_ == 0)
  {
    std::cerr << "WARNING: send_setpoint is targeting SPARK MAX device ID 0\n";
  }

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

  return can_.send_extended_frame(id, data, false);
}

bool CANDevice::send_setpoint_with_control_type(
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
    can_id_);

  std::cout << "DEBUG TX SETPOINT_WITH_CONTROL_TYPE:"
            << " api_class=" << static_cast<int>(api_class)
            << " api_index=" << static_cast<int>(api_index)
            << " device_id_=" << static_cast<int>(can_id_)
            << " can_id=0x"
            << std::hex << std::uppercase << id
            << std::dec
            << " setpoint=" << setpoint
            << " control_type=" << static_cast<int>(control_type)
            << " pid_slot=" << static_cast<int>(pid_slot)
            << "\n";

  if (can_id_ == 0)
  {
    std::cerr << "WARNING: send_setpoint_with_control_type is targeting SPARK MAX device ID 0\n";
  }

  if (can_id_ == 0)
  {
    std::cerr << "WARNING: send_setpoint_with_control_type is targeting SPARK MAX device ID 0\n";
  }

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

bool CANDevice::set_duty_cycle(float duty)
{
  duty = clamp_duty(duty);

  return send_simple_setpoint(
    SPARKMAX_API_DUTY_CYCLE_SET,
    duty);
}

bool CANDevice::stop(bool print)
{
  send_heartbeats(false);

  return set_duty_cycle(0.0f);
}

bool CANDevice::read_telemetry(int max_frames, bool print_status_frames)
{
  bool parsed_any = false;
  int frames_read = 0;
  int empty_reads = 0;

  while (frames_read < max_frames && empty_reads < TELEMETRY_EMPTY_READ_RETRIES)
  {
    CANFrame frame;

    if (!can_.read_frame(frame, false))
    {
      ++empty_reads;
      std::this_thread::sleep_for(TELEMETRY_EMPTY_READ_DELAY);
      continue;
    }

    empty_reads = 0;
    ++frames_read;

    if (parse_status_frame(frame, print_status_frames))
    {
      parsed_any = true;
    }
  }

  return parsed_any;
}

bool CANDevice::handle_status_frame(
  const CANFrame & frame,
  bool print_status_frame)
{
  return parse_status_frame(frame, print_status_frame);
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

bool CANDevice::set_native_velocity_rpm(
  float target_motor_rpm,
  bool print)
{
  return set_velocity_rpm(target_motor_rpm, print);
}

bool CANDevice::set_native_velocity_rad_per_sec(
  float target_wheel_rad_per_sec,
  bool print)
{
  return set_velocity_rad_per_sec(target_wheel_rad_per_sec, print);
}

bool CANDevice::debug_send_setpoint_api(
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

bool CANDevice::set_velocity_rad_per_sec(
  float target_wheel_rad_per_sec,
  bool print)
{
  const float target_motor_rad_per_sec =
    target_wheel_rad_per_sec * gear_ratio_;

  const float target_motor_rpm =
    rad_per_sec_to_rpm(target_motor_rad_per_sec);

  return set_velocity_rpm(target_motor_rpm, print);
}

bool CANDevice::set_velocity_rpm(
  float target_motor_rpm,
  bool print)
{
  return send_simple_setpoint(
    SPARKMAX_API_VELOCITY_SET,
    target_motor_rpm);
}

void CANDevice::reset_velocity_controller()
{
  // No software velocity controller is used anymore.
  // Velocity control is handled internally by the SPARK MAX.
}

float CANDevice::velocity_controller_output() const
{
  if (telemetry_.has_applied_output)
  {
    return telemetry_.applied_output;
  }

  return 0.0f;
}

void CANDevice::setup_ros_state_interfaces(std::vector<hardware_interface::StateInterface> &state_interfaces) {

}
void CANDevice::setup_ros_command_interfaces(std::vector<hardware_interface::CommandInterface> &command_interfaces) {

}


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

float CANDevice::clamp_duty(float duty)
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

bool CANDevice::parse_status_frame(
  const CANFrame & frame,
  bool print_status_frame)
{
  const auto fields = parse_frc_extended_can_id(frame.id);

  if (fields.device_type != DEVICE_TYPE_MOTOR_CONTROLLER ||
      fields.manufacturer != MANUFACTURER_REV ||
      fields.device_id != can_id_)
  {
    return false;
  }

  const bool is_firmware24_status =
    fields.api_class == API_CLASS_PERIODIC_STATUS_FIRMWARE_24;

  const bool is_firmware25_status =
    fields.api_class == API_CLASS_PERIODIC_STATUS_FIRMWARE_25_PLUS;

  if (!is_firmware24_status && !is_firmware25_status)
  {
    return false;
  }

  if (print_status_frame)
  {
    std::cout << "RX SPARK STATUS: "
              << CANComms::frame_to_string(frame)
              << " | api_class=" << static_cast<int>(fields.api_class)
              << " api_index=" << static_cast<int>(fields.api_index)
              << " device_id=" << static_cast<int>(fields.device_id);

    if (is_firmware24_status)
    {
      std::cout << " | style=firmware24";
    }
    else
    {
      std::cout << " | style=firmware25+";
    }

    std::cout << "\n";
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

  if (is_firmware24_status && fields.api_index == API_INDEX_STATUS_1)
  {
    // Firmware 24:
    // Status 1, bytes 0-3 = encoder velocity RPM.
    //
    // Example from your logs:
    //   DATA=[0x39 0x8A 0xF8 0x43 ...] -> about 497 RPM
    // while commanding 500 RPM.
    if (frame.dlc >= 4)
    {
      const float encoder_velocity_rpm =
        le_bytes_to_float(frame.data, 0);

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
    }

    return true;
  }

  if (is_firmware24_status && fields.api_index == API_INDEX_STATUS_2)
  {
    // Firmware 24:
    // Status 2, bytes 0-3 = encoder position rotations.
    //
    // Example from your logs:
    //   DATA=[0xB8 0xC2 0xB0 0x44 ...] -> about 1414 rotations
    // and this value increases over time.
    if (frame.dlc >= 4)
    {
      const float encoder_position_rotations =
        le_bytes_to_float(frame.data, 0);

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

  if (is_firmware25_status && fields.api_index == API_INDEX_STATUS_1)
  {
    // Not decoding firmware 25 status 1 yet.
    return true;
  }

  if (is_firmware25_status && fields.api_index == API_INDEX_STATUS_2)
  {
    // Firmware 25+ assumption from your earlier logs:
    // Status 2, bytes 0-3 = velocity RPM
    // Status 2, bytes 4-7 = position rotations
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

  // Any other status frame from this SPARK MAX is valid traffic,
  // but we are not decoding it yet.
  return true;
}

}  // namespace diffdrive_canbus