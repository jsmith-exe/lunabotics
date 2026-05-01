
#include "diffdrive_canbus/CAN_comms.hpp"

#include <chrono>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

using diffdrive_canbus::CANComms;
using diffdrive_canbus::CANFrame;
using diffdrive_canbus::CANMode;
using diffdrive_canbus::make_frc_extended_can_id;
using diffdrive_canbus::parse_frc_extended_can_id;

namespace
{

constexpr uint8_t DEVICE_TYPE_MOTOR_CONTROLLER = 2;
constexpr uint8_t MANUFACTURER_REV = 5;

constexpr uint8_t API_CLASS_SETPOINT = 0;
constexpr uint8_t API_INDEX_DUTY_CYCLE = 2;

constexpr uint8_t API_CLASS_PERIODIC_STATUS = 6;
constexpr uint8_t API_INDEX_CLEAR_FAULTS = 14;

constexpr uint8_t API_CLASS_ROBORIO = 9;
constexpr uint8_t API_INDEX_ROBORIO_HEARTBEAT = 2;

constexpr uint8_t API_CLASS_NON_RIO = 11;
constexpr uint8_t API_INDEX_NON_RIO_HEARTBEAT = 2;

constexpr uint8_t HEARTBEAT_DEVICE_ID = 0;

constexpr auto LOOP_PERIOD = std::chrono::milliseconds(10);
constexpr auto ARM_TIME = std::chrono::seconds(2);
constexpr auto RUN_TIME = std::chrono::seconds(2);
constexpr auto STOP_TIME = std::chrono::seconds(2);

constexpr int NUM_ATTEMPTS = 3;

std::vector<uint8_t> float_to_le_bytes(float value)
{
  static_assert(sizeof(float) == 4, "float must be 32-bit");

  std::vector<uint8_t> bytes(4);
  std::memcpy(bytes.data(), &value, sizeof(float));
  return bytes;
}

uint32_t make_sparkmax_id(uint8_t api_class, uint8_t api_index, uint8_t device_id)
{
  return make_frc_extended_can_id(
    DEVICE_TYPE_MOTOR_CONTROLLER,
    MANUFACTURER_REV,
    api_class,
    api_index,
    device_id);
}

bool send_heartbeats(CANComms & can, bool print = false)
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

  const bool ok1 = can.send_extended_frame(non_rio_id, data, print);
  const bool ok2 = can.send_extended_frame(rio_id, data, print);

  return ok1 && ok2;
}

bool clear_faults(CANComms & can, uint8_t device_id, bool print = true)
{
  const uint32_t id = make_sparkmax_id(
    API_CLASS_PERIODIC_STATUS,
    API_INDEX_CLEAR_FAULTS,
    device_id);

  return can.send_extended_frame(id, {}, print);
}

bool set_duty_cycle(
  CANComms & can,
  uint8_t device_id,
  float duty,
  uint8_t pid_slot = 0,
  bool print = false)
{
  if (duty < -1.0f || duty > 1.0f)
  {
    std::cerr << "Duty cycle must be between -1 and 1\n";
    return false;
  }

  const uint32_t id = make_sparkmax_id(
    API_CLASS_SETPOINT,
    API_INDEX_DUTY_CYCLE,
    device_id);

  std::vector<uint8_t> data(8, 0x00);

  const auto target = float_to_le_bytes(duty);

  data[0] = target[0];
  data[1] = target[1];
  data[2] = target[2];
  data[3] = target[3];

  data[6] = static_cast<uint8_t>(pid_slot & 0x03);

  return can.send_extended_frame(id, data, print);
}

void print_received_frames(CANComms & can, int frames_to_try = 50)
{
  std::cout << "\nListening for returned/status frames...\n";

  for (int i = 0; i < frames_to_try; ++i)
  {
    CANFrame frame;
    if (!can.read_can_frame(frame, false))
    {
      continue;
    }

    const auto fields = parse_frc_extended_can_id(frame.id);

    std::cout << CANComms::frame_to_string(frame)
              << " | device_type=" << static_cast<int>(fields.device_type)
              << " manufacturer=" << static_cast<int>(fields.manufacturer)
              << " api_class=" << static_cast<int>(fields.api_class)
              << " api_index=" << static_cast<int>(fields.api_index)
              << " device_id=" << static_cast<int>(fields.device_id)
              << "\n";
  }
}

void command_duty_for(
  CANComms & can,
  uint8_t device_id,
  float duty,
  std::chrono::milliseconds duration,
  bool print_commands = false)
{
  const auto start = std::chrono::steady_clock::now();

  while (std::chrono::steady_clock::now() - start < duration)
  {
    send_heartbeats(can, false);
    set_duty_cycle(can, device_id, duty, 0, print_commands);

    std::this_thread::sleep_for(LOOP_PERIOD);
  }
}

void stop_motor_for(
  CANComms & can,
  uint8_t device_id,
  std::chrono::milliseconds duration)
{
  const auto start = std::chrono::steady_clock::now();

  while (std::chrono::steady_clock::now() - start < duration)
  {
    send_heartbeats(can, false);
    set_duty_cycle(can, device_id, 0.0f, 0, false);

    std::this_thread::sleep_for(LOOP_PERIOD);
  }
}

void run_attempt(
  CANComms & can,
  uint8_t spark_id,
  float duty,
  int attempt,
  int total_attempts)
{
  std::cout << "\nAttempt " << attempt << "/" << total_attempts << "\n";

  std::cout << "Clearing faults...\n";
  for (int i = 0; i < 3; ++i)
  {
    clear_faults(can, spark_id, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  std::cout << "Arming with heartbeat pair + zero duty...\n";
  command_duty_for(
    can,
    spark_id,
    0.0f,
    ARM_TIME,
    false);

  std::cout << "Running duty " << duty << "...\n";
  command_duty_for(
    can,
    spark_id,
    duty,
    RUN_TIME,
    false);

  std::cout << "Stopping...\n";
  stop_motor_for(
    can,
    spark_id,
    STOP_TIME);
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3)
  {
    std::cerr << "Usage:\n"
              << "  " << argv[0] << " /dev/ttyUSB0 <spark_can_id> [duty]\n\n"
              << "Example:\n"
              << "  " << argv[0] << " /dev/ttyUSB0 1 0.03\n";
    return 1;
  }

  const std::string serial_device = argv[1];
  const uint8_t spark_id = static_cast<uint8_t>(std::stoi(argv[2]));

  float duty = 0.03f;
  if (argc >= 4)
  {
    duty = std::stof(argv[3]);
  }

  if (duty < -0.2f || duty > 0.2f)
  {
    std::cerr << "Refusing to run duty outside +/-0.2 for this test.\n";
    return 1;
  }

  CANComms can;

  try
  {
    can.connect(serial_device, 2000000, 20);

    if (!can.configure_adapter(
          1000000,
          false,
          0x00000000,
          0x00000000,
          CANMode::NORMAL,
          false,
          true))
    {
      std::cerr << "Failed to configure adapter\n";
      return 1;
    }

    std::cout << "\nConnected and configured.\n";
    std::cout << "Testing SPARK MAX ID " << static_cast<int>(spark_id) << "\n";
    std::cout << "Duty command: " << duty << "\n";
    std::cout << "Attempts in one process: " << NUM_ATTEMPTS << "\n\n";

    std::cout << "Sending one printed heartbeat pair.\n";
    std::cout << "Expected IDs: 0x2052C80 and 0x2052480\n";
    send_heartbeats(can, true);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    for (int attempt = 1; attempt <= NUM_ATTEMPTS; ++attempt)
    {
      run_attempt(
        can,
        spark_id,
        duty,
        attempt,
        NUM_ATTEMPTS);
    }

    print_received_frames(can, 100);

    can.disconnect();
  }
  catch (const std::exception & e)
  {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }

  return 0;
}