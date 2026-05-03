#include "diffdrive_canbus/CAN_comms.hpp"
#include "diffdrive_canbus/spark_max.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

using diffdrive_canbus::CANComms;
using diffdrive_canbus::CANMode;
using diffdrive_canbus::SparkMax;

namespace
{

constexpr auto LOOP_PERIOD = std::chrono::milliseconds(10);
constexpr auto ARM_TIME = std::chrono::seconds(2);
constexpr auto RUN_TIME = std::chrono::seconds(4);
constexpr auto STOP_TIME = std::chrono::seconds(2);

enum class TestMode
{
  DUTY,
  RPM,
  RAD_PER_SEC
};

void print_telemetry(
  TestMode mode,
  float command,
  const SparkMax & spark)
{
  const auto & t = spark.telemetry();

  std::cout << std::fixed << std::setprecision(3);

  if (mode == TestMode::DUTY)
  {
    std::cout << "cmd duty=" << command;
  }
  else if (mode == TestMode::RPM)
  {
    std::cout << "cmd rpm=" << command;
  }
  else
  {
    std::cout << "cmd rad/s=" << command;
  }

  if (t.has_encoder_velocity)
  {
    std::cout << " | encoder_velocity=" << t.encoder_velocity_rpm << " RPM"
              << " | motor=" << t.motor_rad_per_sec << " rad/s"
              << " | wheel=" << t.wheel_rad_per_sec << " rad/s";
  }
  else
  {
    std::cout << " | encoder_velocity=NO STATUS2";
  }

  if (t.has_encoder_position)
  {
    std::cout << " | encoder_position=" << t.encoder_position_rotations << " rev"
              << " | wheel_position=" << t.wheel_position_rotations << " rev";
  }

  if (t.has_applied_output)
  {
    std::cout << " | applied=" << t.applied_output;
  }
  else
  {
    std::cout << " | applied=NO STATUS0";
  }

  if (mode != TestMode::DUTY)
  {
    std::cout << " | pid_correction=" << spark.velocity_controller_output();
  }

  std::cout << "\n";
}

bool send_command(
  SparkMax & spark,
  TestMode mode,
  float command,
  bool print_command = false)
{
  if (mode == TestMode::DUTY)
  {
    return spark.set_duty_cycle(command, print_command);
  }

  if (mode == TestMode::RPM)
  {
    return spark.set_velocity_rpm(command, print_command);
  }

  return spark.set_velocity_rad_per_sec(command, print_command);
}

void command_for(
  SparkMax & spark,
  TestMode mode,
  float command,
  std::chrono::milliseconds duration,
  bool print_commands = false,
  bool print_status_frames = false)
{
  const auto start = std::chrono::steady_clock::now();
  auto last_print = start;

  while (std::chrono::steady_clock::now() - start < duration)
  {
    const bool hb_ok = spark.send_heartbeats(false);

    const bool cmd_ok = send_command(
      spark,
      mode,
      command,
      print_commands);

    // For duty mode this is needed after the command.
    // For velocity mode the set_velocity_* function already reads telemetry internally,
    // but this extra read keeps the printed values fresh.
    spark.read_telemetry(50, print_status_frames);

    if (!hb_ok)
    {
      std::cerr << "WARNING: heartbeat send failed\n";
    }

    if (!cmd_ok)
    {
      std::cerr << "WARNING: command send failed\n";
    }

    const auto now = std::chrono::steady_clock::now();

    if (now - last_print >= std::chrono::milliseconds(100))
    {
      print_telemetry(mode, command, spark);
      last_print = now;
    }

    std::this_thread::sleep_for(LOOP_PERIOD);
  }
}

TestMode parse_mode(const std::string & mode)
{
  if (mode == "duty")
  {
    return TestMode::DUTY;
  }

  if (mode == "rpm")
  {
    return TestMode::RPM;
  }

  if (mode == "rad" || mode == "rad/s" || mode == "rads")
  {
    return TestMode::RAD_PER_SEC;
  }

  throw std::runtime_error("Invalid mode. Use duty, rpm, or rad.");
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 5)
  {
    std::cerr << "Usage:\n"
              << "  " << argv[0] << " /dev/ttyUSB0 <spark_can_id> <mode> <value> [gear_ratio] [--print-status]\n\n"
              << "Modes:\n"
              << "  duty  value is duty from -1.0 to 1.0\n"
              << "  rpm   value is target motor RPM\n"
              << "  rad   value is target wheel rad/s\n\n"
              << "Examples:\n"
              << "  " << argv[0] << " /dev/ttyUSB0 1 duty 0.1\n"
              << "  " << argv[0] << " /dev/ttyUSB0 1 rpm 580\n"
              << "  " << argv[0] << " /dev/ttyUSB0 1 rad 6.0\n"
              << "  " << argv[0] << " /dev/ttyUSB0 1 rpm 580 10.71\n";
    return 1;
  }

  const std::string serial_device = argv[1];
  const uint8_t spark_id = static_cast<uint8_t>(std::stoi(argv[2]));
  const TestMode mode = parse_mode(argv[3]);
  const float command = std::stof(argv[4]);

  float gear_ratio = 1.0f;
  if (argc >= 6 && std::string(argv[5]).rfind("--", 0) != 0)
  {
    gear_ratio = std::stof(argv[5]);
  }

  bool print_status_frames = false;
  for (int i = 1; i < argc; ++i)
  {
    if (std::string(argv[i]) == "--print-status")
    {
      print_status_frames = true;
    }
  }

  if (gear_ratio <= 0.0f)
  {
    std::cerr << "Gear ratio must be greater than zero\n";
    return 1;
  }

  if (mode == TestMode::DUTY && (command < -1.0f || command > 1.0f))
  {
    std::cerr << "Duty must be between -1.0 and 1.0\n";
    return 1;
  }

  if (mode == TestMode::DUTY && std::fabs(command) > 0.2f)
  {
    std::cout << "\nWARNING: duty is above +/-0.2. Make sure the motor is secure.\n";
  }

  if (mode == TestMode::RPM && std::fabs(command) > 5800.0f)
  {
    std::cout << "\nWARNING: target RPM is above 5800. Output will clamp to +/-1 duty.\n";
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

    SparkMax spark(can, spark_id, gear_ratio);

    // Gentle starting gains because the feedforward already does most of the work.
    spark.configure_velocity_pid(
      0.001f,  // Kp
      0.0f,    // Ki
      0.0f,    // Kd
      -1.0f,   // integral reset disabled
      0.02f);  // derivative filter tau

    std::cout << "\nConnected and configured.\n";
    std::cout << "Testing SPARK MAX ID " << static_cast<int>(spark_id) << "\n";
    std::cout << "Gear ratio: " << gear_ratio << " motor rev / wheel rev\n";

    if (mode == TestMode::DUTY)
    {
      std::cout << "Command mode: duty\n";
      std::cout << "Duty command: " << command << "\n";
    }
    else if (mode == TestMode::RPM)
    {
      std::cout << "Command mode: motor RPM\n";
      std::cout << "Target motor RPM: " << command << "\n";
    }
    else
    {
      std::cout << "Command mode: wheel rad/s\n";
      std::cout << "Target wheel rad/s: " << command << "\n";
    }

    if (print_status_frames)
    {
      std::cout << "Verbose status printing enabled.\n";
    }

    std::cout << "\nSending one printed heartbeat pair.\n";
    spark.send_heartbeats(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout << "\nClearing faults...\n";
    for (int i = 0; i < 3; ++i)
    {
      spark.clear_faults(false);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    std::cout << "\nArming with heartbeat pair + zero duty...\n";
    spark.reset_velocity_controller();
    command_for(
      spark,
      TestMode::DUTY,
      0.0f,
      ARM_TIME,
      false,
      print_status_frames);

    std::cout << "\nRunning command...\n";
    spark.reset_velocity_controller();

    // Prime feedback once before the first velocity command.
    spark.send_heartbeats(false);
    spark.read_telemetry(50, false);

    command_for(
      spark,
      mode,
      command,
      RUN_TIME,
      false,
      print_status_frames);

    std::cout << "\nStopping...\n";
    spark.reset_velocity_controller();
    command_for(
      spark,
      TestMode::DUTY,
      0.0f,
      STOP_TIME,
      false,
      print_status_frames);

    can.disconnect();
  }
  catch (const std::exception & e)
  {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }

  return 0;
}