#include "diffdrive_canbus/CAN_comms.hpp"
#include "diffdrive_canbus/spark_max.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

using diffdrive_canbus::CANComms;
using diffdrive_canbus::CANMode;
using diffdrive_canbus::SparkMax;

namespace
{

// Command loop period.
// Keep setpoint commands on a 20 ms robot-style loop.
constexpr auto LOOP_PERIOD = std::chrono::milliseconds(20);

// Heartbeat is deliberately separate from the command loop.
// Old REVLib-style heartbeat behaviour appears closer to a 50 ms periodic task.
constexpr auto HEARTBEAT_PERIOD = std::chrono::milliseconds(50);

// Read telemetry frequently so we can catch what happens before a runaway.
constexpr auto TELEMETRY_PERIOD = std::chrono::milliseconds(20);

constexpr auto PRINT_PERIOD = std::chrono::milliseconds(250);
constexpr auto STOP_TIME = std::chrono::milliseconds(300);

constexpr float PI = 3.14159265358979323846f;

std::atomic_bool g_running{true};

void signal_handler(int)
{
  g_running = false;
}

enum class TestMode
{
  DUTY,
  RPM,
  RAD_PER_SEC
};

struct CommandInfo
{
  float user_command = 0.0f;
  float target_motor_rpm = 0.0f;
  float target_wheel_rad_per_sec = 0.0f;
};

float rad_per_sec_to_rpm(float rad_per_sec)
{
  return rad_per_sec * 60.0f / (2.0f * PI);
}

CommandInfo make_command_info(
  TestMode mode,
  float command,
  float gear_ratio)
{
  CommandInfo info;
  info.user_command = command;

  if (mode == TestMode::DUTY)
  {
    return info;
  }

  if (mode == TestMode::RPM)
  {
    info.target_motor_rpm = command;

    info.target_wheel_rad_per_sec =
      (info.target_motor_rpm / gear_ratio) * (2.0f * PI / 60.0f);

    return info;
  }

  if (mode == TestMode::RAD_PER_SEC)
  {
    const float target_motor_rad_per_sec = command * gear_ratio;

    info.target_motor_rpm =
      rad_per_sec_to_rpm(target_motor_rad_per_sec);

    info.target_wheel_rad_per_sec = command;

    return info;
  }

  return info;
}

bool send_command(
  SparkMax & spark,
  TestMode mode,
  const CommandInfo & info,
  bool print_command)
{
  if (mode == TestMode::DUTY)
  {
    return spark.set_duty_cycle(info.user_command, print_command);
  }

  if (mode == TestMode::RPM)
  {
    return spark.set_velocity_rpm(info.target_motor_rpm, print_command);
  }

  if (mode == TestMode::RAD_PER_SEC)
  {
    return spark.set_velocity_rad_per_sec(
      info.target_wheel_rad_per_sec,
      print_command);
  }

  return false;
}

void print_runtime_status(
  SparkMax & spark,
  TestMode mode,
  const CommandInfo & info,
  int heartbeat_count,
  int command_count,
  int telemetry_count,
  bool enable_heartbeat)
{
  std::cout << std::fixed << std::setprecision(3);

  if (mode == TestMode::DUTY)
  {
    std::cout << "cmd duty=" << info.user_command;
  }
  else if (mode == TestMode::RPM)
  {
    std::cout << "cmd motor rpm=" << info.target_motor_rpm
              << " | route=native velocity";
  }
  else if (mode == TestMode::RAD_PER_SEC)
  {
    std::cout << "cmd wheel rad/s=" << info.target_wheel_rad_per_sec
              << " | target motor rpm=" << info.target_motor_rpm
              << " | route=native velocity";
  }

  std::cout << " | measured rpm=" << spark.encoder_velocity_rpm()
            << " | motor rad/s=" << spark.motor_rad_per_sec()
            << " | wheel rad/s=" << spark.wheel_rad_per_sec()
            << " | applied=" << spark.applied_output()
            << " | heartbeat=" << (enable_heartbeat ? "on" : "off")
            << " | heartbeats=" << heartbeat_count
            << " | commands=" << command_count
            << " | telemetry=" << telemetry_count
            << "\n";
}

void send_stop_for_duration(
  SparkMax & spark,
  std::chrono::milliseconds duration,
  bool enable_heartbeat)
{
  using clock = std::chrono::steady_clock;

  const auto start = clock::now();

  auto next_heartbeat = start;
  auto next_command = start;

  while (clock::now() - start < duration)
  {
    const auto now = clock::now();

    if (enable_heartbeat && now >= next_heartbeat)
    {
      spark.send_heartbeats(false);

      next_heartbeat += HEARTBEAT_PERIOD;

      if (next_heartbeat < now - HEARTBEAT_PERIOD)
      {
        next_heartbeat = now + HEARTBEAT_PERIOD;
      }
    }

    if (now >= next_command)
    {
      spark.set_duty_cycle(0.0f, false);

      next_command += LOOP_PERIOD;

      if (next_command < now - LOOP_PERIOD)
      {
        next_command = now + LOOP_PERIOD;
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void command_until_ctrl_c(
  SparkMax & spark,
  TestMode mode,
  const CommandInfo & info,
  int timeout_seconds,
  bool print_commands,
  bool enable_heartbeat)
{
  using clock = std::chrono::steady_clock;

  const auto start = clock::now();

  auto next_heartbeat = start;
  auto next_command = start;
  auto next_telemetry = start;
  auto next_print = start + PRINT_PERIOD;

  int heartbeat_count = 0;
  int command_count = 0;
  int telemetry_count = 0;

  std::cout << "\nRunning command. Press Ctrl+C to stop.\n";

  if (timeout_seconds > 0)
  {
    std::cout << "Automatic stop after " << timeout_seconds << " seconds.\n";
  }

  std::cout << "Heartbeat: " << (enable_heartbeat ? "enabled" : "disabled") << "\n";
  if (enable_heartbeat)
  {
    std::cout << "Heartbeat period: " << HEARTBEAT_PERIOD.count() << " ms\n";
  }
  std::cout << "Command period: " << LOOP_PERIOD.count() << " ms\n";
  std::cout << "Telemetry period: " << TELEMETRY_PERIOD.count() << " ms\n";
  std::cout << "\n";

  while (g_running)
  {
    const auto now = clock::now();

    if (timeout_seconds > 0 &&
        now - start >= std::chrono::seconds(timeout_seconds))
    {
      std::cout << "\nTimeout reached. Stopping...\n";
      break;
    }

    if (enable_heartbeat && now >= next_heartbeat)
    {
      const bool hb_ok = spark.send_heartbeats(false);

      if (hb_ok)
      {
        ++heartbeat_count;
      }
      else
      {
        std::cerr << "WARNING: heartbeat send failed\n";
      }

      next_heartbeat += HEARTBEAT_PERIOD;

      if (next_heartbeat < now - HEARTBEAT_PERIOD)
      {
        next_heartbeat = now + HEARTBEAT_PERIOD;
      }
    }

    if (now >= next_command)
    {
      const bool cmd_ok = send_command(
        spark,
        mode,
        info,
        print_commands);

      if (cmd_ok)
      {
        ++command_count;
      }
      else
      {
        std::cerr << "WARNING: command send failed\n";
      }

      next_command += LOOP_PERIOD;

      if (next_command < now - LOOP_PERIOD)
      {
        next_command = now + LOOP_PERIOD;
      }
    }

    if (now >= next_telemetry)
    {
      if (spark.read_telemetry(10, false))
      {
        ++telemetry_count;
      }

      next_telemetry += TELEMETRY_PERIOD;

      if (next_telemetry < now - TELEMETRY_PERIOD)
      {
        next_telemetry = now + TELEMETRY_PERIOD;
      }
    }

    if (now >= next_print)
    {
      print_runtime_status(
        spark,
        mode,
        info,
        heartbeat_count,
        command_count,
        telemetry_count,
        enable_heartbeat);

      next_print += PRINT_PERIOD;

      if (next_print < now - PRINT_PERIOD)
      {
        next_print = now + PRINT_PERIOD;
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
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
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  if (argc < 5)
  {
    std::cerr << "Usage:\n"
              << "  " << argv[0]
              << " /dev/ttyUSB0 <spark_can_id> <mode> <value> [gear_ratio]"
              << " [--pid-slot N] [--print-command] [--timeout S] [--no-heartbeat]\n\n"
              << "Modes:\n"
              << "  duty   value is duty from -1.0 to 1.0\n"
              << "  rpm    value is target motor RPM\n"
              << "  rad    value is target wheel rad/s\n\n"
              << "Options:\n"
              << "  [gear_ratio]       motor rev / wheel rev, default 1.0\n"
              << "  --pid-slot N       native PID slot, default 0\n"
              << "  --print-command    print transmitted command frames\n"
              << "  --timeout S        automatically stop after S seconds, default 5\n"
              << "  --no-heartbeat     disable heartbeat frames for testing\n\n"
              << "Examples:\n"
              << "  " << argv[0] << " /dev/ttyUSB0 1 rpm 50 --timeout 5\n"
              << "  " << argv[0] << " /dev/ttyUSB0 1 rpm 50 --timeout 5 --no-heartbeat\n"
              << "  " << argv[0] << " /dev/ttyUSB0 1 duty 0.03 --timeout 2 --no-heartbeat\n";

    return 1;
  }

  const std::string serial_device = argv[1];
  const uint8_t spark_id = static_cast<uint8_t>(std::stoi(argv[2]));
  const TestMode mode = parse_mode(argv[3]);
  const float command = std::stof(argv[4]);

  float gear_ratio = 1.0f;
  bool print_commands = false;
  bool enable_heartbeat = true;
  uint8_t pid_slot = 0;
  int timeout_seconds = 5;

  int arg_index = 5;

  if (arg_index < argc && std::string(argv[arg_index]).rfind("--", 0) != 0)
  {
    gear_ratio = std::stof(argv[arg_index]);
    ++arg_index;
  }

  for (int i = arg_index; i < argc; ++i)
  {
    const std::string arg = argv[i];

    if (arg == "--print-command")
    {
      print_commands = true;
    }
    else if (arg == "--no-heartbeat")
    {
      enable_heartbeat = false;
    }
    else if (arg == "--pid-slot")
    {
      if (i + 1 >= argc)
      {
        std::cerr << "--pid-slot requires a value\n";
        return 1;
      }

      const int parsed_slot = std::stoi(argv[++i]);

      if (parsed_slot < 0 || parsed_slot > 3)
      {
        std::cerr << "PID slot should be between 0 and 3\n";
        return 1;
      }

      pid_slot = static_cast<uint8_t>(parsed_slot);
    }
    else if (arg == "--timeout")
    {
      if (i + 1 >= argc)
      {
        std::cerr << "--timeout requires a value in seconds\n";
        return 1;
      }

      timeout_seconds = std::stoi(argv[++i]);

      if (timeout_seconds < 0)
      {
        std::cerr << "--timeout must be >= 0. Use 0 for no timeout.\n";
        return 1;
      }
    }
    else
    {
      std::cerr << "Unknown argument: " << arg << "\n";
      return 1;
    }
  }

  if (spark_id == 0)
  {
    std::cerr << "SPARK MAX CAN ID 0 is blocked for this test.\n";
    std::cerr << "Use your real controller ID, e.g. 1.\n";
    return 1;
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

  if (mode == TestMode::DUTY && std::fabs(command) > 0.15f)
  {
    std::cout << "\nWARNING: duty is above +/-0.15. Make sure the motor is secure.\n";
  }

  if (mode == TestMode::RPM && std::fabs(command) > 500.0f)
  {
    std::cout << "\nWARNING: target RPM is above 500. Start lower while debugging runaway behaviour.\n";
  }

  const CommandInfo info = make_command_info(mode, command, gear_ratio);

  CANComms can;

  try
  {
    can.connect(serial_device, 2000000, 1);

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
    spark.set_native_velocity_pid_slot(pid_slot);

    std::cout << "\nConnected and configured.\n";
    std::cout << "Testing SPARK MAX ID " << static_cast<int>(spark_id) << "\n";
    std::cout << "Gear ratio: " << gear_ratio << " motor rev / wheel rev\n";
    std::cout << "Command period: " << LOOP_PERIOD.count() << " ms\n";
    std::cout << "Heartbeat: " << (enable_heartbeat ? "enabled" : "disabled") << "\n";
    if (enable_heartbeat)
    {
      std::cout << "Heartbeat period: " << HEARTBEAT_PERIOD.count() << " ms\n";
    }
    std::cout << "Telemetry period: " << TELEMETRY_PERIOD.count() << " ms\n";
    std::cout << "Timeout: " << timeout_seconds << " s";
    if (timeout_seconds == 0)
    {
      std::cout << " (disabled)";
    }
    std::cout << "\n";

    if (mode == TestMode::DUTY)
    {
      std::cout << "Command mode: duty cycle\n";
      std::cout << "Duty command: " << command << "\n";
      std::cout << "Expected command ID: 0x205008"
                << static_cast<int>(spark_id)
                << " for small CAN IDs\n";
    }
    else if (mode == TestMode::RPM)
    {
      std::cout << "Command mode: native velocity setpoint\n";
      std::cout << "Target motor RPM: " << info.target_motor_rpm << "\n";
      std::cout << "Expected command ID: 0x205048"
                << static_cast<int>(spark_id)
                << " for small CAN IDs\n";
    }
    else if (mode == TestMode::RAD_PER_SEC)
    {
      std::cout << "Command mode: native velocity setpoint\n";
      std::cout << "Target wheel rad/s: " << info.target_wheel_rad_per_sec << "\n";
      std::cout << "Calculated target motor RPM: " << info.target_motor_rpm << "\n";
      std::cout << "Expected command ID: 0x205048"
                << static_cast<int>(spark_id)
                << " for small CAN IDs\n";
    }

    std::cout << "PID slot: " << static_cast<int>(pid_slot) << "\n";

    if (print_commands)
    {
      std::cout << "\nVerbose command printing enabled.\n";
      std::cout << "NOTE: use this only briefly, because printing can disturb timing.\n";
    }

    if (enable_heartbeat)
    {
      std::cout << "\nSending one printed heartbeat...\n";
      spark.send_heartbeats(true);
    }
    else
    {
      std::cout << "\nHeartbeat disabled. No heartbeat frames will be sent by this test.\n";
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout << "\nClearing faults...\n";
    for (int i = 0; i < 3; ++i)
    {
      spark.clear_faults(false);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    std::cout << "\nSending one printed command frame...\n";
    send_command(
      spark,
      mode,
      info,
      true);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    spark.read_telemetry(20, true);

    command_until_ctrl_c(
      spark,
      mode,
      info,
      timeout_seconds,
      print_commands,
      enable_heartbeat);

    std::cout << "\nStopping...\n";

    send_stop_for_duration(
      spark,
      STOP_TIME,
      enable_heartbeat);

    std::cout << "\nFinal stop frame...\n";
    spark.stop(true);

    can.disconnect();
  }
  catch (const std::exception & e)
  {
    std::cerr << "Error: " << e.what() << "\n";

    try
    {
      can.disconnect();
    }
    catch (...)
    {
    }

    return 1;
  }

  return 0;
}