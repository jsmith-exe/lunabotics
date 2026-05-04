#include "diffdrive_canbus/CAN_comms.hpp"
#include "diffdrive_canbus/spark_max.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
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

constexpr auto LOOP_PERIOD = std::chrono::milliseconds(10);
constexpr auto ARM_TIME = std::chrono::seconds(2);
constexpr auto STOP_TIME = std::chrono::seconds(2);

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

enum class VelocityRoute
{
  SOFTWARE_DUTY_PID,
  NATIVE_SPARKMAX_PID
};

const char * route_to_string(VelocityRoute route)
{
  if (route == VelocityRoute::NATIVE_SPARKMAX_PID)
  {
    return "native SPARK MAX velocity PID";
  }

  return "software PID to duty-cycle";
}

void print_telemetry(
  TestMode mode,
  VelocityRoute route,
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
  else if (mode == TestMode::RAD_PER_SEC)
  {
    std::cout << "cmd rad/s=" << command;
  }

  if (mode == TestMode::RPM || mode == TestMode::RAD_PER_SEC)
  {
    if (route == VelocityRoute::NATIVE_SPARKMAX_PID)
    {
      std::cout << " | route=native";
    }
    else
    {
      std::cout << " | route=software";
    }
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

  if ((mode == TestMode::RPM || mode == TestMode::RAD_PER_SEC) &&
      route == VelocityRoute::SOFTWARE_DUTY_PID)
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

  if (mode == TestMode::RAD_PER_SEC)
  {
    return spark.set_velocity_rad_per_sec(command, print_command);
  }

  return false;
}

void command_for_duration(
  SparkMax & spark,
  TestMode mode,
  VelocityRoute route,
  float command,
  std::chrono::milliseconds duration,
  bool print_commands = false,
  bool print_status_frames = false)
{
  const auto start = std::chrono::steady_clock::now();
  auto last_print = start;

  while (g_running && std::chrono::steady_clock::now() - start < duration)
  {
    const bool hb_ok = spark.send_heartbeats(false);

    const bool cmd_ok = send_command(
      spark,
      mode,
      command,
      print_commands);

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
      print_telemetry(mode, route, command, spark);
      last_print = now;
    }

    std::this_thread::sleep_for(LOOP_PERIOD);
  }
}

void command_until_ctrl_c(
  SparkMax & spark,
  TestMode mode,
  VelocityRoute route,
  float command,
  bool print_commands = false,
  bool print_status_frames = false)
{
  auto last_print = std::chrono::steady_clock::now();

  std::cout << "\nRunning command continuously. Press Ctrl+C to stop.\n\n";

  while (g_running)
  {
    const bool hb_ok = spark.send_heartbeats(false);

    const bool cmd_ok = send_command(
      spark,
      mode,
      command,
      print_commands);

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
      print_telemetry(mode, route, command, spark);
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

VelocityRoute parse_route_flag(const std::string & flag)
{
  if (flag == "--software" || flag == "--software-pid")
  {
    return VelocityRoute::SOFTWARE_DUTY_PID;
  }

  if (flag == "--native" || flag == "--native-pid")
  {
    return VelocityRoute::NATIVE_SPARKMAX_PID;
  }

  throw std::runtime_error("Invalid velocity route. Use --software or --native.");
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
              << " [--software | --native] [--pid-slot N] [--print-status] [--print-command]\n\n"
              << "Modes:\n"
              << "  duty   value is duty from -1.0 to 1.0\n"
              << "  rpm    value is target motor RPM\n"
              << "  rad    value is target wheel rad/s\n\n"
              << "Velocity routes:\n"
              << "  --software  use local software PID to generate duty-cycle commands, default\n"
              << "  --native    send velocity setpoint directly to SPARK MAX internal PID\n\n"
              << "Options:\n"
              << "  --pid-slot N       native PID slot, default 0\n"
              << "  --print-status     print received SPARK MAX status frames\n"
              << "  --print-command    print transmitted command frames\n\n"
              << "Examples:\n"
              << "  " << argv[0] << " /dev/ttyUSB0 1 duty 0.1\n"
              << "  " << argv[0] << " /dev/ttyUSB0 1 rpm 580 10.71 --software\n"
              << "  " << argv[0] << " /dev/ttyUSB0 1 rpm 580 10.71 --native --pid-slot 0 --print-command\n"
              << "  " << argv[0] << " /dev/ttyUSB0 1 rad 6.0 10.71 --native --pid-slot 0\n";
    return 1;
  }

  const std::string serial_device = argv[1];
  const uint8_t spark_id = static_cast<uint8_t>(std::stoi(argv[2]));
  const TestMode mode = parse_mode(argv[3]);
  const float command = std::stof(argv[4]);

  float gear_ratio = 1.0f;
  bool print_status_frames = false;
  bool print_commands = false;
  uint8_t pid_slot = 0;

  VelocityRoute velocity_route = VelocityRoute::SOFTWARE_DUTY_PID;

  int arg_index = 5;

  if (arg_index < argc && std::string(argv[arg_index]).rfind("--", 0) != 0)
  {
    gear_ratio = std::stof(argv[arg_index]);
    ++arg_index;
  }

  for (int i = arg_index; i < argc; ++i)
  {
    const std::string arg = argv[i];

    if (arg == "--print-status")
    {
      print_status_frames = true;
    }
    else if (arg == "--print-command")
    {
      print_commands = true;
    }
    else if (arg == "--software" ||
             arg == "--software-pid" ||
             arg == "--native" ||
             arg == "--native-pid")
    {
      velocity_route = parse_route_flag(arg);
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
        std::cerr << "PID slot should usually be between 0 and 3\n";
        return 1;
      }

      pid_slot = static_cast<uint8_t>(parsed_slot);
    }
    else
    {
      std::cerr << "Unknown argument: " << arg << "\n";
      return 1;
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
    std::cout << "\nWARNING: target RPM is above 5800.\n";
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

    if (velocity_route == VelocityRoute::NATIVE_SPARKMAX_PID)
    {
      spark.set_velocity_command_mode(
        SparkMax::VelocityCommandMode::NativeSparkMaxPid);

      spark.set_native_velocity_pid_slot(pid_slot);
    }
    else
    {
      spark.set_velocity_command_mode(
        SparkMax::VelocityCommandMode::SoftwareDutyPid);
    }

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
      std::cout << "Velocity route ignored for duty mode.\n";
    }
    else if (mode == TestMode::RPM)
    {
      std::cout << "Command mode: motor RPM\n";
      std::cout << "Target motor RPM: " << command << "\n";
      std::cout << "Velocity route: " << route_to_string(velocity_route) << "\n";
    }
    else if (mode == TestMode::RAD_PER_SEC)
    {
      std::cout << "Command mode: wheel rad/s\n";
      std::cout << "Target wheel rad/s: " << command << "\n";
      std::cout << "Velocity route: " << route_to_string(velocity_route) << "\n";
    }

    if (velocity_route == VelocityRoute::NATIVE_SPARKMAX_PID &&
        mode != TestMode::DUTY)
    {
      std::cout << "Native PID slot: " << static_cast<int>(pid_slot) << "\n";
      std::cout << "NOTE: native mode should now send 0x012 for RPM commands.\n";
      std::cout << "      PID constants must already be configured on the SPARK MAX.\n";
    }

    if (print_status_frames)
    {
      std::cout << "Verbose status printing enabled.\n";
    }

    if (print_commands)
    {
      std::cout << "Verbose command printing enabled.\n";
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

    command_for_duration(
      spark,
      TestMode::DUTY,
      velocity_route,
      0.0f,
      ARM_TIME,
      print_commands,
      print_status_frames);

    if (!g_running)
    {
      std::cout << "\nStopped during arming.\n";
    }
    else
    {
      spark.reset_velocity_controller();

      spark.send_heartbeats(false);
      spark.read_telemetry(50, false);

      command_until_ctrl_c(
        spark,
        mode,
        velocity_route,
        command,
        print_commands,
        print_status_frames);
    }

    std::cout << "\nCtrl+C received. Stopping...\n";

    spark.reset_velocity_controller();

    command_for_duration(
      spark,
      TestMode::DUTY,
      velocity_route,
      0.0f,
      STOP_TIME,
      print_commands,
      print_status_frames);

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