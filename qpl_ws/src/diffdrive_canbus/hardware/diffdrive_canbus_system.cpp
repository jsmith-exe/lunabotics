#include "diffdrive_canbus/CAN_comms.hpp"
#include "diffdrive_canbus/spark_max.hpp"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "pluginlib/class_list_macros.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

namespace diffdrive_canbus
{

namespace
{

constexpr double TWO_PI = 2.0 * M_PI;

constexpr auto HEARTBEAT_PERIOD = std::chrono::milliseconds(50);
constexpr auto STOP_TIME = std::chrono::milliseconds(350);
constexpr auto EXTRA_STOP_TIME = std::chrono::milliseconds(150);
constexpr auto STOP_COMMAND_PERIOD = std::chrono::milliseconds(20);
constexpr auto PRINT_PERIOD = std::chrono::milliseconds(250);

// Native velocity safety clamp.
// This is wheel rad/s before gear-ratio conversion inside SparkMax.
// Your logs showed wheel commands around +/-11.65 rad/s, so 12.0 is a sensible cap.
constexpr double MAX_NATIVE_WHEEL_RAD_PER_SEC = 12.0;

bool string_to_bool(const std::string & value)
{
  return value == "true" ||
         value == "True" ||
         value == "TRUE" ||
         value == "1" ||
         value == "yes" ||
         value == "on";
}

double apply_deadband(double value, double deadband)
{
  if (std::fabs(value) <= deadband)
  {
    return 0.0;
  }

  return value;
}

double clamp_native_velocity(double value)
{
  if (value > MAX_NATIVE_WHEEL_RAD_PER_SEC)
  {
    return MAX_NATIVE_WHEEL_RAD_PER_SEC;
  }

  if (value < -MAX_NATIVE_WHEEL_RAD_PER_SEC)
  {
    return -MAX_NATIVE_WHEEL_RAD_PER_SEC;
  }

  return value;
}

}  // namespace

class DiffDriveCanbusHardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override
  {
    if (
      hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    logger_ = rclcpp::get_logger("DiffDriveCanbusHardware");

    try
    {
      parse_hardware_parameters();
      initialise_joint_storage();
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(logger_, "on_init failed: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_WARN(logger_, "============================================================");
    RCLCPP_WARN(logger_, "DIFFDRIVE CANBUS HARDWARE IS RUNNING IN SIMPLE NATIVE VELOCITY MODE");
    RCLCPP_WARN(logger_, "Runtime path is based on the simple working duty version");
    RCLCPP_WARN(logger_, "No CAN reads are performed in read() while debugging command sticking");
    RCLCPP_WARN(logger_, "No command/heartbeat frames are sent during configure() or activate()");
    RCLCPP_WARN(logger_, "Heartbeat is only sent while commanding or stopping");
    RCLCPP_WARN(logger_, "Active commands use native SPARK MAX velocity setpoints");
    RCLCPP_WARN(logger_, "Shutdown/Ctrl+C uses zero-duty stop bursts");
    RCLCPP_WARN(logger_, "Max wheel velocity command: +/-%.3f rad/s", MAX_NATIVE_WHEEL_RAD_PER_SEC);
    RCLCPP_WARN(logger_, "============================================================");

    RCLCPP_INFO(logger_, "Initialised DiffDriveCanbusHardware");
    RCLCPP_INFO(logger_, "Serial device: %s", serial_device_.c_str());
    RCLCPP_INFO(logger_, "Serial baud rate: %d", serial_baud_rate_);
    RCLCPP_INFO(logger_, "CAN baud rate: %d", can_baud_rate_);
    RCLCPP_INFO(logger_, "Timeout: %d ms", timeout_ms_);
    RCLCPP_INFO(logger_, "Gear ratio: %.3f motor rev / wheel rev", gear_ratio_);
    RCLCPP_INFO(logger_, "Command deadband: %.6f rad/s", command_deadband_rad_per_sec_);
    RCLCPP_INFO(logger_, "PID slot: %u", pid_slot_);
    RCLCPP_INFO(logger_, "Loopback mode: %s", loopback_mode_ ? "true" : "false");

    RCLCPP_INFO(logger_, "Debug flag print_commands: %s", print_commands_ ? "true" : "false");
    RCLCPP_INFO(logger_, "Debug flag print_status: %s", print_status_frames_ ? "true" : "false");
    RCLCPP_INFO(
      logger_,
      "Debug flag debug_printing_enabled: %s",
      debug_printing_enabled_ ? "true" : "false");

    if (timeout_ms_ > 10)
    {
      RCLCPP_WARN(
        logger_,
        "timeout_ms is %d ms. Set timeout_ms to 1 or 5 inside the ros2_control hardware params.",
        timeout_ms_);
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(
      front_left_wheel_name_,
      hardware_interface::HW_IF_POSITION,
      &front_left_position_);

    state_interfaces.emplace_back(
      front_left_wheel_name_,
      hardware_interface::HW_IF_VELOCITY,
      &front_left_velocity_);

    state_interfaces.emplace_back(
      front_right_wheel_name_,
      hardware_interface::HW_IF_POSITION,
      &front_right_position_);

    state_interfaces.emplace_back(
      front_right_wheel_name_,
      hardware_interface::HW_IF_VELOCITY,
      &front_right_velocity_);

    state_interfaces.emplace_back(
      rear_left_wheel_name_,
      hardware_interface::HW_IF_POSITION,
      &rear_left_position_);

    state_interfaces.emplace_back(
      rear_left_wheel_name_,
      hardware_interface::HW_IF_VELOCITY,
      &rear_left_velocity_);

    state_interfaces.emplace_back(
      rear_right_wheel_name_,
      hardware_interface::HW_IF_POSITION,
      &rear_right_position_);

    state_interfaces.emplace_back(
      rear_right_wheel_name_,
      hardware_interface::HW_IF_VELOCITY,
      &rear_right_velocity_);

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(
      front_left_wheel_name_,
      hardware_interface::HW_IF_VELOCITY,
      &front_left_command_);

    command_interfaces.emplace_back(
      front_right_wheel_name_,
      hardware_interface::HW_IF_VELOCITY,
      &front_right_command_);

    command_interfaces.emplace_back(
      rear_left_wheel_name_,
      hardware_interface::HW_IF_VELOCITY,
      &rear_left_command_);

    command_interfaces.emplace_back(
      rear_right_wheel_name_,
      hardware_interface::HW_IF_VELOCITY,
      &rear_right_command_);

    return command_interfaces;
  }

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State &) override
  {
    try
    {
      RCLCPP_INFO(logger_, "Connecting to CAN adapter on %s", serial_device_.c_str());

      can_.connect(
        serial_device_,
        serial_baud_rate_,
        timeout_ms_);

      const bool configured = can_.configure_adapter(
        can_baud_rate_,
        false,
        0x00000000,
        0x00000000,
        loopback_mode_ ? CANMode::LOOPBACK : CANMode::NORMAL,
        false,
        true);

      if (!configured)
      {
        RCLCPP_ERROR(logger_, "Failed to configure CAN adapter");
        return hardware_interface::CallbackReturn::ERROR;
      }

      RCLCPP_WARN(logger_, "Constructing all 4 SparkMax objects");
      RCLCPP_WARN(logger_, "No SPARK MAX setpoint, zero-duty, or heartbeat frames are sent in configure()");

      front_left_spark_ = std::make_unique<SparkMax>(
        can_,
        front_left_can_id_,
        static_cast<float>(gear_ratio_));

      front_right_spark_ = std::make_unique<SparkMax>(
        can_,
        front_right_can_id_,
        static_cast<float>(gear_ratio_));

      rear_left_spark_ = std::make_unique<SparkMax>(
        can_,
        rear_left_can_id_,
        static_cast<float>(gear_ratio_));

      rear_right_spark_ = std::make_unique<SparkMax>(
        can_,
        rear_right_can_id_,
        static_cast<float>(gear_ratio_));

      front_left_spark_->set_native_velocity_pid_slot(pid_slot_);
      front_right_spark_->set_native_velocity_pid_slot(pid_slot_);
      rear_left_spark_->set_native_velocity_pid_slot(pid_slot_);
      rear_right_spark_->set_native_velocity_pid_slot(pid_slot_);

      RCLCPP_INFO(logger_, "CAN adapter configured");
      RCLCPP_INFO(logger_, "Front left SPARK MAX ID: %u", front_left_can_id_);
      RCLCPP_INFO(logger_, "Front right SPARK MAX ID: %u", front_right_can_id_);
      RCLCPP_INFO(logger_, "Rear left SPARK MAX ID: %u", rear_left_can_id_);
      RCLCPP_INFO(logger_, "Rear right SPARK MAX ID: %u", rear_right_can_id_);
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(logger_, "on_configure failed: %s", e.what());

      try
      {
        can_.disconnect();
      }
      catch (...)
      {
      }

      return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &) override
  {
    front_left_command_ = 0.0;
    front_right_command_ = 0.0;
    rear_left_command_ = 0.0;
    rear_right_command_ = 0.0;

    front_left_velocity_ = 0.0;
    front_right_velocity_ = 0.0;
    rear_left_velocity_ = 0.0;
    rear_right_velocity_ = 0.0;

    front_left_position_ = 0.0;
    front_right_position_ = 0.0;
    rear_left_position_ = 0.0;
    rear_right_position_ = 0.0;

    front_left_position_offset_ = 0.0;
    front_right_position_offset_ = 0.0;
    rear_left_position_offset_ = 0.0;
    rear_right_position_offset_ = 0.0;

    front_left_position_offset_valid_ = false;
    front_right_position_offset_valid_ = false;
    rear_left_position_offset_valid_ = false;
    rear_right_position_offset_valid_ = false;

    command_count_ = 0;
    skipped_idle_write_count_ = 0;
    telemetry_read_count_ = 0;
    telemetry_hit_count_ = 0;
    can_frames_read_count_ = 0;
    can_frames_parsed_count_ = 0;
    heartbeat_count_ = 0;
    idle_stop_count_ = 0;

    motors_currently_commanded_ = false;

    next_heartbeat_time_ = std::chrono::steady_clock::now();
    last_print_time_ = std::chrono::steady_clock::now() + PRINT_PERIOD;

    RCLCPP_WARN(logger_, "Activated in simple native velocity mode");
    RCLCPP_WARN(logger_, "No SPARK MAX frames sent during activate()");
    RCLCPP_WARN(logger_, "First heartbeat will only be sent once a real command or stop is being sent");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &) override
  {
    RCLCPP_WARN(logger_, "Deactivating DiffDriveCanbusHardware");
    RCLCPP_WARN(logger_, "Sending guaranteed zero-duty stop burst to all 4 motors");

    try
    {
      send_stop_for_duration(STOP_TIME);

      if (front_left_spark_)
      {
        front_left_spark_->stop(false);
      }

      if (front_right_spark_)
      {
        front_right_spark_->stop(false);
      }

      if (rear_left_spark_)
      {
        rear_left_spark_->stop(false);
      }

      if (rear_right_spark_)
      {
        rear_right_spark_->stop(false);
      }

      send_stop_for_duration(EXTRA_STOP_TIME);

      can_.disconnect();
    }
    catch (const std::exception & e)
    {
      RCLCPP_WARN(logger_, "Error while deactivating hardware: %s", e.what());
    }
    catch (...)
    {
      RCLCPP_WARN(logger_, "Unknown error while deactivating hardware");
    }

    RCLCPP_INFO(logger_, "DiffDriveCanbusHardware deactivated");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type read(
    const rclcpp::Time &,
    const rclcpp::Duration &) override
  {
    ++telemetry_read_count_;

    // Feedback reads remain disabled because this was the version that avoided command sticking.
    if (debug_printing_enabled_)
    {
      maybe_print_status();
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(
    const rclcpp::Time &,
    const rclcpp::Duration &) override
  {
    const double front_left_command =
      clamp_native_velocity(
        apply_deadband(front_left_command_, command_deadband_rad_per_sec_));

    const double front_right_command =
      clamp_native_velocity(
        apply_deadband(front_right_command_, command_deadband_rad_per_sec_));

    const double rear_left_command =
      clamp_native_velocity(
        apply_deadband(rear_left_command_, command_deadband_rad_per_sec_));

    const double rear_right_command =
      clamp_native_velocity(
        apply_deadband(rear_right_command_, command_deadband_rad_per_sec_));

    const bool any_active_command =
      std::fabs(front_left_command) > 0.0 ||
      std::fabs(front_right_command) > 0.0 ||
      std::fabs(rear_left_command) > 0.0 ||
      std::fabs(rear_right_command) > 0.0;

    if (!any_active_command)
    {
      ++skipped_idle_write_count_;

      // Simple working behaviour:
      // Do not send frames continuously while idle.
      // Only send one zero-duty stop when transitioning from active to idle.
      if (motors_currently_commanded_)
      {
        ++idle_stop_count_;

        RCLCPP_WARN_THROTTLE(
          logger_,
          *rclcpp::Clock::make_shared(),
          1000,
          "Commands returned to zero. Sending immediate zero-duty stop frames.");

        maybe_send_heartbeat();
        send_zero_duty_all(false);
      }

      motors_currently_commanded_ = false;

      return hardware_interface::return_type::OK;
    }

    // Heartbeat only when there is an actual non-zero command.
    // This prevents heartbeat from waking a latched command during idle launch.
    maybe_send_heartbeat();

    motors_currently_commanded_ = true;

    write_one_motor_native_velocity(
      "front_left",
      front_left_spark_,
      front_left_command);

    write_one_motor_native_velocity(
      "front_right",
      front_right_spark_,
      front_right_command);

    write_one_motor_native_velocity(
      "rear_left",
      rear_left_spark_,
      rear_left_command);

    write_one_motor_native_velocity(
      "rear_right",
      rear_right_spark_,
      rear_right_command);

    return hardware_interface::return_type::OK;
  }

private:
  void parse_hardware_parameters()
  {
    front_left_wheel_name_ = get_required_string("front_left_wheel_name");
    front_right_wheel_name_ = get_required_string("front_right_wheel_name");
    rear_left_wheel_name_ = get_required_string("rear_left_wheel_name");
    rear_right_wheel_name_ = get_required_string("rear_right_wheel_name");

    serial_device_ = get_required_string("serial_device");

    serial_baud_rate_ = get_int("serial_baud_rate", 2000000);
    can_baud_rate_ = get_int("can_baud_rate", 1000000);

    timeout_ms_ = get_int("timeout_ms", 5);

    front_left_can_id_ = get_can_id("front_left_can_id");
    front_right_can_id_ = get_can_id("front_right_can_id");
    rear_left_can_id_ = get_can_id("rear_left_can_id");
    rear_right_can_id_ = get_can_id("rear_right_can_id");

    enc_counts_per_rev_ = get_int("enc_counts_per_rev", 2048);
    loopback_mode_ = get_bool("loopback_mode", false);

    gear_ratio_ = get_double("gear_ratio", 1.0);
    pid_slot_ = static_cast<uint8_t>(get_int("pid_slot", 0));

    print_commands_ = get_bool("print_commands", false);
    print_status_frames_ = get_bool("print_status", false);
    debug_printing_enabled_ = get_bool("debug_printing_enabled", false);

    command_deadband_rad_per_sec_ =
      get_double("command_deadband_rad_per_sec", 0.001);

    if (gear_ratio_ <= 0.0)
    {
      throw std::runtime_error("gear_ratio must be greater than zero");
    }

    if (pid_slot_ > 3)
    {
      throw std::runtime_error("pid_slot must be between 0 and 3");
    }

    if (serial_baud_rate_ <= 0)
    {
      throw std::runtime_error("serial_baud_rate must be greater than zero");
    }

    if (can_baud_rate_ <= 0)
    {
      throw std::runtime_error("can_baud_rate must be greater than zero");
    }

    if (timeout_ms_ <= 0)
    {
      throw std::runtime_error("timeout_ms must be greater than zero");
    }

    if (enc_counts_per_rev_ <= 0)
    {
      throw std::runtime_error("enc_counts_per_rev must be greater than zero");
    }

    if (command_deadband_rad_per_sec_ < 0.0)
    {
      throw std::runtime_error("command_deadband_rad_per_sec must be >= 0");
    }
  }

  void initialise_joint_storage()
  {
    validate_joint_exists(front_left_wheel_name_);
    validate_joint_exists(front_right_wheel_name_);
    validate_joint_exists(rear_left_wheel_name_);
    validate_joint_exists(rear_right_wheel_name_);

    validate_joint_interfaces(front_left_wheel_name_);
    validate_joint_interfaces(front_right_wheel_name_);
    validate_joint_interfaces(rear_left_wheel_name_);
    validate_joint_interfaces(rear_right_wheel_name_);
  }

  std::string get_required_string(const std::string & name) const
  {
    const auto it = info_.hardware_parameters.find(name);

    if (it == info_.hardware_parameters.end())
    {
      throw std::runtime_error("Missing hardware parameter: " + name);
    }

    if (it->second.empty())
    {
      throw std::runtime_error("Hardware parameter is empty: " + name);
    }

    return it->second;
  }

  int get_int(
    const std::string & name,
    int default_value) const
  {
    const auto it = info_.hardware_parameters.find(name);

    if (it == info_.hardware_parameters.end())
    {
      return default_value;
    }

    return std::stoi(it->second);
  }

  double get_double(
    const std::string & name,
    double default_value) const
  {
    const auto it = info_.hardware_parameters.find(name);

    if (it == info_.hardware_parameters.end())
    {
      return default_value;
    }

    return std::stod(it->second);
  }

  bool get_bool(
    const std::string & name,
    bool default_value) const
  {
    const auto it = info_.hardware_parameters.find(name);

    if (it == info_.hardware_parameters.end())
    {
      return default_value;
    }

    return string_to_bool(it->second);
  }

  uint8_t get_can_id(const std::string & name) const
  {
    const int parsed_id = get_int(name, -1);

    if (parsed_id <= 0 || parsed_id > 63)
    {
      throw std::runtime_error(name + " must be between 1 and 63");
    }

    return static_cast<uint8_t>(parsed_id);
  }

  void validate_joint_exists(const std::string & joint_name) const
  {
    for (const auto & joint : info_.joints)
    {
      if (joint.name == joint_name)
      {
        return;
      }
    }

    throw std::runtime_error(
      "Joint '" + joint_name +
      "' was listed in hardware parameters but does not exist in ros2_control");
  }

  void validate_joint_interfaces(const std::string & joint_name) const
  {
    const hardware_interface::ComponentInfo * joint_info = nullptr;

    for (const auto & joint : info_.joints)
    {
      if (joint.name == joint_name)
      {
        joint_info = &joint;
        break;
      }
    }

    if (joint_info == nullptr)
    {
      throw std::runtime_error("Could not find joint: " + joint_name);
    }

    bool has_velocity_command = false;
    bool has_position_state = false;
    bool has_velocity_state = false;

    for (const auto & command_interface : joint_info->command_interfaces)
    {
      if (command_interface.name == hardware_interface::HW_IF_VELOCITY)
      {
        has_velocity_command = true;
      }
    }

    for (const auto & state_interface : joint_info->state_interfaces)
    {
      if (state_interface.name == hardware_interface::HW_IF_POSITION)
      {
        has_position_state = true;
      }

      if (state_interface.name == hardware_interface::HW_IF_VELOCITY)
      {
        has_velocity_state = true;
      }
    }

    if (!has_velocity_command)
    {
      throw std::runtime_error(
        "Joint '" + joint_name + "' is missing velocity command interface");
    }

    if (!has_position_state)
    {
      throw std::runtime_error(
        "Joint '" + joint_name + "' is missing position state interface");
    }

    if (!has_velocity_state)
    {
      throw std::runtime_error(
        "Joint '" + joint_name + "' is missing velocity state interface");
    }
  }

  void maybe_send_heartbeat()
  {
    const auto now = std::chrono::steady_clock::now();

    if (now < next_heartbeat_time_)
    {
      return;
    }

    if (front_left_spark_)
    {
      if (front_left_spark_->send_heartbeats(false))
      {
        ++heartbeat_count_;
      }
    }

    next_heartbeat_time_ += HEARTBEAT_PERIOD;

    if (next_heartbeat_time_ < now - HEARTBEAT_PERIOD)
    {
      next_heartbeat_time_ = now + HEARTBEAT_PERIOD;
    }
  }

  void send_zero_duty_all(bool print)
  {
    if (front_left_spark_)
    {
      front_left_spark_->set_duty_cycle(0.0f, print);
    }

    if (front_right_spark_)
    {
      front_right_spark_->set_duty_cycle(0.0f, print);
    }

    if (rear_left_spark_)
    {
      rear_left_spark_->set_duty_cycle(0.0f, print);
    }

    if (rear_right_spark_)
    {
      rear_right_spark_->set_duty_cycle(0.0f, print);
    }
  }

  void write_one_motor_native_velocity(
    const std::string & label,
    const std::unique_ptr<SparkMax> & spark,
    double command_rad_per_sec)
  {
    if (!spark)
    {
      RCLCPP_WARN_THROTTLE(
        logger_,
        *rclcpp::Clock::make_shared(),
        1000,
        "Cannot write to %s motor because SparkMax object is null",
        label.c_str());

      return;
    }

    if (!std::isfinite(command_rad_per_sec))
    {
      RCLCPP_WARN_THROTTLE(
        logger_,
        *rclcpp::Clock::make_shared(),
        1000,
        "Command for %s motor is not finite. Skipping this command.",
        label.c_str());

      return;
    }

    const double target_motor_rpm =
      command_rad_per_sec * gear_ratio_ * 60.0 / TWO_PI;

    RCLCPP_WARN_THROTTLE(
      logger_,
      *rclcpp::Clock::make_shared(),
      2000,
      "NATIVE VELOCITY: %s cmd_wheel_rad/s=%.6f target_motor_rpm=%.3f",
      label.c_str(),
      command_rad_per_sec,
      target_motor_rpm);

    const bool ok = spark->set_velocity_rad_per_sec(
      static_cast<float>(command_rad_per_sec),
      print_commands_);

    if (ok)
    {
      ++command_count_;
    }
    else
    {
      RCLCPP_WARN_THROTTLE(
        logger_,
        *rclcpp::Clock::make_shared(),
        1000,
        "Failed to send native velocity command to %s motor",
        label.c_str());
    }
  }

  void send_stop_for_duration(std::chrono::milliseconds duration)
  {
    using clock = std::chrono::steady_clock;

    const auto start = clock::now();
    auto next_command = start;
    auto next_heartbeat = start;

    while (clock::now() - start < duration)
    {
      const auto now = clock::now();

      if (now >= next_heartbeat)
      {
        if (front_left_spark_)
        {
          front_left_spark_->send_heartbeats(false);
        }

        next_heartbeat += HEARTBEAT_PERIOD;

        if (next_heartbeat < now - HEARTBEAT_PERIOD)
        {
          next_heartbeat = now + HEARTBEAT_PERIOD;
        }
      }

      if (now >= next_command)
      {
        send_zero_duty_all(false);

        next_command += STOP_COMMAND_PERIOD;

        if (next_command < now - STOP_COMMAND_PERIOD)
        {
          next_command = now + STOP_COMMAND_PERIOD;
        }
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  void maybe_print_status()
  {
    const auto now = std::chrono::steady_clock::now();

    if (now < last_print_time_)
    {
      return;
    }

    last_print_time_ += PRINT_PERIOD;

    if (last_print_time_ < now - PRINT_PERIOD)
    {
      last_print_time_ = now + PRINT_PERIOD;
    }

    std::cout << std::fixed << std::setprecision(3);

    std::cout << "\n[DiffDriveCanbusHardware SIMPLE NATIVE VELOCITY MODE]\n";
    std::cout << "command_state="
              << (motors_currently_commanded_ ? "ACTIVE_NATIVE_VELOCITY" : "PASSIVE_IDLE")
              << " | command_deadband_rad/s=" << command_deadband_rad_per_sec_
              << " | max_native_wheel_rad/s=" << MAX_NATIVE_WHEEL_RAD_PER_SEC
              << "\n";

    print_motor_status(
      "front_left",
      front_left_can_id_,
      front_left_command_,
      front_left_position_,
      front_left_velocity_,
      front_left_spark_);

    print_motor_status(
      "front_right",
      front_right_can_id_,
      front_right_command_,
      front_right_position_,
      front_right_velocity_,
      front_right_spark_);

    print_motor_status(
      "rear_left",
      rear_left_can_id_,
      rear_left_command_,
      rear_left_position_,
      rear_left_velocity_,
      rear_left_spark_);

    print_motor_status(
      "rear_right",
      rear_right_can_id_,
      rear_right_command_,
      rear_right_position_,
      rear_right_velocity_,
      rear_right_spark_);

    std::cout << "commands_sent=" << command_count_
              << " | idle_writes_skipped=" << skipped_idle_write_count_
              << " | idle_stops_sent=" << idle_stop_count_
              << " | heartbeats=" << heartbeat_count_
              << " | telemetry_reads_skipped=" << telemetry_read_count_
              << " | telemetry_hits=" << telemetry_hit_count_
              << " | can_frames_read=" << can_frames_read_count_
              << " | can_frames_parsed=" << can_frames_parsed_count_
              << "\n";
  }

  void print_motor_status(
    const std::string & label,
    uint8_t can_id,
    double command_rad_per_sec,
    double position_rad,
    double velocity_rad_per_sec,
    const std::unique_ptr<SparkMax> & spark)
  {
    const double deadbanded_command =
      clamp_native_velocity(
        apply_deadband(command_rad_per_sec, command_deadband_rad_per_sec_));

    const double target_motor_rpm =
      deadbanded_command * gear_ratio_ * 60.0 / TWO_PI;

    std::cout << label
              << " id=" << static_cast<int>(can_id)
              << " | raw cmd wheel rad/s=" << command_rad_per_sec
              << " | db/clamped cmd wheel rad/s=" << deadbanded_command
              << " | target motor rpm=" << target_motor_rpm
              << " | ros pos rad=" << position_rad
              << " | ros vel rad/s=" << velocity_rad_per_sec;

    if (!spark)
    {
      std::cout << " | spark=NULL\n";
      return;
    }

    const auto & tel = spark->telemetry();

    if (tel.has_encoder_velocity)
    {
      std::cout << " | measured rpm=" << tel.encoder_velocity_rpm
                << " | motor rad/s=" << tel.motor_rad_per_sec
                << " | wheel rad/s=" << tel.wheel_rad_per_sec;
    }
    else
    {
      std::cout << " | measured rpm=NO_FEEDBACK"
                << " | motor rad/s=NO_FEEDBACK"
                << " | wheel rad/s=NO_FEEDBACK";
    }

    if (tel.has_encoder_position)
    {
      std::cout << " | pos rev=" << tel.encoder_position_rotations
                << " | wheel rev=" << tel.wheel_position_rotations;
    }
    else
    {
      std::cout << " | pos=NO_FEEDBACK";
    }

    if (tel.has_applied_output)
    {
      std::cout << " | applied=" << tel.applied_output;
    }
    else
    {
      std::cout << " | applied=NO_FEEDBACK";
    }

    std::cout << "\n";
  }

private:
  rclcpp::Logger logger_{rclcpp::get_logger("DiffDriveCanbusHardware")};

  CANComms can_;

  std::string front_left_wheel_name_{"front_left_wheel_joint"};
  std::string front_right_wheel_name_{"front_right_wheel_joint"};
  std::string rear_left_wheel_name_{"rear_left_wheel_joint"};
  std::string rear_right_wheel_name_{"rear_right_wheel_joint"};

  std::string serial_device_{"/dev/ttyUSB0"};

  int serial_baud_rate_{2000000};
  int can_baud_rate_{1000000};
  int timeout_ms_{5};
  int enc_counts_per_rev_{2048};

  bool loopback_mode_{false};

  bool print_commands_{false};
  bool print_status_frames_{false};
  bool debug_printing_enabled_{false};

  double gear_ratio_{1.0};
  double command_deadband_rad_per_sec_{0.001};

  uint8_t pid_slot_{0};

  uint8_t front_left_can_id_{1};
  uint8_t front_right_can_id_{2};
  uint8_t rear_left_can_id_{3};
  uint8_t rear_right_can_id_{4};

  std::unique_ptr<SparkMax> front_left_spark_;
  std::unique_ptr<SparkMax> front_right_spark_;
  std::unique_ptr<SparkMax> rear_left_spark_;
  std::unique_ptr<SparkMax> rear_right_spark_;

  double front_left_command_{0.0};
  double front_right_command_{0.0};
  double rear_left_command_{0.0};
  double rear_right_command_{0.0};

  double front_left_position_{0.0};
  double front_right_position_{0.0};
  double rear_left_position_{0.0};
  double rear_right_position_{0.0};

  double front_left_velocity_{0.0};
  double front_right_velocity_{0.0};
  double rear_left_velocity_{0.0};
  double rear_right_velocity_{0.0};

  double front_left_position_offset_{0.0};
  double front_right_position_offset_{0.0};
  double rear_left_position_offset_{0.0};
  double rear_right_position_offset_{0.0};

  bool front_left_position_offset_valid_{false};
  bool front_right_position_offset_valid_{false};
  bool rear_left_position_offset_valid_{false};
  bool rear_right_position_offset_valid_{false};

  int command_count_{0};
  int skipped_idle_write_count_{0};
  int telemetry_read_count_{0};
  int telemetry_hit_count_{0};
  int can_frames_read_count_{0};
  int can_frames_parsed_count_{0};
  int heartbeat_count_{0};
  int idle_stop_count_{0};

  bool motors_currently_commanded_{false};

  std::chrono::steady_clock::time_point next_heartbeat_time_{
    std::chrono::steady_clock::now()};

  std::chrono::steady_clock::time_point last_print_time_{
    std::chrono::steady_clock::now()};
};

}  // namespace diffdrive_canbus

PLUGINLIB_EXPORT_CLASS(
  diffdrive_canbus::DiffDriveCanbusHardware,
  hardware_interface::SystemInterface)