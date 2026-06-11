#include "diffdrive_canbus/CAN_comms.hpp"
#include "diffdrive_canbus/spark_max.hpp"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "pluginlib/class_list_macros.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace diffdrive_canbus
{

namespace
{

constexpr double TWO_PI = 2.0 * M_PI;

// Keep heartbeats fast. If this is too slow, SPARK MAX duty commands can feel
// jumpy because the controller may briefly drop back into its neutral behaviour.
constexpr auto HEARTBEAT_PERIOD = std::chrono::milliseconds(50);

// Active command traffic. 20 ms keeps the SPARK MAX refreshed often enough
// that it should not repeatedly fall back into neutral/brake behaviour.
constexpr auto COMMAND_WRITE_PERIOD = std::chrono::milliseconds(100);

// Gap between outgoing serial/CAN writes.
// Increase to 10 or 15 ms if the bus is still jumpy.
constexpr auto BUS_FRAME_GAP = std::chrono::milliseconds(10);

constexpr auto STOP_TIME = std::chrono::milliseconds(50);
constexpr auto EXTRA_STOP_TIME = std::chrono::milliseconds(50);
constexpr auto STOP_COMMAND_PERIOD = std::chrono::milliseconds(20);
constexpr auto PRINT_PERIOD = std::chrono::milliseconds(250);

// Feedback copied closer to spark_max_test behaviour.
constexpr auto FEEDBACK_READ_PERIOD = std::chrono::milliseconds(20);
constexpr int MAX_FEEDBACK_FRAMES_PER_READ = 20;
constexpr int FEEDBACK_EMPTY_READ_RETRIES = 8;
constexpr auto FEEDBACK_EMPTY_READ_DELAY = std::chrono::milliseconds(1);
constexpr int INITIAL_FEEDBACK_FRAMES = 50;

// Even when feedback_enabled is false, keep draining a small number of incoming
// CAN frames. Some serial-to-CAN adapters behave badly if the RX buffer fills
// with SPARK MAX status frames, and actuator duty commands can then appear
// jumpy or fail to register unless feedback is enabled.
constexpr auto RX_DRAIN_PERIOD = std::chrono::milliseconds(10);
constexpr int RX_DRAIN_FRAMES_PER_READ = 20;
constexpr int RX_DRAIN_EMPTY_READ_RETRIES = 2;

// Linear actuator feedback/status readback.
// This is read-only telemetry for both actuators. It does not control the actuators yet.
constexpr auto LINEAR_ACTUATOR_FEEDBACK_PRINT_PERIOD = std::chrono::milliseconds(500);

// SPARK MAX Periodic Status 3 contains analogue sensor information.
// The non-FRC CAN reference uses base ID 0x020518C0 + device_id for Status 3.
constexpr uint32_t SPARKMAX_PERIODIC_STATUS_3_BASE_ID = 0x020518C0;
constexpr uint16_t LINEAR_ACTUATOR_STATUS3_PERIOD_MS = 20;

// Runaway watchdog.
// These are deliberately conservative for catching obvious runaway,
// not for doing normal closed-loop control.
constexpr double RUNAWAY_MIN_MEASURED_RPM = 8000.0;
constexpr double RUNAWAY_ALLOWED_RPM_ERROR = 1500.0;
constexpr double RUNAWAY_SMALL_TARGET_RPM = 500.0;
constexpr double RUNAWAY_SIGN_TARGET_MIN_RPM = 50.0;
constexpr double RUNAWAY_HIGH_APPLIED_OUTPUT = 1.0;
constexpr auto RUNAWAY_STOP_TIME = std::chrono::milliseconds(100);

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

double clean_command(double value, double deadband)
{
  if (!std::isfinite(value))
  {
    return 0.0;
  }

  return apply_deadband(value, deadband);
}

// Clamp duty-cycle/throttle commands to the safe SPARK MAX range.
double clamp_throttle(double value)
{
  if (!std::isfinite(value))
  {
    return 0.0;
  }

  return std::clamp(value, -1.0, 1.0);
}

double apply_throttle_deadband(double value, double deadband)
{
  const double safe_deadband = std::max(0.0, deadband);

  if (std::fabs(value) <= safe_deadband)
  {
    return 0.0;
  }

  return value;
}


uint8_t get_frc_device_id_from_can_id(uint32_t can_id)
{
  // FRC extended CAN IDs store the device ID in the lowest 6 bits.
  return static_cast<uint8_t>(can_id & 0x3F);
}

uint8_t get_frc_api_index_from_can_id(uint32_t can_id)
{
  // The API index is the nibble immediately above the 6-bit device ID.
  return static_cast<uint8_t>((can_id >> 6) & 0x0F);
}

bool is_actuator_status3_id(uint32_t can_id, uint8_t device_id)
{
  const uint32_t clean_id = can_id & 0x1FFFFFFF;
  return clean_id == (SPARKMAX_PERIODIC_STATUS_3_BASE_ID + static_cast<uint32_t>(device_id));
}

uint16_t le_u16_from_frame_data(const uint8_t data[8], size_t offset)
{
  if (offset + sizeof(uint16_t) > 8)
  {
    return 0;
  }

  uint16_t value = 0;
  std::memcpy(&value, data + offset, sizeof(uint16_t));
  return value;
}

std::string can_data_to_hex_string(const uint8_t data[8], uint8_t dlc)
{
  std::ostringstream oss;
  oss << std::hex << std::uppercase << std::setfill('0');

  for (uint8_t i = 0; i < dlc && i < 8; ++i)
  {
    if (i > 0)
    {
      oss << ' ';
    }

    oss << "0x" << std::setw(2) << static_cast<int>(data[i]);
  }

  return oss.str();
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
      drum_can_id_ = static_cast<uint8_t>(get_int("drum_can_id", 7));
      initialise_joint_storage();
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(logger_, "on_init failed: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_WARN(logger_, "============================================================");
    RCLCPP_WARN(logger_, "DIFFDRIVE CANBUS HARDWARE IS RUNNING IN SIMPLE NATIVE VELOCITY MODE");
    RCLCPP_WARN(logger_, "Runtime command path remains based on the best working version");
    RCLCPP_WARN(logger_, "Heartbeat is only sent while commanding or stopping");
    RCLCPP_WARN(logger_, "Active command stream is refreshed quickly for SPARK MAX keepalive stability");
    RCLCPP_WARN(logger_, "Heartbeat is handled globally by maybe_send_heartbeat()");
    RCLCPP_WARN(logger_, "Active commands use native SPARK MAX velocity setpoints");
    RCLCPP_WARN(logger_, "Left linear actuator on CAN ID %u uses direct duty-cycle/throttle output", left_actuator_.can_id);
    RCLCPP_WARN(logger_, "Right linear actuator on CAN ID %u uses direct duty-cycle/throttle output", right_actuator_.can_id);
    RCLCPP_WARN(logger_, "Left linear actuator command interface: %s/%s", left_actuator_.joint_name.c_str(), hardware_interface::HW_IF_POSITION);
    RCLCPP_WARN(logger_, "Right linear actuator command interface: %s/%s", right_actuator_.joint_name.c_str(), hardware_interface::HW_IF_POSITION);
    RCLCPP_WARN(logger_, "Left linear actuator ros2_control interface enabled: %s", left_actuator_.ros2_control_interface_enabled ? "true" : "false");
    RCLCPP_WARN(logger_, "Right linear actuator ros2_control interface enabled: %s", right_actuator_.ros2_control_interface_enabled ? "true" : "false");
    RCLCPP_WARN(logger_, "If disabled, actuator command comes from test_position_command only");
    RCLCPP_WARN(logger_, "Linear actuators use direct open-loop duty-cycle output at the top of every write() cycle");
    RCLCPP_WARN(logger_, "When feedback is disabled, CAN RX frames are still lightly drained for serial adapter stability");
    RCLCPP_WARN(logger_, "Linear actuator raw CAN frames are sniffed for analogue-voltage feedback without editing spark_max.cpp");
    RCLCPP_WARN(logger_, "Linear actuator ros2_control command is position: 0.0 retract, 0.5 stop, 1.0 extend");
    RCLCPP_WARN(logger_, "Shutdown/Ctrl+C uses zero-duty stop bursts");
    RCLCPP_WARN(logger_, "Velocity clamp removed");
    RCLCPP_WARN(logger_, "NaN/+inf/-inf commands are forced to zero before deadband");
    RCLCPP_WARN(logger_, "Gear ratio is REQUIRED from ros2_control hardware params");
    RCLCPP_WARN(logger_, "Feedback enabled: %s", feedback_enabled_ ? "true" : "false");
    RCLCPP_WARN(logger_, "Feedback path copied closer to spark_max_test read_telemetry behaviour");
    RCLCPP_WARN(logger_, "Command logs include cached SPARK MAX readback when available");
    RCLCPP_WARN(logger_, "Runaway watchdog is enabled");
    RCLCPP_WARN(logger_, "Runaway stop time: %ld ms", RUNAWAY_STOP_TIME.count());
    RCLCPP_WARN(logger_, "Command write period: %ld ms", COMMAND_WRITE_PERIOD.count());
    RCLCPP_WARN(logger_, "Bus frame gap: %ld ms", BUS_FRAME_GAP.count());
    RCLCPP_WARN(logger_, "============================================================");

    RCLCPP_INFO(logger_, "Initialised DiffDriveCanbusHardware");
    RCLCPP_INFO(logger_, "Serial device: %s", serial_device_.c_str());
    RCLCPP_INFO(logger_, "Serial baud rate: %d", serial_baud_rate_);
    RCLCPP_INFO(logger_, "CAN baud rate: %d", can_baud_rate_);
    RCLCPP_INFO(logger_, "Timeout: %d ms", timeout_ms_);
    RCLCPP_INFO(logger_, "Gear ratio from ros2_control: %.3f motor rev / wheel rev", gear_ratio_);
    RCLCPP_INFO(logger_, "Command deadband: %.6f rad/s", command_deadband_rad_per_sec_);
    RCLCPP_INFO(logger_, "PID slot: %u", pid_slot_);
    RCLCPP_INFO(logger_, "Loopback mode: %s", loopback_mode_ ? "true" : "false");

    RCLCPP_INFO(logger_, "Debug flag print_commands: %s", print_commands_ ? "true" : "false");
    RCLCPP_INFO(logger_, "Debug flag print_status: %s", print_status_frames_ ? "true" : "false");
    RCLCPP_INFO(
      logger_,
      "Debug flag debug_printing_enabled: %s",
      debug_printing_enabled_ ? "true" : "false");

    RCLCPP_INFO(
      logger_,
      "Left linear actuator ros2_control interface enabled: %s",
      left_actuator_.ros2_control_interface_enabled ? "true" : "false");

    RCLCPP_INFO(
      logger_,
      "Right linear actuator ros2_control interface enabled: %s",
      right_actuator_.ros2_control_interface_enabled ? "true" : "false");

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

    if (left_actuator_.ros2_control_interface_enabled)
    {
      state_interfaces.emplace_back(
        left_actuator_.joint_name,
        hardware_interface::HW_IF_POSITION,
        &left_actuator_.position);
    }

    if (right_actuator_.ros2_control_interface_enabled)
    {
      state_interfaces.emplace_back(
        right_actuator_.joint_name,
        hardware_interface::HW_IF_POSITION,
        &right_actuator_.position);
    }

    state_interfaces.emplace_back(
      "drum_spin_joint",
      hardware_interface::HW_IF_POSITION,
      &drum_position_);

    state_interfaces.emplace_back(
      "drum_spin_joint",
      hardware_interface::HW_IF_VELOCITY,
      &drum_velocity_);

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

    if (left_actuator_.ros2_control_interface_enabled)
    {
      command_interfaces.emplace_back(
        left_actuator_.joint_name,
        hardware_interface::HW_IF_POSITION,
        &left_actuator_.command);
    }

    if (right_actuator_.ros2_control_interface_enabled)
    {
      command_interfaces.emplace_back(
        right_actuator_.joint_name,
        hardware_interface::HW_IF_POSITION,
        &right_actuator_.command);
    }

    command_interfaces.emplace_back(
      "drum_spin_joint",
      hardware_interface::HW_IF_VELOCITY,
      &drum_command_);

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
      RCLCPP_WARN(logger_, "Using gear ratio %.3f from ros2_control for all SparkMax objects", gear_ratio_);
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

      left_actuator_.spark = std::make_unique<SparkMax>(
        can_,
        left_actuator_.can_id,
        static_cast<float>(gear_ratio_));

      right_actuator_.spark = std::make_unique<SparkMax>(
        can_,
        right_actuator_.can_id,
        static_cast<float>(gear_ratio_));

      drum_spark_ = std::make_unique<SparkMax>(
        can_,
        drum_can_id_,
        static_cast<float>(125.0));

      front_left_spark_->set_native_velocity_pid_slot(pid_slot_);
      front_right_spark_->set_native_velocity_pid_slot(pid_slot_);
      rear_left_spark_->set_native_velocity_pid_slot(pid_slot_);
      rear_right_spark_->set_native_velocity_pid_slot(pid_slot_);

      RCLCPP_INFO(logger_, "CAN adapter configured");
      RCLCPP_INFO(logger_, "Front left SPARK MAX ID: %u", front_left_can_id_);
      RCLCPP_INFO(logger_, "Front right SPARK MAX ID: %u", front_right_can_id_);
      RCLCPP_INFO(logger_, "Rear left SPARK MAX ID: %u", rear_left_can_id_);
      RCLCPP_INFO(logger_, "Rear right SPARK MAX ID: %u", rear_right_can_id_);
      RCLCPP_INFO(logger_, "Drum spin SPARK MAX ID: %u", drum_can_id_);
      RCLCPP_INFO(logger_, "Left linear actuator SPARK MAX ID: %u", left_actuator_.can_id);
      RCLCPP_INFO(logger_, "Right linear actuator SPARK MAX ID: %u", right_actuator_.can_id);
      RCLCPP_INFO(logger_, "Left linear actuator test position command starts at: %.3f", left_actuator_.test_position_command);
      RCLCPP_INFO(logger_, "Right linear actuator test position command starts at: %.3f", right_actuator_.test_position_command);
      RCLCPP_INFO(logger_, "Left linear actuator ros2_control position command starts at: %.3f", left_actuator_.command);
      RCLCPP_INFO(logger_, "Right linear actuator ros2_control position command starts at: %.3f", right_actuator_.command);
      RCLCPP_INFO(logger_, "Actuator commands are resent every write() cycle, independent of wheel commands");

      request_actuator_status3_period(left_actuator_, "left");
      request_actuator_status3_period(right_actuator_, "right");
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

    reset_actuator_state(left_actuator_);
    reset_actuator_state(right_actuator_);

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
    non_finite_command_count_ = 0;
    feedback_cycle_count_ = 0;
    feedback_empty_count_ = 0;
    runaway_count_ = 0;

    motors_currently_commanded_ = false;
    runaway_latched_ = false;

    drum_command_ = 0.0;
    drum_position_ = 0.0;
    drum_velocity_ = 0.0;
    drum_position_offset_ = 0.0;
    drum_position_offset_valid_ = false;
    next_drum_command_write_time_ = std::chrono::steady_clock::now();

    next_heartbeat_time_ = std::chrono::steady_clock::now();
    next_feedback_read_time_ = std::chrono::steady_clock::now();
    next_rx_drain_time_ = std::chrono::steady_clock::now();
    next_command_write_time_ = std::chrono::steady_clock::now();
    last_print_time_ = std::chrono::steady_clock::now() + PRINT_PERIOD;

    RCLCPP_WARN(logger_, "Activated in simple native velocity mode");
    RCLCPP_WARN(logger_, "No SPARK MAX command frames sent during activate()");
    RCLCPP_WARN(logger_, "Gear ratio active: %.3f motor rev / wheel rev", gear_ratio_);
    RCLCPP_WARN(logger_, "Feedback enabled: %s", feedback_enabled_ ? "true" : "false");
    RCLCPP_WARN(logger_, "Runaway latch reset");
    RCLCPP_WARN(logger_, "Left linear actuator CAN ID %u position command starts at %.3f", left_actuator_.can_id, left_actuator_.command);
    RCLCPP_WARN(logger_, "Right linear actuator CAN ID %u position command starts at %.3f", right_actuator_.can_id, right_actuator_.command);
    RCLCPP_WARN(logger_, "First heartbeat will only be sent once a real command or stop is being sent");
    RCLCPP_WARN(logger_, "Active velocity commands limited to one command batch every %ld ms", COMMAND_WRITE_PERIOD.count());
    RCLCPP_WARN(logger_, "Each heartbeat/command frame is separated by %ld ms", BUS_FRAME_GAP.count());

    if (feedback_enabled_)
    {
      RCLCPP_WARN(logger_, "Performing initial telemetry drain like spark_max_test");

      read_telemetry_like_test_script(
        INITIAL_FEEDBACK_FRAMES,
        print_status_frames_);

      update_all_joint_states_from_telemetry();
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &) override
  {
    RCLCPP_WARN(logger_, "Deactivating DiffDriveCanbusHardware");
    RCLCPP_WARN(logger_, "Sending guaranteed zero-duty stop burst to all motors and actuators");

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

      if (left_actuator_.spark)
      {
        left_actuator_.spark->stop(false);
      }

      if (right_actuator_.spark)
      {
        right_actuator_.spark->stop(false);
      }

      if (drum_spark_)
      {
        drum_spark_->stop(false);
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

    if (feedback_enabled_)
    {
      maybe_read_feedback_like_test_script();
      update_all_joint_states_from_telemetry();
    }
    else
    {
      maybe_drain_can_rx_without_feedback();
    }

    maybe_print_actuator_feedback(left_actuator_, "left");
    maybe_print_actuator_feedback(right_actuator_, "right");

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
    if (
      !std::isfinite(front_left_command_) ||
      !std::isfinite(front_right_command_) ||
      !std::isfinite(rear_left_command_) ||
      !std::isfinite(rear_right_command_))
    {
      ++non_finite_command_count_;

      RCLCPP_WARN_THROTTLE(
        logger_,
        *rclcpp::Clock::make_shared(),
        1000,
        "Non-finite ros2_control command detected. Forcing non-finite commands to zero.");
    }

    // Global non-RIO heartbeat. Keep this independent from individual motor writes.
    maybe_send_heartbeat();

    // Open-loop actuator commands are intentionally independent of the wheels.
    write_actuator_open_loop(left_actuator_, "left");
    write_actuator_open_loop(right_actuator_, "right");
    write_drum_velocity();

    const double front_left_command =
      clean_command(front_left_command_, command_deadband_rad_per_sec_);

    const double front_right_command =
      clean_command(front_right_command_, command_deadband_rad_per_sec_);

    const double rear_left_command =
      clean_command(rear_left_command_, command_deadband_rad_per_sec_);

    const double rear_right_command =
      clean_command(rear_right_command_, command_deadband_rad_per_sec_);

    const bool any_wheel_command =
      std::fabs(front_left_command) > 0.0 ||
      std::fabs(front_right_command) > 0.0 ||
      std::fabs(rear_left_command) > 0.0 ||
      std::fabs(rear_right_command) > 0.0;

    const bool any_active_command = any_wheel_command;

    if (runaway_latched_)
    {
      if (!any_active_command)
      {
        RCLCPP_WARN(
          logger_,
          "Runaway latch cleared because ros2_control command returned to zero.");

        runaway_latched_ = false;
      }
      else
      {
        RCLCPP_ERROR_THROTTLE(
          logger_,
          *rclcpp::Clock::make_shared(),
          1000,
          "Runaway latch active. Blocking velocity commands and sending zero-duty stop frames.");

        maybe_send_heartbeat();
        send_zero_duty_wheels_only(false);

        return hardware_interface::return_type::OK;
      }
    }

    if (!any_active_command)
    {
      ++skipped_idle_write_count_;

      if (motors_currently_commanded_)
      {
        ++idle_stop_count_;

        RCLCPP_WARN_THROTTLE(
          logger_,
          *rclcpp::Clock::make_shared(),
          1000,
          "Commands returned to zero. Sending immediate zero-duty stop frames to drive motors only.");

        if (front_left_spark_)
        {
            front_left_spark_->send_heartbeats(false);
            sleep_bus_gap();
        }
        // Send several stop frames to improve reliability
        for (int i = 0; i < 3; ++i)
        {
            send_zero_duty_wheels_only(false);
            sleep_bus_gap();
        }
        maybe_send_heartbeat();
        send_zero_duty_wheels_only(false);
      }

      motors_currently_commanded_ = false;

      return hardware_interface::return_type::OK;
    }

    const auto now = std::chrono::steady_clock::now();

    if (now < next_command_write_time_)
    {
      return hardware_interface::return_type::OK;
    }

    next_command_write_time_ = now + COMMAND_WRITE_PERIOD;

    if (!any_wheel_command)
    {
      motors_currently_commanded_ = false;
      return hardware_interface::return_type::OK;
    }

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
  // All per-actuator state bundled so left and right share identical logic.
  struct LinearActuatorHW
  {
    std::string joint_name{"linear_actuator_joint"};
    uint8_t can_id{5};

    double test_position_command{0.5};
    double command{0.5};
    double position{0.0};
    double voltage{0.0};
    bool has_voltage{false};

    double feedback_min_voltage{0.279};
    double feedback_max_voltage{1.85};
    double deadband{0.02};
    double last_sent_output{999.0};

    std::unique_ptr<SparkMax> spark;

    int command_count{0};
    bool has_raw_can_frame{false};
    bool has_status3_frame{false};
    bool has_analog_voltage_candidate{false};
    double analog_voltage_candidate{0.0};
    std::string analog_voltage_source{"NONE"};

    uint32_t last_raw_can_id{0};
    uint8_t last_raw_dlc{0};
    std::array<uint8_t, 8> last_raw_data{};

    uint32_t last_status3_can_id{0};
    uint8_t last_status3_dlc{0};
    std::array<uint8_t, 8> last_status3_data{};

    int raw_can_frame_count{0};
    int status3_frame_count{0};

    bool currently_commanded{false};
    bool ros2_control_interface_enabled{false};

    std::chrono::steady_clock::time_point last_feedback_print_time{};
  };

  void parse_hardware_parameters()
  {
    front_left_wheel_name_ = get_required_string("front_left_wheel_name");
    front_right_wheel_name_ = get_required_string("front_right_wheel_name");
    rear_left_wheel_name_ = get_required_string("rear_left_wheel_name");
    rear_right_wheel_name_ = get_required_string("rear_right_wheel_name");

    left_actuator_.joint_name =
      get_string("left_linear_actuator_joint_name", "left_linear_actuator_joint");
    right_actuator_.joint_name =
      get_string("right_linear_actuator_joint_name", "right_linear_actuator_joint");

    serial_device_ = get_required_string("serial_device");

    serial_baud_rate_ = get_int("serial_baud_rate", 2000000);
    can_baud_rate_ = get_int("can_baud_rate", 1000000);

    timeout_ms_ = get_int("timeout_ms", 5);

    front_left_can_id_ = get_can_id("front_left_can_id");
    front_right_can_id_ = get_can_id("front_right_can_id");
    rear_left_can_id_ = get_can_id("rear_left_can_id");
    rear_right_can_id_ = get_can_id("rear_right_can_id");

    left_actuator_.can_id = static_cast<uint8_t>(get_int("left_linear_actuator_can_id", 5));
    if (left_actuator_.can_id <= 0 || left_actuator_.can_id > 63)
    {
      throw std::runtime_error("left_linear_actuator_can_id must be between 1 and 63");
    }

    right_actuator_.can_id = static_cast<uint8_t>(get_int("right_linear_actuator_can_id", 6));
    if (right_actuator_.can_id <= 0 || right_actuator_.can_id > 63)
    {
      throw std::runtime_error("right_linear_actuator_can_id must be between 1 and 63");
    }

    left_actuator_.test_position_command = std::clamp(
      get_double("left_linear_actuator_initial_position_command", 0.5), 0.0, 1.0);
    left_actuator_.command = left_actuator_.test_position_command;

    right_actuator_.test_position_command = std::clamp(
      get_double("right_linear_actuator_initial_position_command", 0.5), 0.0, 1.0);
    right_actuator_.command = right_actuator_.test_position_command;

    left_actuator_.deadband =
      std::clamp(get_double("left_linear_actuator_deadband", 0.02), 0.0, 0.5);
    right_actuator_.deadband =
      std::clamp(get_double("right_linear_actuator_deadband", 0.02), 0.0, 0.5);

    left_actuator_.feedback_min_voltage =
      get_double("left_linear_actuator_feedback_min_voltage", 0.279);
    left_actuator_.feedback_max_voltage =
      get_double("left_linear_actuator_feedback_max_voltage", 1.85);

    right_actuator_.feedback_min_voltage =
      get_double("right_linear_actuator_feedback_min_voltage", 0.279);
    right_actuator_.feedback_max_voltage =
      get_double("right_linear_actuator_feedback_max_voltage", 1.85);

    enc_counts_per_rev_ = get_int("enc_counts_per_rev", 2048);
    loopback_mode_ = get_bool("loopback_mode", false);

    gear_ratio_ = get_required_double("gear_ratio");

    pid_slot_ = static_cast<uint8_t>(get_int("pid_slot", 0));

    print_commands_ = get_bool("print_commands", false);
    print_status_frames_ = get_bool("print_status", false);
    debug_printing_enabled_ = get_bool("debug_printing_enabled", false);

    feedback_enabled_ = get_bool("feedback_enabled", false);

    left_actuator_.ros2_control_interface_enabled =
      get_bool("left_linear_actuator_ros2_control_interface_enabled", false);
    right_actuator_.ros2_control_interface_enabled =
      get_bool("right_linear_actuator_ros2_control_interface_enabled", false);

    command_deadband_rad_per_sec_ =
      get_double("command_deadband_rad_per_sec", 0.001);

    if (gear_ratio_ <= 0.0)
    {
      throw std::runtime_error("gear_ratio must be greater than zero");
    }

    if (left_actuator_.feedback_max_voltage <= left_actuator_.feedback_min_voltage)
    {
      throw std::runtime_error(
        "left_linear_actuator_feedback_max_voltage must be greater than left_linear_actuator_feedback_min_voltage");
    }

    if (right_actuator_.feedback_max_voltage <= right_actuator_.feedback_min_voltage)
    {
      throw std::runtime_error(
        "right_linear_actuator_feedback_max_voltage must be greater than right_linear_actuator_feedback_min_voltage");
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

    if (left_actuator_.ros2_control_interface_enabled)
    {
      validate_joint_exists(left_actuator_.joint_name);
      validate_linear_actuator_interfaces(left_actuator_.joint_name);
    }

    if (right_actuator_.ros2_control_interface_enabled)
    {
      validate_joint_exists(right_actuator_.joint_name);
      validate_linear_actuator_interfaces(right_actuator_.joint_name);
    }
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

  std::string get_string(
    const std::string & name,
    const std::string & default_value) const
  {
    const auto it = info_.hardware_parameters.find(name);

    if (it == info_.hardware_parameters.end())
    {
      return default_value;
    }

    return it->second;
  }

  double get_required_double(const std::string & name) const
  {
    const auto it = info_.hardware_parameters.find(name);

    if (it == info_.hardware_parameters.end())
    {
      throw std::runtime_error(
        "Missing required hardware parameter: " + name +
        ". Add <param name=\"" + name + "\">100.0</param> to the ros2_control hardware block.");
    }

    if (it->second.empty())
    {
      throw std::runtime_error("Hardware parameter is empty: " + name);
    }

    return std::stod(it->second);
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

  void validate_linear_actuator_interfaces(const std::string & joint_name) const
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

    bool has_position_command = false;
    bool has_position_state = false;

    for (const auto & command_interface : joint_info->command_interfaces)
    {
      if (command_interface.name == hardware_interface::HW_IF_POSITION)
      {
        has_position_command = true;
      }
    }

    for (const auto & state_interface : joint_info->state_interfaces)
    {
      if (state_interface.name == hardware_interface::HW_IF_POSITION)
      {
        has_position_state = true;
      }
    }

    if (!has_position_command)
    {
      throw std::runtime_error(
        "Joint '" + joint_name + "' is missing position command interface for actuator target");
    }

    if (!has_position_state)
    {
      throw std::runtime_error(
        "Joint '" + joint_name + "' is missing position state interface for actuator feedback");
    }
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

  void reset_actuator_state(LinearActuatorHW & act)
  {
    act.command = std::clamp(act.test_position_command, 0.0, 1.0);
    act.position = 0.0;
    act.voltage = 0.0;
    act.has_voltage = false;
    act.command_count = 0;
    act.last_sent_output = 999.0;
    act.has_raw_can_frame = false;
    act.has_status3_frame = false;
    act.has_analog_voltage_candidate = false;
    act.analog_voltage_candidate = 0.0;
    act.analog_voltage_source = "NONE";
    act.last_raw_can_id = 0;
    act.last_raw_dlc = 0;
    act.last_raw_data.fill(0);
    act.last_status3_can_id = 0;
    act.last_status3_dlc = 0;
    act.last_status3_data.fill(0);
    act.raw_can_frame_count = 0;
    act.status3_frame_count = 0;
    act.currently_commanded = false;
    act.last_feedback_print_time =
      std::chrono::steady_clock::now() + LINEAR_ACTUATOR_FEEDBACK_PRINT_PERIOD;
  }

  void sleep_bus_gap()
  {
    std::this_thread::sleep_for(BUS_FRAME_GAP);
  }

  void send_heartbeat_before_motor_command(const std::string & label)
  {
    if (!front_left_spark_)
    {
      return;
    }

    const bool ok = front_left_spark_->send_heartbeats(false);

    if (ok)
    {
      ++heartbeat_count_;
    }
    else
    {
      RCLCPP_WARN_THROTTLE(
        logger_,
        *rclcpp::Clock::make_shared(),
        1000,
        "Failed to send heartbeat before %s motor command",
        label.c_str());
    }

    sleep_bus_gap();
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

  void send_zero_duty_wheels_only(bool print)
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

    if (left_actuator_.spark)
    {
      left_actuator_.spark->set_duty_cycle(0.0f, print);
    }

    if (right_actuator_.spark)
    {
      right_actuator_.spark->set_duty_cycle(0.0f, print);
    }

    if (drum_spark_)
    {
      drum_spark_->set_duty_cycle(0.0f, print);
    }
  }

  bool detect_runaway(
    const std::string & label,
    const std::unique_ptr<SparkMax> & spark,
    double target_motor_rpm)
  {
    if (!spark)
    {
      return false;
    }

    const auto & tel = spark->telemetry();

    if (!tel.has_encoder_velocity)
    {
      return false;
    }

    const double measured_rpm =
      static_cast<double>(tel.encoder_velocity_rpm);

    const double rpm_error =
      measured_rpm - target_motor_rpm;

    const bool target_small =
      std::fabs(target_motor_rpm) < RUNAWAY_SMALL_TARGET_RPM;

    const bool measured_large =
      std::fabs(measured_rpm) > RUNAWAY_MIN_MEASURED_RPM;

    const bool huge_error =
      std::fabs(rpm_error) > RUNAWAY_ALLOWED_RPM_ERROR;

    const bool sign_opposed =
      std::fabs(target_motor_rpm) > RUNAWAY_SIGN_TARGET_MIN_RPM &&
      std::fabs(measured_rpm) > RUNAWAY_MIN_MEASURED_RPM &&
      ((target_motor_rpm > 0.0 && measured_rpm < 0.0) ||
       (target_motor_rpm < 0.0 && measured_rpm > 0.0));

    const bool high_applied =
      tel.has_applied_output &&
      std::fabs(static_cast<double>(tel.applied_output)) >
        RUNAWAY_HIGH_APPLIED_OUTPUT;

    const bool runaway =
      (target_small && measured_large) ||
      (huge_error && measured_large && high_applied) ||
      sign_opposed;

    if (runaway)
    {
      if (tel.has_applied_output)
      {
        RCLCPP_ERROR(
          logger_,
          "RUNAWAY DETECTED on %s: target_motor_rpm=%.3f measured_motor_rpm=%.3f rpm_error=%.3f applied=%.3f",
          label.c_str(),
          target_motor_rpm,
          measured_rpm,
          rpm_error,
          static_cast<double>(tel.applied_output));
      }
      else
      {
        RCLCPP_ERROR(
          logger_,
          "RUNAWAY DETECTED on %s: target_motor_rpm=%.3f measured_motor_rpm=%.3f rpm_error=%.3f applied=NO_FEEDBACK",
          label.c_str(),
          target_motor_rpm,
          measured_rpm,
          rpm_error);
      }
    }

    return runaway;
  }

  void latch_runaway_and_stop()
  {
    ++runaway_count_;
    runaway_latched_ = true;

    RCLCPP_ERROR(
      logger_,
      "Runaway latched. Sending zero-duty stop burst and blocking velocity commands until command returns to zero.");

    send_stop_for_duration(RUNAWAY_STOP_TIME);
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

    const auto & tel = spark->telemetry();

    const bool has_velocity = tel.has_encoder_velocity;
    const bool has_applied = tel.has_applied_output;

    if (has_velocity && has_applied)
    {
      RCLCPP_WARN_THROTTLE(
        logger_,
        *rclcpp::Clock::make_shared(),
        500,
        "NATIVE VELOCITY: %s cmd_wheel_rad/s=%.6f gear_ratio=%.3f target_motor_rpm=%.3f | READBACK measured_motor_rpm=%.3f measured_wheel_rad/s=%.6f applied=%.3f",
        label.c_str(),
        command_rad_per_sec,
        gear_ratio_,
        target_motor_rpm,
        static_cast<double>(tel.encoder_velocity_rpm),
        static_cast<double>(tel.wheel_rad_per_sec),
        static_cast<double>(tel.applied_output));
    }
    else if (has_velocity && !has_applied)
    {
      RCLCPP_WARN_THROTTLE(
        logger_,
        *rclcpp::Clock::make_shared(),
        500,
        "NATIVE VELOCITY: %s cmd_wheel_rad/s=%.6f gear_ratio=%.3f target_motor_rpm=%.3f | READBACK measured_motor_rpm=%.3f measured_wheel_rad/s=%.6f applied=NO_FEEDBACK",
        label.c_str(),
        command_rad_per_sec,
        gear_ratio_,
        target_motor_rpm,
        static_cast<double>(tel.encoder_velocity_rpm),
        static_cast<double>(tel.wheel_rad_per_sec));
    }
    else if (!has_velocity && has_applied)
    {
      RCLCPP_WARN_THROTTLE(
        logger_,
        *rclcpp::Clock::make_shared(),
        500,
        "NATIVE VELOCITY: %s cmd_wheel_rad/s=%.6f gear_ratio=%.3f target_motor_rpm=%.3f | READBACK measured_motor_rpm=NO_FEEDBACK measured_wheel_rad/s=NO_FEEDBACK applied=%.3f",
        label.c_str(),
        command_rad_per_sec,
        gear_ratio_,
        target_motor_rpm,
        static_cast<double>(tel.applied_output));
    }
    else
    {
      RCLCPP_WARN_THROTTLE(
        logger_,
        *rclcpp::Clock::make_shared(),
        500,
        "NATIVE VELOCITY: %s cmd_wheel_rad/s=%.6f gear_ratio=%.3f target_motor_rpm=%.3f | READBACK measured_motor_rpm=NO_FEEDBACK measured_wheel_rad/s=NO_FEEDBACK applied=NO_FEEDBACK",
        label.c_str(),
        command_rad_per_sec,
        gear_ratio_,
        target_motor_rpm);
    }

    if (detect_runaway(label, spark, target_motor_rpm))
    {
      latch_runaway_and_stop();
      return;
    }

    if (runaway_latched_)
    {
      return;
    }

    send_heartbeat_before_motor_command(label);

    const bool ok = spark->set_velocity_rad_per_sec(
      static_cast<float>(command_rad_per_sec),
      print_commands_);

    sleep_bus_gap();

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

  void request_actuator_status3_period(LinearActuatorHW & act, const std::string & label)
  {
    const uint32_t status3_id =
      SPARKMAX_PERIODIC_STATUS_3_BASE_ID + static_cast<uint32_t>(act.can_id);

    std::vector<uint8_t> data(2, 0x00);
    data[0] = static_cast<uint8_t>(LINEAR_ACTUATOR_STATUS3_PERIOD_MS & 0xFF);
    data[1] = static_cast<uint8_t>((LINEAR_ACTUATOR_STATUS3_PERIOD_MS >> 8) & 0xFF);

    const bool ok = can_.send_extended_frame(status3_id, data, print_commands_);

    if (ok)
    {
      RCLCPP_WARN(
        logger_,
        "Requested SPARK MAX Status 3 period for %s linear actuator: id=%u can_id=0x%08X period=%u ms",
        label.c_str(),
        act.can_id,
        status3_id,
        LINEAR_ACTUATOR_STATUS3_PERIOD_MS);
    }
    else
    {
      RCLCPP_WARN(
        logger_,
        "Failed to request SPARK MAX Status 3 period for %s linear actuator: id=%u can_id=0x%08X",
        label.c_str(),
        act.can_id,
        status3_id);
    }

    sleep_bus_gap();
  }

  void write_actuator_open_loop(LinearActuatorHW & act, const std::string & label)
  {
    if (!act.spark)
    {
      RCLCPP_WARN_THROTTLE(
        logger_,
        *rclcpp::Clock::make_shared(),
        1000,
        "Cannot write to %s linear actuator because SparkMax object is null",
        label.c_str());

      return;
    }

    const double safe_position_command =
      std::clamp(
        std::isfinite(act.command) ? act.command : 0.5,
        0.0,
        1.0);

    // Present the actuator to ros2_control as a position command from 0 to 1,
    // but keep the SPARK MAX in reliable open-loop duty mode internally:
    //   0.0 -> -1.0 duty, retract
    //   0.5 ->  0.0 duty, stop
    //   1.0 ->  1.0 duty, extend
    double safe_throttle = (2.0 * safe_position_command) - 1.0;

    safe_throttle =
      apply_throttle_deadband(
        clamp_throttle(safe_throttle),
        act.deadband);

    if (std::fabs(safe_throttle) <= act.deadband)
    {
      safe_throttle = 0.0;
    }

    RCLCPP_WARN_THROTTLE(
      logger_,
      *rclcpp::Clock::make_shared(),
      500,
      "%s LINEAR ACTUATOR POSITION CMD: id=%u target_pos=%.3f duty_sent=%.3f",
      label.c_str(),
      act.can_id,
      safe_position_command,
      safe_throttle);

    const bool ok = act.spark->set_duty_cycle(
      static_cast<float>(safe_throttle),
      print_commands_);

    sleep_bus_gap();

    if (ok)
    {
      ++act.command_count;
      act.last_sent_output = safe_throttle;
      act.currently_commanded = std::fabs(safe_throttle) > 0.0;
    }
    else
    {
      RCLCPP_WARN_THROTTLE(
        logger_,
        *rclcpp::Clock::make_shared(),
        1000,
        "Failed to send open-loop duty command to %s linear actuator on CAN ID %u",
        label.c_str(),
        act.can_id);
    }
  }

  void write_drum_velocity()
  {
    if (!drum_spark_)
    {
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (now < next_drum_command_write_time_)
    {
      return;
    }
    next_drum_command_write_time_ = now + COMMAND_WRITE_PERIOD;

    const double command = std::isfinite(drum_command_) ? drum_command_ : 0.0;
    const double duty = clamp_throttle(command);  // command IS the duty, -1.0 to 1.0

    RCLCPP_WARN_THROTTLE(
      logger_,
      *rclcpp::Clock::make_shared(),
      500,
      "DRUM: cmd=%.4f duty_sent=%.4f",
      command,
      duty);

    drum_spark_->set_duty_cycle(static_cast<float>(duty), print_commands_);
    sleep_bus_gap();
  }

  void maybe_print_actuator_feedback(LinearActuatorHW & act, const std::string & label)
  {
    const auto now = std::chrono::steady_clock::now();

    if (now < act.last_feedback_print_time)
    {
      return;
    }

    act.last_feedback_print_time += LINEAR_ACTUATOR_FEEDBACK_PRINT_PERIOD;

    if (act.last_feedback_print_time < now - LINEAR_ACTUATOR_FEEDBACK_PRINT_PERIOD)
    {
      act.last_feedback_print_time = now + LINEAR_ACTUATOR_FEEDBACK_PRINT_PERIOD;
    }

    if (!act.spark)
    {
      RCLCPP_WARN_THROTTLE(
        logger_,
        *rclcpp::Clock::make_shared(),
        1000,
        "%s LINEAR ACTUATOR FEEDBACK: spark=NULL",
        label.c_str());

      return;
    }

    const auto & tel = act.spark->telemetry();

    if (act.has_analog_voltage_candidate)
    {
      RCLCPP_WARN(
        logger_,
        "%s LINEAR ACTUATOR ANALOG: id=%u cmd=%.3f voltage=%.4f V position=%.4f source=%s status3_frames=%d raw_frames=%d status3_can_id=0x%08X dlc=%u data=[%s] applied=%s%.3f",
        label.c_str(),
        act.can_id,
        std::clamp(act.command, 0.0, 1.0),
        act.analog_voltage_candidate,
        act.position,
        act.analog_voltage_source.c_str(),
        act.status3_frame_count,
        act.raw_can_frame_count,
        act.last_status3_can_id,
        act.last_status3_dlc,
        can_data_to_hex_string(act.last_status3_data.data(), act.last_status3_dlc).c_str(),
        tel.has_applied_output ? "" : "NO_",
        tel.has_applied_output ? static_cast<double>(tel.applied_output) : 0.0);

      return;
    }

    if (act.has_status3_frame)
    {
      RCLCPP_WARN(
        logger_,
        "%s LINEAR ACTUATOR ANALOG RAW: id=%u cmd=%.3f STATUS3_SEEN_BUT_NO_VOLTAGE_CANDIDATE status3_frames=%d raw_frames=%d last_can_id=0x%08X dlc=%u data=[%s] applied=%s%.3f",
        label.c_str(),
        act.can_id,
        std::clamp(act.command, 0.0, 1.0),
        act.status3_frame_count,
        act.raw_can_frame_count,
        act.last_raw_can_id,
        act.last_raw_dlc,
        can_data_to_hex_string(act.last_raw_data.data(), act.last_raw_dlc).c_str(),
        tel.has_applied_output ? "" : "NO_",
        tel.has_applied_output ? static_cast<double>(tel.applied_output) : 0.0);

      return;
    }

    if (act.has_raw_can_frame)
    {
      RCLCPP_WARN(
        logger_,
        "%s LINEAR ACTUATOR ANALOG RAW: id=%u cmd=%.3f NO_STATUS3_ANALOG_FRAME_YET raw_frames=%d last_can_id=0x%08X api_index=%u dlc=%u data=[%s] applied=%s%.3f",
        label.c_str(),
        act.can_id,
        std::clamp(act.command, 0.0, 1.0),
        act.raw_can_frame_count,
        act.last_raw_can_id,
        get_frc_api_index_from_can_id(act.last_raw_can_id),
        act.last_raw_dlc,
        can_data_to_hex_string(act.last_raw_data.data(), act.last_raw_dlc).c_str(),
        tel.has_applied_output ? "" : "NO_",
        tel.has_applied_output ? static_cast<double>(tel.applied_output) : 0.0);

      return;
    }

    RCLCPP_WARN(
      logger_,
      "%s LINEAR ACTUATOR ANALOG RAW: id=%u cmd=%.3f no CAN frames from actuator decoded yet",
      label.c_str(),
      act.can_id,
      std::clamp(act.command, 0.0, 1.0));
  }

  void print_actuator_status(LinearActuatorHW & act, const std::string & label)
  {
    const double safe_position_command = std::clamp(act.command, 0.0, 1.0);
    const double safe_throttle = (2.0 * safe_position_command) - 1.0;

    std::cout << label << "_linear_actuator"
              << " id=" << static_cast<int>(act.can_id)
              << " | command position=" << safe_position_command
              << " | duty=" << safe_throttle
              << " | position=" << act.position;

    if (act.has_voltage)
    {
      std::cout << " | voltage=" << act.voltage;
    }
    else
    {
      std::cout << " | voltage=NO_ANALOG";
    }

    if (!act.spark)
    {
      std::cout << " | spark=NULL\n";
      return;
    }

    const auto & tel = act.spark->telemetry();

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

  double normalise_actuator_voltage(double voltage, double min_voltage, double max_voltage) const
  {
    const double span = max_voltage - min_voltage;

    if (!std::isfinite(voltage) || std::fabs(span) < 1e-9)
    {
      return 0.0;
    }

    return std::clamp(
      (voltage - min_voltage) / span,
      0.0,
      1.0);
  }

  void observe_actuator_raw_frame(LinearActuatorHW & act, const CANFrame & frame)
  {
    if (get_frc_device_id_from_can_id(frame.id) != act.can_id)
    {
      return;
    }

    act.has_raw_can_frame = true;
    ++act.raw_can_frame_count;

    act.last_raw_can_id = frame.id;
    act.last_raw_dlc = static_cast<uint8_t>(std::min<int>(frame.dlc, 8));
    act.last_raw_data.fill(0);

    for (uint8_t i = 0; i < act.last_raw_dlc; ++i)
    {
      act.last_raw_data[i] = frame.data[i];
    }

    // SPARK MAX analogue sensor data is expected on Periodic Status 3.
    // Use the exact Status 3 CAN ID instead of only the API index so Status 0/1/2
    // frames cannot be mistaken for analogue feedback.
    if (!is_actuator_status3_id(frame.id, act.can_id))
    {
      return;
    }

    act.has_status3_frame = true;
    ++act.status3_frame_count;

    // Per the non-FRC SPARK MAX CAN reference, Periodic Status 3 starts with
    // adcVoltage in 2q8 fixed-point format. That means the raw 16-bit
    // little-endian value is voltage * 256.
    const uint16_t raw_adc_voltage_2q8 = le_u16_from_frame_data(frame.data, 0);
    const double analog_voltage = static_cast<double>(raw_adc_voltage_2q8) / 256.0;

    act.has_analog_voltage_candidate = true;
    act.analog_voltage_candidate = analog_voltage;
    act.analog_voltage_source = "status3 adcVoltage 2q8 bytes0-1 /256";

    act.voltage = analog_voltage;
    act.has_voltage = true;
    act.position =
      normalise_actuator_voltage(analog_voltage, act.feedback_min_voltage, act.feedback_max_voltage);

    act.last_status3_can_id = frame.id;
    act.last_status3_dlc = static_cast<uint8_t>(std::min<int>(frame.dlc, 8));
    act.last_status3_data.fill(0);
    for (uint8_t i = 0; i < act.last_status3_dlc; ++i)
    {
      act.last_status3_data[i] = frame.data[i];
    }
  }

  void maybe_read_feedback_like_test_script()
  {
    const auto now = std::chrono::steady_clock::now();

    if (now < next_feedback_read_time_)
    {
      return;
    }

    next_feedback_read_time_ += FEEDBACK_READ_PERIOD;

    if (next_feedback_read_time_ < now - FEEDBACK_READ_PERIOD)
    {
      next_feedback_read_time_ = now + FEEDBACK_READ_PERIOD;
    }

    ++feedback_cycle_count_;

    read_telemetry_like_test_script(
      MAX_FEEDBACK_FRAMES_PER_READ,
      print_status_frames_);
  }

  void maybe_drain_can_rx_without_feedback()
  {
    const auto now = std::chrono::steady_clock::now();

    if (now < next_rx_drain_time_)
    {
      return;
    }

    next_rx_drain_time_ += RX_DRAIN_PERIOD;

    if (next_rx_drain_time_ < now - RX_DRAIN_PERIOD)
    {
      next_rx_drain_time_ = now + RX_DRAIN_PERIOD;
    }

    drain_can_rx_without_feedback(
      RX_DRAIN_FRAMES_PER_READ,
      false);
  }

  bool drain_can_rx_without_feedback(
    int max_frames,
    bool print_status_frames)
  {
    bool drained_any = false;
    int frames_read = 0;
    int empty_reads = 0;

    while (frames_read < max_frames && empty_reads < RX_DRAIN_EMPTY_READ_RETRIES)
    {
      CANFrame frame;

      if (!can_.read_can_frame(frame, false))
      {
        ++empty_reads;
        ++feedback_empty_count_;
        std::this_thread::sleep_for(FEEDBACK_EMPTY_READ_DELAY);
        continue;
      }

      empty_reads = 0;
      ++frames_read;
      ++can_frames_read_count_;
      drained_any = true;

      observe_actuator_raw_frame(left_actuator_, frame);
      observe_actuator_raw_frame(right_actuator_, frame);

      // Parse status frames into cached telemetry if they happen to match, but
      // do not update ros2_control joint state when feedback is disabled.
      bool parsed_this_frame = false;

      if (front_left_spark_ &&
          front_left_spark_->handle_status_frame(frame, print_status_frames))
      {
        parsed_this_frame = true;
      }

      if (front_right_spark_ &&
          front_right_spark_->handle_status_frame(frame, print_status_frames))
      {
        parsed_this_frame = true;
      }

      if (rear_left_spark_ &&
          rear_left_spark_->handle_status_frame(frame, print_status_frames))
      {
        parsed_this_frame = true;
      }

      if (rear_right_spark_ &&
          rear_right_spark_->handle_status_frame(frame, print_status_frames))
      {
        parsed_this_frame = true;
      }

      if (left_actuator_.spark &&
          left_actuator_.spark->handle_status_frame(frame, print_status_frames))
      {
        parsed_this_frame = true;
      }

      if (right_actuator_.spark &&
          right_actuator_.spark->handle_status_frame(frame, print_status_frames))
      {
        parsed_this_frame = true;
      }

      if (drum_spark_ &&
          drum_spark_->handle_status_frame(frame, print_status_frames))
      {
        parsed_this_frame = true;
      }

      if (parsed_this_frame)
      {
        ++can_frames_parsed_count_;
      }
    }

    if (drained_any)
    {
      ++telemetry_hit_count_;
    }

    return drained_any;
  }

  bool read_telemetry_like_test_script(
    int max_frames,
    bool print_status_frames)
  {
    bool parsed_any = false;
    int frames_read = 0;
    int empty_reads = 0;

    while (frames_read < max_frames && empty_reads < FEEDBACK_EMPTY_READ_RETRIES)
    {
      CANFrame frame;

      if (!can_.read_can_frame(frame, false))
      {
        ++empty_reads;
        ++feedback_empty_count_;
        std::this_thread::sleep_for(FEEDBACK_EMPTY_READ_DELAY);
        continue;
      }

      empty_reads = 0;
      ++frames_read;
      ++can_frames_read_count_;

      observe_actuator_raw_frame(left_actuator_, frame);
      observe_actuator_raw_frame(right_actuator_, frame);

      bool parsed_this_frame = false;

      if (front_left_spark_ &&
          front_left_spark_->handle_status_frame(frame, print_status_frames))
      {
        parsed_this_frame = true;
      }

      if (front_right_spark_ &&
          front_right_spark_->handle_status_frame(frame, print_status_frames))
      {
        parsed_this_frame = true;
      }

      if (rear_left_spark_ &&
          rear_left_spark_->handle_status_frame(frame, print_status_frames))
      {
        parsed_this_frame = true;
      }

      if (rear_right_spark_ &&
          rear_right_spark_->handle_status_frame(frame, print_status_frames))
      {
        parsed_this_frame = true;
      }

      if (left_actuator_.spark &&
          left_actuator_.spark->handle_status_frame(frame, print_status_frames))
      {
        parsed_this_frame = true;
      }

      if (right_actuator_.spark &&
          right_actuator_.spark->handle_status_frame(frame, print_status_frames))
      {
        parsed_this_frame = true;
      }

      if (drum_spark_ &&
          drum_spark_->handle_status_frame(frame, print_status_frames))
      {
        parsed_this_frame = true;
      }

      if (parsed_this_frame)
      {
        parsed_any = true;
        ++can_frames_parsed_count_;
      }
    }

    if (parsed_any)
    {
      ++telemetry_hit_count_;
    }

    return parsed_any;
  }

  void update_all_joint_states_from_telemetry()
  {
    update_joint_state_from_telemetry(
      front_left_spark_,
      front_left_position_,
      front_left_velocity_,
      front_left_position_offset_,
      front_left_position_offset_valid_);

    update_joint_state_from_telemetry(
      front_right_spark_,
      front_right_position_,
      front_right_velocity_,
      front_right_position_offset_,
      front_right_position_offset_valid_);

    update_joint_state_from_telemetry(
      rear_left_spark_,
      rear_left_position_,
      rear_left_velocity_,
      rear_left_position_offset_,
      rear_left_position_offset_valid_);

    update_joint_state_from_telemetry(
      rear_right_spark_,
      rear_right_position_,
      rear_right_velocity_,
      rear_right_position_offset_,
      rear_right_position_offset_valid_);

    update_joint_state_from_telemetry(
      drum_spark_,
      drum_position_,
      drum_velocity_,
      drum_position_offset_,
      drum_position_offset_valid_);
  }

  void update_joint_state_from_telemetry(
    const std::unique_ptr<SparkMax> & spark,
    double & position_rad,
    double & velocity_rad_per_sec,
    double & position_offset_rad,
    bool & position_offset_valid)
  {
    if (!spark)
    {
      return;
    }

    const auto & tel = spark->telemetry();

    if (tel.has_encoder_velocity)
    {
      velocity_rad_per_sec = static_cast<double>(tel.wheel_rad_per_sec);
    }

    if (tel.has_encoder_position)
    {
      const double absolute_position_rad =
        static_cast<double>(tel.wheel_position_rotations) * TWO_PI;

      if (!position_offset_valid)
      {
        position_offset_rad = absolute_position_rad;
        position_offset_valid = true;
      }

      position_rad = absolute_position_rad - position_offset_rad;
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
              << " | velocity_clamp=DISABLED"
              << " | command_write_period_ms=" << COMMAND_WRITE_PERIOD.count()
              << " | bus_frame_gap_ms=" << BUS_FRAME_GAP.count()
              << " | gear_ratio=" << gear_ratio_
              << " | feedback=" << (feedback_enabled_ ? "ENABLED" : "DISABLED")
              << " | runaway_latched=" << (runaway_latched_ ? "YES" : "NO")
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
              << " | non_finite_commands=" << non_finite_command_count_
              << " | runaway_count=" << runaway_count_
              << " | read_calls=" << telemetry_read_count_
              << " | telemetry_hits=" << telemetry_hit_count_
              << " | feedback_cycles=" << feedback_cycle_count_
              << " | feedback_empty=" << feedback_empty_count_
              << " | can_frames_read=" << can_frames_read_count_
              << " | can_frames_parsed=" << can_frames_parsed_count_
              << " | left_actuator_commands_sent=" << left_actuator_.command_count
              << " | right_actuator_commands_sent=" << right_actuator_.command_count
              << "\n";

    print_actuator_status(left_actuator_, "left");
    print_actuator_status(right_actuator_, "right");
  }

  void print_motor_status(
    const std::string & label,
    uint8_t can_id,
    double command_rad_per_sec,
    double position_rad,
    double velocity_rad_per_sec,
    const std::unique_ptr<SparkMax> & spark)
  {
    const double cleaned_command =
      clean_command(command_rad_per_sec, command_deadband_rad_per_sec_);

    const double target_motor_rpm =
      cleaned_command * gear_ratio_ * 60.0 / TWO_PI;

    std::cout << label
              << " id=" << static_cast<int>(can_id)
              << " | raw cmd wheel rad/s=" << command_rad_per_sec
              << " | clean cmd wheel rad/s=" << cleaned_command
              << " | gear_ratio=" << gear_ratio_
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
  bool feedback_enabled_{false};

  double gear_ratio_{1.0};
  double command_deadband_rad_per_sec_{0.001};

  uint8_t pid_slot_{0};

  uint8_t front_left_can_id_{1};
  uint8_t front_right_can_id_{2};
  uint8_t rear_left_can_id_{3};
  uint8_t rear_right_can_id_{4};

  LinearActuatorHW left_actuator_;
  LinearActuatorHW right_actuator_;

  std::unique_ptr<SparkMax> drum_spark_;
  uint8_t drum_can_id_{7};
  double drum_command_{0.0};
  std::chrono::steady_clock::time_point next_drum_command_write_time_{
    std::chrono::steady_clock::now()};

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
  int non_finite_command_count_{0};
  int feedback_cycle_count_{0};
  int feedback_empty_count_{0};
  int runaway_count_{0};

  bool motors_currently_commanded_{false};
  bool runaway_latched_{false};

  double drum_position_{0.0};
  double drum_velocity_{0.0};
  double drum_position_offset_{0.0};
  bool drum_position_offset_valid_{false};

  std::chrono::steady_clock::time_point next_heartbeat_time_{
    std::chrono::steady_clock::now()};

  std::chrono::steady_clock::time_point next_feedback_read_time_{
    std::chrono::steady_clock::now()};

  std::chrono::steady_clock::time_point next_rx_drain_time_{
    std::chrono::steady_clock::now()};

  std::chrono::steady_clock::time_point next_command_write_time_{
    std::chrono::steady_clock::now()};

  std::chrono::steady_clock::time_point last_print_time_{
    std::chrono::steady_clock::now()};
};

}  // namespace diffdrive_canbus

PLUGINLIB_EXPORT_CLASS(
  diffdrive_canbus::DiffDriveCanbusHardware,
  hardware_interface::SystemInterface)
