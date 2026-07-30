#include "diffdrive_canbus/can_comms.hpp"
#include "diffdrive_canbus/can_device.hpp"
#include "diffdrive_canbus/diffdrive_canbus_system.hpp"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "pluginlib/class_list_macros.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace diffdrive_canbus
{
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

      can_system_ = std::make_unique<CANSystem>(can_);

      front_left_spark_ = std::make_unique<Motor>(front_left_wheel_name_, 1,
        can_, static_cast<float>(gear_ratio_));

      front_right_spark_ = std::make_unique<Motor>(front_right_wheel_name_, 2,
        can_, static_cast<float>(gear_ratio_));

      rear_left_spark_ = std::make_unique<Motor>(rear_left_wheel_name_, 3,
        can_, static_cast<float>(gear_ratio_));

      rear_right_spark_ = std::make_unique<Motor>(rear_right_wheel_name_, 4,
        can_, static_cast<float>(gear_ratio_));

      left_actuator_spark_ = std::make_unique<Actuator>(left_actuator_name_, 5, can_);

      right_actuator_spark_ = std::make_unique<Actuator>(right_actuator_name_, 6, can_);

      drum_spark_ = std::make_unique<Motor>("drum_spin_joint", 7,
      can_, static_cast<float>(125.0));

      can_system_->add_device(front_left_spark_);
      can_system_->add_device(front_right_spark_);
      can_system_->add_device(rear_left_spark_);
      can_system_->add_device(rear_right_spark_);
      can_system_->add_device(left_actuator_spark_);
      can_system_->add_device(right_actuator_spark_);
      can_system_->add_device(drum_spark_);
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(logger_, "on_init failed: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    can_system_->setup_ros_state_interfaces(state_interfaces);
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    can_system_->setup_ros_command_interfaces(command_interfaces);
    return command_interfaces;
  }

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
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

      front_left_spark_->set_native_velocity_pid_slot(pid_slot_);
      front_right_spark_->set_native_velocity_pid_slot(pid_slot_);
      rear_left_spark_->set_native_velocity_pid_slot(pid_slot_);
      rear_right_spark_->set_native_velocity_pid_slot(pid_slot_);

      RCLCPP_INFO(logger_, "CAN adapter configured");

      left_actuator_spark_->request_actuator_status3_period(can_);
      right_actuator_spark_->request_actuator_status3_period(can_);
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

    next_drum_command_write_time_ = std::chrono::steady_clock::now();

    next_heartbeat_time_ = std::chrono::steady_clock::now();
    next_feedback_read_time_ = std::chrono::steady_clock::now();
    next_rx_drain_time_ = std::chrono::steady_clock::now();
    next_command_write_time_ = std::chrono::steady_clock::now();
    last_print_time_ = std::chrono::steady_clock::now() + PRINT_PERIOD;

    if (feedback_enabled_)
    {
      RCLCPP_WARN(logger_, "Performing initial telemetry drain like spark_max_test");

      read_telemetry_like_test_script(
        INITIAL_FEEDBACK_FRAMES,
        print_status_frames_);

      can_system_->update_joint_state_from_telemetry();
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

      if (left_actuator_spark_)
      {
        left_actuator_spark_->stop(false);
      }

      if (right_actuator_spark_)
      {
        right_actuator_spark_->stop(false);
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
      can_system_->update_joint_state_from_telemetry();
    }
    else
    {
      maybe_drain_can_rx_without_feedback();
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(
    const rclcpp::Time &,
    const rclcpp::Duration &) override
  {
    // TODO validate wheel commands - this was previously just checking isfinite

    // Global non-RIO heartbeat. Keep this independent from individual motor writes.
    maybe_send_heartbeat();

    // Closed-loop actuator position servos are intentionally independent of the wheels.
    left_actuator_spark_->write_actuator_closed_loop();
    right_actuator_spark_->write_actuator_closed_loop();
    write_drum_velocity();

    const double front_left_command = clean_command(front_left_spark_->commanded_velocity(), command_deadband_rad_per_sec_);

    const double front_right_command = clean_command(front_right_spark_->commanded_velocity(), command_deadband_rad_per_sec_);

    const double rear_left_command = clean_command(rear_left_spark_->commanded_velocity(), command_deadband_rad_per_sec_);

    const double rear_right_command = clean_command(rear_right_spark_->commanded_velocity(), command_deadband_rad_per_sec_);

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
  void parse_hardware_parameters()
  {
    front_left_wheel_name_ = get_required_string("front_left_wheel_name");
    front_right_wheel_name_ = get_required_string("front_right_wheel_name");
    rear_left_wheel_name_ = get_required_string("rear_left_wheel_name");
    rear_right_wheel_name_ = get_required_string("rear_right_wheel_name");

    left_actuator_name_ = get_string("left_linear_actuator_joint_name", "left_linear_actuator_joint");
    right_actuator_name_ = get_string("right_linear_actuator_joint_name", "right_linear_actuator_joint");

    serial_device_ = get_required_string("serial_device");

    serial_baud_rate_ = get_int("serial_baud_rate", 2000000);
    can_baud_rate_ = get_int("can_baud_rate", 1000000);

    timeout_ms_ = get_int("timeout_ms", 5);

    enc_counts_per_rev_ = get_int("enc_counts_per_rev", 2048);
    loopback_mode_ = get_bool("loopback_mode", false);

    gear_ratio_ = get_required_double("gear_ratio");

    pid_slot_ = static_cast<uint8_t>(get_int("pid_slot", 0));

    print_status_frames_ = get_bool("print_status", false);
    debug_printing_enabled_ = get_bool("debug_printing_enabled", false);

    feedback_enabled_ = get_bool("feedback_enabled", false);

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
    // TODO note no drum?
    validate_joint_exists(front_left_wheel_name_);
    validate_joint_exists(front_right_wheel_name_);
    validate_joint_exists(rear_left_wheel_name_);
    validate_joint_exists(rear_right_wheel_name_);
    validate_joint_exists(left_actuator_name_);
    validate_joint_exists(right_actuator_name_);

    validate_joint_interfaces(front_left_wheel_name_);
    validate_joint_interfaces(front_right_wheel_name_);
    validate_joint_interfaces(rear_left_wheel_name_);
    validate_joint_interfaces(rear_right_wheel_name_);
    validate_linear_actuator_interfaces(left_actuator_name_);
    validate_linear_actuator_interfaces(right_actuator_name_);
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
  // Parameter parsing stuff up here




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

    if (left_actuator_spark_)
    {
      left_actuator_spark_->set_duty_cycle(0.0f, print);
    }

    if (right_actuator_spark_)
    {
      right_actuator_spark_->set_duty_cycle(0.0f, print);
    }

    if (drum_spark_)
    {
      drum_spark_->set_duty_cycle(0.0f, print);
    }
  }

  bool detect_runaway(
    const std::string & label,
    const std::unique_ptr<CANDevice> & spark,
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
    const std::unique_ptr<CANDevice> & spark,
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

    const double drum_command = drum_spark_->commanded_velocity();
    const double command = std::isfinite(drum_command) ? drum_command : 0.0;
    const double duty = clamp_throttle(command);  // command IS the duty, -1.0 to 1.0

    drum_spark_->set_duty_cycle(static_cast<float>(duty), print_commands_);
    sleep_bus_gap();
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

      if (!can_.read_frame(frame, false))
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

      left_actuator_spark_->observe_actuator_raw_frame(frame);
      right_actuator_spark_->observe_actuator_raw_frame(frame);

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

      if (left_actuator_spark_ &&
          left_actuator_spark_->handle_status_frame(frame, print_status_frames))
      {
        parsed_this_frame = true;
      }

      if (right_actuator_spark_ &&
          right_actuator_spark_->handle_status_frame(frame, print_status_frames))
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

      if (!can_.read_frame(frame, false))
      {
        ++empty_reads;
        ++feedback_empty_count_;
        std::this_thread::sleep_for(FEEDBACK_EMPTY_READ_DELAY);
        continue;
      }

      empty_reads = 0;
      ++frames_read;
      ++can_frames_read_count_;

      left_actuator_spark_->observe_actuator_raw_frame(frame);
      right_actuator_spark_->observe_actuator_raw_frame(frame);

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

      if (left_actuator_spark_ &&
          left_actuator_spark_->handle_status_frame(frame, print_status_frames))
      {
        parsed_this_frame = true;
      }

      if (right_actuator_spark_ &&
          right_actuator_spark_->handle_status_frame(frame, print_status_frames))
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

private:
  rclcpp::Logger logger_{rclcpp::get_logger("DiffDriveCanbusHardware")};

  CANComms can_;

  std::string front_left_wheel_name_{"front_left_wheel_joint"};
  std::string front_right_wheel_name_{"front_right_wheel_joint"};
  std::string rear_left_wheel_name_{"rear_left_wheel_joint"};
  std::string rear_right_wheel_name_{"rear_right_wheel_joint"};
  std::string left_actuator_name_{"left_linear_actuator_joint"};
  std::string right_actuator_name_{"right_linear_actuator_joint"};

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

  std::chrono::steady_clock::time_point next_drum_command_write_time_{
    std::chrono::steady_clock::now()};

  std::unique_ptr<CANDevice> front_left_spark_;
  std::unique_ptr<CANDevice> front_right_spark_;
  std::unique_ptr<CANDevice> rear_left_spark_;
  std::unique_ptr<CANDevice> rear_right_spark_;
  std::unique_ptr<CANDevice> left_actuator_spark_;
  std::unique_ptr<CANDevice> right_actuator_spark_;
  std::unique_ptr<CANDevice> drum_spark_;

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

  std::unique_ptr<CANSystem> can_system_;
};

}  // namespace diffdrive_canbus

PLUGINLIB_EXPORT_CLASS(
  diffdrive_canbus::DiffDriveCanbusHardware,
  hardware_interface::SystemInterface)
