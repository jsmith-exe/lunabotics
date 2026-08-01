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
    if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
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
        can_, static_cast<float>(100.0));

      front_right_spark_ = std::make_unique<Motor>(front_right_wheel_name_, 2,
        can_, static_cast<float>(100.0));

      rear_left_spark_ = std::make_unique<Motor>(rear_left_wheel_name_, 3,
        can_, static_cast<float>(100.0));

      rear_right_spark_ = std::make_unique<Motor>(rear_right_wheel_name_, 4,
        can_, static_cast<float>(100.0));

      left_actuator_spark_ = std::make_unique<Actuator>(left_actuator_name_, 5, can_);

      right_actuator_spark_ = std::make_unique<Actuator>(right_actuator_name_, 6, can_);

      drum_spark_ = std::make_unique<Motor>(drum_spin_name_, 7,
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
        CANMode::NORMAL,
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
    next_drum_command_write_time_ = std::chrono::steady_clock::now();

    next_heartbeat_time_ = std::chrono::steady_clock::now();
    next_feedback_read_time_ = std::chrono::steady_clock::now();
    next_command_write_time_ = std::chrono::steady_clock::now();

    RCLCPP_WARN(logger_, "Performing initial telemetry drain like spark_max_test");

    read_telemetry_like_test_script(INITIAL_FEEDBACK_FRAMES);

    can_system_->update_joint_state_from_telemetry();

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &) override
  {
    RCLCPP_WARN(logger_, "Deactivating DiffDriveCanbusHardware");
    RCLCPP_WARN(logger_, "Sending guaranteed zero-duty stop burst to all motors and actuators");

    try
    {
      can_system_->send_zero_duty_all();
      send_stop_for_duration(STOP_TIME);
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
    maybe_read_feedback_like_test_script();
    can_system_->update_joint_state_from_telemetry();
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(
    const rclcpp::Time &,
    const rclcpp::Duration &) override
  {
    // Global non-RIO heartbeat. Keep this independent from individual motor writes.
    maybe_send_heartbeat();

    // if (CANSystem::runaway_latched_)
    // {
    //   if (!any_active_command)
    //   {
    //     CANSystem::runaway_latched_ = false;
    //   }
    //   else
    //   {
    //     maybe_send_heartbeat();
    //     can_system_->send_zero_duty_all();
    //
    //     return hardware_interface::return_type::OK;
    //   }
    // }

    const auto now = std::chrono::steady_clock::now();
    if (now < next_command_write_time_)
    {
      return hardware_interface::return_type::OK;
    }
    next_command_write_time_ = now + COMMAND_WRITE_PERIOD;

    motors_currently_commanded_ = true;

    front_left_spark_->write_one_motor_native_velocity();
    front_right_spark_->write_one_motor_native_velocity();
    rear_left_spark_->write_one_motor_native_velocity();
    rear_right_spark_->write_one_motor_native_velocity();
    drum_spark_->write_one_motor_native_velocity();

    left_actuator_spark_->write_actuator_closed_loop();
    right_actuator_spark_->write_actuator_closed_loop();

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

    pid_slot_ = static_cast<uint8_t>(get_int("pid_slot", 0));

    command_deadband_rad_per_sec_ =
      get_double("command_deadband_rad_per_sec", 0.001);

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
    validate_joint_exists(drum_spin_name_);

    validate_joint_interfaces(front_left_wheel_name_);
    validate_joint_interfaces(front_right_wheel_name_);
    validate_joint_interfaces(rear_left_wheel_name_);
    validate_joint_interfaces(rear_right_wheel_name_);
    validate_linear_actuator_interfaces(left_actuator_name_);
    validate_linear_actuator_interfaces(right_actuator_name_);
    validate_joint_interfaces(drum_spin_name_);
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





  void maybe_send_heartbeat()
  {
    const auto now = std::chrono::steady_clock::now();

    if (now < next_heartbeat_time_)
    {
      return;
    }

    if (front_left_spark_)
    {
      front_left_spark_->send_heartbeats(false);
    }

    next_heartbeat_time_ += HEARTBEAT_PERIOD;

    if (next_heartbeat_time_ < now - HEARTBEAT_PERIOD)
    {
      next_heartbeat_time_ = now + HEARTBEAT_PERIOD;
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
        can_system_->send_zero_duty_all();

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

    // TODO fix this - shouldn't be '< now - FEEDBACK_READ_PERIOD', could just be '< now'. Alternatively, always set to now + FEEDBACK_READ_PERIOD
    if (next_feedback_read_time_ < now - FEEDBACK_READ_PERIOD)
    {
      next_feedback_read_time_ = now + FEEDBACK_READ_PERIOD;
    }

    read_telemetry_like_test_script(MAX_FEEDBACK_FRAMES_PER_READ);
  }

  void read_telemetry_like_test_script(int max_frames)
  {
    int frames_read = 0;
    int empty_reads = 0;

    while (frames_read < max_frames && empty_reads < FEEDBACK_EMPTY_READ_RETRIES)
    {
      CANFrame frame;

      if (!can_.read_frame(frame, false))
      {
        ++empty_reads;
        std::this_thread::sleep_for(FEEDBACK_EMPTY_READ_DELAY);
        continue;
      }

      empty_reads = 0;
      ++frames_read;

      left_actuator_spark_->observe_actuator_raw_frame(frame);
      right_actuator_spark_->observe_actuator_raw_frame(frame);
      can_system_->handle_status_frame(frame);
    }
  }

private:
  rclcpp::Logger logger_{rclcpp::get_logger("DiffDriveCanbusHardware")};

  CANComms can_;
  std::string serial_device_{"/dev/ttyUSB0"};
  int serial_baud_rate_{2000000};
  int can_baud_rate_{1000000};
  int timeout_ms_{5};

  std::string front_left_wheel_name_{"front_left_wheel_joint"};
  std::string front_right_wheel_name_{"front_right_wheel_joint"};
  std::string rear_left_wheel_name_{"rear_left_wheel_joint"};
  std::string rear_right_wheel_name_{"rear_right_wheel_joint"};
  std::string left_actuator_name_{"left_linear_actuator_joint"};
  std::string right_actuator_name_{"right_linear_actuator_joint"};
  std::string drum_spin_name_{"drum_spin_joint"};

  double command_deadband_rad_per_sec_{0.001};

  uint8_t pid_slot_{0};

  std::chrono::steady_clock::time_point next_drum_command_write_time_{
    std::chrono::steady_clock::now()};

  std::unique_ptr<CANSystem> can_system_;

  std::unique_ptr<CANDevice> front_left_spark_;
  std::unique_ptr<CANDevice> front_right_spark_;
  std::unique_ptr<CANDevice> rear_left_spark_;
  std::unique_ptr<CANDevice> rear_right_spark_;
  std::unique_ptr<CANDevice> left_actuator_spark_;
  std::unique_ptr<CANDevice> right_actuator_spark_;
  std::unique_ptr<CANDevice> drum_spark_;

  bool motors_currently_commanded_{false};

  std::chrono::steady_clock::time_point next_heartbeat_time_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point next_feedback_read_time_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point next_command_write_time_{std::chrono::steady_clock::now()};
};

}  // namespace diffdrive_canbus

PLUGINLIB_EXPORT_CLASS(
  diffdrive_canbus::DiffDriveCanbusHardware,
  hardware_interface::SystemInterface)
