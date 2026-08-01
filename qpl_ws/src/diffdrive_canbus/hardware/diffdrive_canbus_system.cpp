#include "diffdrive_canbus/can_comms.hpp"
#include "diffdrive_canbus/can_device.hpp"
#include "diffdrive_canbus/diffdrive_canbus_system.hpp"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "pluginlib/class_list_macros.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace diffdrive_canbus
{
class DiffDriveCanbusHardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
  {
    if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    logger_ = rclcpp::get_logger("DiffDriveCanbusHardware");

    try
    {
      can_system_ = std::make_unique<CANSystem>(can_, logger_);

      front_left_motor_ = std::make_unique<Motor>("front_left_wheel_joint", 1, can_, 100.0, logger_);
      front_right_motor_ = std::make_unique<Motor>("front_right_wheel_joint", 2, can_, 100.0, logger_);
      rear_left_motor_ = std::make_unique<Motor>("rear_left_wheel_joint", 3, can_, 100.0, logger_);
      rear_right_motor_ = std::make_unique<Motor>("rear_right_wheel_joint", 4, can_, 100.0, logger_);
      left_actuator_ = std::make_unique<Actuator>("left_linear_actuator_joint", 5, can_, logger_);
      right_actuator_ = std::make_unique<Actuator>("right_linear_actuator_joint", 6, can_, logger_);
      drum_motor_ = std::make_unique<Motor>("drum_spin_joint", 7, can_, 125.0, logger_);

      can_system_->add_device(front_left_motor_);
      can_system_->add_device(front_right_motor_);
      can_system_->add_device(rear_left_motor_);
      can_system_->add_device(rear_right_motor_);
      can_system_->add_device(left_actuator_);
      can_system_->add_device(right_actuator_);
      can_system_->add_device(drum_motor_);
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

      can_.connect(serial_device_, serial_baud_rate_, timeout_ms_);

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
      RCLCPP_INFO(logger_, "CAN adapter configured");

      constexpr int pid_slot = 0;
      front_left_motor_->set_native_velocity_pid_slot(pid_slot);
      front_right_motor_->set_native_velocity_pid_slot(pid_slot);
      rear_left_motor_->set_native_velocity_pid_slot(pid_slot);
      rear_right_motor_->set_native_velocity_pid_slot(pid_slot);
      drum_motor_->set_native_velocity_pid_slot(pid_slot);

      left_actuator_->request_actuator_status3_period(can_);
      right_actuator_->request_actuator_status3_period(can_);
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

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    next_drum_command_write_time_ = std::chrono::steady_clock::now();

    next_heartbeat_time_ = std::chrono::steady_clock::now();
    next_feedback_read_time_ = std::chrono::steady_clock::now();
    next_command_write_time_ = std::chrono::steady_clock::now();

    RCLCPP_WARN(logger_, "Performing initial telemetry drain like spark_max_test");

    read_frames(INITIAL_FEEDBACK_FRAMES);

    can_system_->update_joint_state_from_telemetry();

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
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
    read_frames_if_due();
    can_system_->update_joint_state_from_telemetry();
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(
    const rclcpp::Time &,
    const rclcpp::Duration &) override
  {
    send_heartbeat_if_due();
    send_frames_if_due();
    return hardware_interface::return_type::OK;
  }

private:

  void send_heartbeat_if_due()
  {
    const auto now = std::chrono::steady_clock::now();

    if (now < next_heartbeat_time_)
    {
      return;
    }

    next_heartbeat_time_ += HEARTBEAT_PERIOD;
    if (next_heartbeat_time_ < now - HEARTBEAT_PERIOD)
    {
      next_heartbeat_time_ = now + HEARTBEAT_PERIOD;
    }

    can_system_->send_heartbeat();
  }

  void read_frames_if_due()
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

    read_frames(MAX_FEEDBACK_FRAMES_PER_READ);
  }

  void read_frames(int max_frames)
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

      left_actuator_->observe_actuator_raw_frame(frame);
      right_actuator_->observe_actuator_raw_frame(frame);
      can_system_->handle_status_frame(frame);
    }
  }

  void send_frames_if_due() {
    const auto now = std::chrono::steady_clock::now();
    if (now < next_command_write_time_) { return; }
    next_command_write_time_ = now + COMMAND_WRITE_PERIOD;

    front_left_motor_->write();
    front_right_motor_->write();
    rear_left_motor_->write();
    rear_right_motor_->write();
    drum_motor_->write();

    left_actuator_->write_actuator_closed_loop();
    right_actuator_->write_actuator_closed_loop();
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
        if (front_left_motor_)
        {
          front_left_motor_->send_heartbeats(false);
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

  rclcpp::Logger logger_{rclcpp::get_logger("DiffDriveCanbusHardware")};

  CANComms can_;
  std::string serial_device_{"/dev/ttyUSB0"};
  int serial_baud_rate_{2000000};
  int can_baud_rate_{1000000};
  int timeout_ms_{5};

  std::chrono::steady_clock::time_point next_drum_command_write_time_{
    std::chrono::steady_clock::now()};

  std::unique_ptr<CANSystem> can_system_;

  std::unique_ptr<CANDevice> front_left_motor_;
  std::unique_ptr<CANDevice> front_right_motor_;
  std::unique_ptr<CANDevice> rear_left_motor_;
  std::unique_ptr<CANDevice> rear_right_motor_;
  std::unique_ptr<CANDevice> left_actuator_;
  std::unique_ptr<CANDevice> right_actuator_;
  std::unique_ptr<CANDevice> drum_motor_;

  std::chrono::steady_clock::time_point next_heartbeat_time_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point next_feedback_read_time_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point next_command_write_time_{std::chrono::steady_clock::now()};
};

}  // namespace diffdrive_canbus

PLUGINLIB_EXPORT_CLASS(
  diffdrive_canbus::DiffDriveCanbusHardware,
  hardware_interface::SystemInterface)
