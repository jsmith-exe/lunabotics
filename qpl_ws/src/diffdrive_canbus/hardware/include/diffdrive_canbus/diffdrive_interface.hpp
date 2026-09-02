#ifndef DIFFDRIVE_CANBUS__DIFFDRIVE_CANBUS_SYSTEM_HPP_
#define DIFFDRIVE_CANBUS__DIFFDRIVE_CANBUS_SYSTEM_HPP_

#include <chrono>
#include <map>
#include <cmath>
#include <string>
#include <rclcpp/logger.hpp>
#include "diffdrive_canbus/can_device.hpp"

// Constants
constexpr double TWO_PI = 2.0 * M_PI;

constexpr auto HEARTBEAT_PERIOD = std::chrono::milliseconds(50);
constexpr auto COMMAND_WRITE_PERIOD = std::chrono::milliseconds(100);

// Gap between outgoing serial/CAN writes.
constexpr auto BUS_FRAME_GAP = std::chrono::milliseconds(20);

constexpr auto STOP_TIME = std::chrono::milliseconds(50);
constexpr auto STOP_COMMAND_PERIOD = std::chrono::milliseconds(20);

constexpr auto FEEDBACK_READ_PERIOD = std::chrono::milliseconds(20);
constexpr int MAX_FEEDBACK_FRAMES_PER_READ = 20;
constexpr int FEEDBACK_EMPTY_READ_RETRIES = 8;
constexpr auto FEEDBACK_EMPTY_READ_DELAY = std::chrono::milliseconds(1);
constexpr int INITIAL_FEEDBACK_FRAMES = 50;

constexpr uint32_t SPARKMAX_PERIODIC_STATUS_3_BASE_ID = 0x020518C0; // TODO rename to be clearer  (can comms helpers)
constexpr uint16_t STATUS0_PERIOD_MS = 500;
constexpr uint16_t STATUS1_PERIOD_MS = 500;
constexpr uint16_t STATUS3_PERIOD_MS = 500;

constexpr double MIN_VELOCITY_CHANGE = 0.1;
constexpr double MIN_ACTUATOR_VELOCITY_CHANGE = 0.1;
// ^ Constants

namespace diffdrive_canbus {
  // can system
  class CANSystem {
  public:
    CANSystem(CANComms &comms, rclcpp::Logger &logger) : comms_(comms), logger_(logger) {}
    void add_device(const std::unique_ptr<CANDevice> &device);
    void setup_ros_state_interfaces(std::vector<hardware_interface::StateInterface> &state_interfaces);
    void setup_ros_command_interfaces(std::vector<hardware_interface::CommandInterface> &command_interfaces);
    void configure_devices();
    void update_joint_state(CANFrame &frame);
    void send_zero_duty_all();
    void send_heartbeat();
    bool are_all_motors_stopped();

  private:
    std::map<uint8_t, CANDevice*> devices_;
    CANComms &comms_;
    rclcpp::Logger logger_;
  };

  class Motor : public CANDevice {
  public:
    Motor(const std::string &name, const uint8_t &can_id, CANComms &can, float gear_ratio, rclcpp::Logger &logger)
      : CANDevice(name, can_id, can, gear_ratio, logger) {}

    void setup_ros_state_interfaces(std::vector<hardware_interface::StateInterface> &state_interfaces) override;
    void setup_ros_command_interfaces(std::vector<hardware_interface::CommandInterface> &command_interfaces) override;

    double rotation_position() const override { return rotation_position_; }
    double velocity() const override { return velocity_; }
    double commanded_velocity() const override { return commanded_velocity_; }
    void update_joint_state(const CANFrame &frame) override;
    bool handle_status_frame(const CANFrame &frame, bool print_status_frame);

    void write() override;

  protected:
    double rotation_position_{0.0};
    double velocity_{0.0};
    double commanded_velocity_{0.0}; // TODO rename to command_rad_per_sec??
    double prev_commanded_velocity_{0.0};
    double smoothed_velocity_{0.0};
    std::chrono::system_clock::time_point last_write_time_{std::chrono::system_clock::now()};
    double rate_of_velocity_change_{3.0};
    double position_offset_rad_{0.0};
    bool position_offset_valid_{false};
  };


  class Actuator : public CANDevice {
  public:
    Actuator(const std::string &name, const uint8_t &can_id, CANComms &can, rclcpp::Logger &logger)
      : CANDevice(name, can_id, can, 1.0, logger) {} // TODO remove gear_ratio from actuators and base class

    void setup_ros_state_interfaces(std::vector<hardware_interface::StateInterface> &state_interfaces) override;
    void setup_ros_command_interfaces(std::vector<hardware_interface::CommandInterface> &command_interfaces) override;

    void configure() override;

    void write() override;
    double feedback_to_distance(uint16_t raw_voltage_feedback);
    void update_joint_state(const CANFrame & frame) override;
  private:
    double commanded_pos_mm_{0.0};
    double position_{0.0};
    bool reached_position_{false};
    bool stop_sent_{false};
  };
}
#endif  // DIFFDRIVE_CANBUS__DIFFDRIVE_CANBUS_SYSTEM_HPP_