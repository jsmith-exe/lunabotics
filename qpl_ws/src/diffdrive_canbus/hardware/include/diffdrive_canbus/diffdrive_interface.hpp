#ifndef DIFFDRIVE_CANBUS__DIFFDRIVE_CANBUS_SYSTEM_HPP_
#define DIFFDRIVE_CANBUS__DIFFDRIVE_CANBUS_SYSTEM_HPP_

#include <chrono>
#include <map>
#include <cmath>
#include <string>
#include <rclcpp/logger.hpp>

#include "diffdrive_canbus/can_device.hpp"

// Toggles
constexpr bool SOFTWARE_SIDE_MOTOR_SMOOTHING = false;
constexpr bool SEND_ZERO_DUTY_FOR_MOTORS = false; // Replaces sending zero velocity with zero duty commands; bypasses PID.

// Constants
constexpr auto CONFIG_FILE_PATH = "qpl_ws/src/diffdrive_canbus/hardware/can_config.yaml";

constexpr double TWO_PI = 2.0 * M_PI;

constexpr auto HEARTBEAT_PERIOD = std::chrono::milliseconds(50);
// Every device's current setpoint is resent each period, so a frame lost on the bus is replaced within one period.
constexpr auto COMMAND_WRITE_PERIOD = std::chrono::milliseconds(100);
constexpr auto STOP_COMMAND_PERIOD = std::chrono::milliseconds(20);
constexpr auto STOP_TIMEOUT = std::chrono::seconds(2);
constexpr double STOPPED_RPM_THRESHOLD = 5.0;
constexpr auto ACTUATOR_RESEND_PERIOD = std::chrono::milliseconds(500);

constexpr auto FEEDBACK_READ_PERIOD = std::chrono::milliseconds(20);
constexpr int MAX_FRAMES_PER_DRAIN = 1000; // Safety cap; a drain normally empties the socket well before this.

constexpr uint32_t SPARKMAX_PERIODIC_STATUS_3_BASE_ID = 0x020518C0; // TODO rename to be clearer  (can comms helpers)
constexpr uint16_t STATUS0_PERIOD_MS = 1000;          // Applied output; not used for control.
constexpr uint16_t STATUS1_PERIOD_MS = 50;            // Velocity; used for joint state and the stop check.
constexpr uint16_t UNUSED_STATUS_PERIOD_MS = 1000;    // Status frames nothing reads, slowed to save bandwidth.
constexpr uint16_t ACTUATOR_STATUS2_PERIOD_MS = 1000; // Actuators read position from status 3, not status 2.

// Motor is capable of going 7500+ when elevated; 5600 is based on the datasheet on https://www.revrobotics.com/rev-21-1650/
constexpr float MAX_MOTOR_RPM = 5600.0;

constexpr double ACTUATOR_POSITION_LOW_PASS_ALPHA = 0.1;
// ^ Constants

namespace diffdrive_canbus {
  // can system
  class CANSystem {
  public:
    CANSystem(rclcpp::Logger &logger) : logger_(logger) {}
    void add_device(const std::shared_ptr<CANDevice> &device);
    void setup_ros_state_interfaces(std::vector<hardware_interface::StateInterface> &state_interfaces);
    void setup_ros_command_interfaces(std::vector<hardware_interface::CommandInterface> &command_interfaces);
    void configure_devices();
    void update_joint_state(const can_frame &frame);
    void send_zero_duty_all();
    void send_heartbeat();
    bool are_all_motors_stopped(double threshold_rpm = STOPPED_RPM_THRESHOLD);

  private:
    std::map<uint8_t, CANDevice*> devices_;
    rclcpp::Logger logger_;
  };

  class Motor : public CANDevice {
  public:
    Motor(const std::string &name, const uint8_t &can_id, SocketCanInterface &can, float gear_ratio, rclcpp::Logger &logger)
      : CANDevice(name, can_id, can, gear_ratio, logger) {}

    void setup_ros_state_interfaces(std::vector<hardware_interface::StateInterface> &state_interfaces) override;
    void setup_ros_command_interfaces(std::vector<hardware_interface::CommandInterface> &command_interfaces) override;

    void configure() override;

    void write() override;
    double preprocess_velocity();
    virtual bool set_velocity(double velocity_to_write);
    void calculate_smoothed_velocity();

    void update_joint_state(const can_frame &frame) override;
    bool handle_status_frame(const can_frame &frame);

    double rotation_position() const override { return rotation_position_; }
    double velocity() const override { return velocity_; }
    double commanded_velocity() const override { return commanded_velocity_; }

  protected:
    // In the context of the Motor class, velocity is a proportion of max RPM. In DiffdriveMotor class, velocity is rad/s.
    double rotation_position_{0.0};
    double velocity_{0.0};
    double commanded_velocity_{0.0};

    double smoothed_velocity_{0.0};
    std::chrono::system_clock::time_point last_write_time_{std::chrono::system_clock::now()};
    double max_rate_of_velocity_change_{0.4}; // If enabled, smooths change

    double position_offset_rad_{0.0};
    bool position_offset_valid_{false};
  };

  class DiffdriveMotor : public Motor {
  public:
    DiffdriveMotor(const std::string &name, const uint8_t &can_id, SocketCanInterface &can, float gear_ratio, rclcpp::Logger &logger)
      : Motor(name, can_id, can, gear_ratio, logger) {
        max_rate_of_velocity_change_ = 2.0;
      }

    bool set_velocity(double velocity) override;
  };

  class Actuator : public CANDevice {
  public:
    Actuator(const std::string &name, const uint8_t &can_id, SocketCanInterface &can, rclcpp::Logger &logger)
      : CANDevice(name, can_id, can, 1.0, logger) {} // TODO remove gear_ratio from actuators and base class

    void setup_ros_state_interfaces(std::vector<hardware_interface::StateInterface> &state_interfaces) override;
    void setup_ros_command_interfaces(std::vector<hardware_interface::CommandInterface> &command_interfaces) override;

    void configure() override;

    void write() override;
    void go_to_position(double setpoint_m);
    void stop();
    double feedback_to_distance(uint16_t raw_voltage_feedback);
    void update_joint_state(const can_frame & frame) override;

    static double min_lift_mm;
    static double max_lift_mm;

  protected:
    double commanded_pos_{0.0};
    double prev_target_mm_{0.0};
    double position_{0.0};
    double filtered_position_mm_{0.0};
    bool stop_sent_{false};
    bool resend_setpoint_{false};
    bool received_position_{false};
    std::chrono::steady_clock::time_point last_resend_time_{std::chrono::steady_clock::now()};
  };

  class SynchronisedActuator : public Actuator {
  public:
    SynchronisedActuator(const std::string &name, const uint8_t &can_id, SocketCanInterface &can,
      rclcpp::Logger &logger, bool is_dominant_actuator)
      : Actuator(name, can_id, can, logger), is_dominant_actuator(is_dominant_actuator) {}

    void set_other_actuator(std::shared_ptr<SynchronisedActuator> &other_actuator) {
      other_actuator_ = other_actuator;
    }

    std::vector<double> generate_resync_points(double from, double to, double max_distance_between_points_mm);

    void write() override;

    static double max_distance_between_points_down_metres;
    static double max_distance_between_points_up_metres;

  protected:
    bool is_dominant_actuator{false};
    std::shared_ptr<SynchronisedActuator> other_actuator_{nullptr};
    std::vector<double> checkpoints_;
    size_t checkpoint_index_{0};
    double prev_commanded_pos_{0.0};
  };
}
#endif  // DIFFDRIVE_CANBUS__DIFFDRIVE_CANBUS_SYSTEM_HPP_