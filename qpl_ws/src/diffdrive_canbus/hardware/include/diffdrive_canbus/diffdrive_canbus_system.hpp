#ifndef DIFFDRIVE_CANBUS__DIFFDRIVE_CANBUS_SYSTEM_HPP_
#define DIFFDRIVE_CANBUS__DIFFDRIVE_CANBUS_SYSTEM_HPP_

#include <chrono>
#include <map>
#include <cmath>
#include <string>
#include <rclcpp/logger.hpp>
#include "diffdrive_canbus/can_device.hpp"

#pragma region Constants
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
// The analog position feedback is used to close the loop on the actuator
// position command (see write_actuator_closed_loop), not just for telemetry.
constexpr auto LINEAR_ACTUATOR_FEEDBACK_PRINT_PERIOD = std::chrono::milliseconds(500);

// Closed-loop linear-actuator servo limits. The actuators are open-loop duty
// devices, so we close the loop on the analog position feedback: a held position
// command drives toward the target and then stops, instead of stalling at an
// end-stop. This is bang-bang (full drive or stop), not a PID controller.
//   ACTUATOR_FEEDBACK_TIMEOUT      - if no fresh position feedback within this
//                                    window, hold the motor stopped (never drive blind).
//   ACTUATOR_STALL_TIMEOUT         - if commanding motion but the measured
//                                    position stops progressing for this long
//                                    (jam, hard end-stop, frozen feedback), latch off.
//   ACTUATOR_STALL_MOTION_EPSILON  - position change that counts as "still moving".
constexpr auto ACTUATOR_FEEDBACK_TIMEOUT = std::chrono::milliseconds(500);
constexpr auto ACTUATOR_STALL_TIMEOUT = std::chrono::milliseconds(1500);
constexpr double ACTUATOR_STALL_MOTION_EPSILON = 0.02;

// SPARK MAX Periodic Status 3 contains analogue sensor information.
// The non-FRC CAN reference uses base ID 0x020518C0 + device_id for Status 3.
constexpr uint32_t SPARKMAX_PERIODIC_STATUS_3_BASE_ID = 0x020518C0; // TODO rename to be clearer  (can comms helpers)
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
#pragma endregion Constants

namespace diffdrive_canbus {
  // helpers
  bool string_to_bool(const std::string & value);
  double apply_deadband(double value, double deadband);
  double clean_command(double value, double deadband);
  double clamp_throttle(double value);
  double apply_throttle_deadband(double value, double deadband);

  // can comms helpers
  uint8_t get_frc_device_id_from_can_id(uint32_t can_id);
  uint8_t get_frc_api_index_from_can_id(uint32_t can_id);
  bool is_actuator_status3_id(uint32_t can_id, uint8_t device_id);
  uint16_t le_u16_from_frame_data(const uint8_t data[8], std::size_t offset);
  std::string can_data_to_hex_string(const uint8_t data[8], uint8_t dlc);
  void sleep_bus_gap();

  // actuator
  struct LinearActuatorHW
  {
    rclcpp::Logger logger{rclcpp::get_logger("linear_actuator")};
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

    std::unique_ptr<CANDevice> spark;

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

    // Closed-loop position servo state (bang-bang on analog feedback).
    double position_tolerance{0.03};
    std::chrono::steady_clock::time_point last_feedback_time{};
    bool stall_latched{false};
    double stall_latched_command{0.5};
    bool watchdog_initialised{false};
    double watchdog_ref_position{0.0};
    std::chrono::steady_clock::time_point watchdog_ref_time{};
  };
  void reset_actuator_state(LinearActuatorHW & act, const rclcpp::Logger & logger);
  void send_actuator_duty(LinearActuatorHW & act, double duty, const std::string & label);
  void request_actuator_status3_period(LinearActuatorHW & act, const std::string & label, CANComms & can_);
  void write_actuator_closed_loop(LinearActuatorHW & act, const std::string & label);
  void print_actuator_status(LinearActuatorHW & act, const std::string & label);
  double normalise_actuator_voltage(double voltage, double min_voltage, double max_voltage);
  void observe_actuator_raw_frame(LinearActuatorHW & act, const CANFrame & frame);

  // can system
  class CANSystem {
  public:
    CANSystem(CANComms &comms);
    void add_device(const std::unique_ptr<CANDevice> &device);
    void setup_ros_state_interfaces(std::vector<hardware_interface::StateInterface> &state_interfaces);
    void setup_ros_command_interfaces(std::vector<hardware_interface::CommandInterface> &command_interfaces);

  private:
    std::map<uint8_t, CANDevice*> devices_;
    CANComms &comms_;
  };

  class Motor : public CANDevice {
  public:
    Motor(const std::string &name, const uint8_t &can_id, CANComms &can, float gear_ratio)
      : CANDevice(name, can_id, can, gear_ratio) {}

    void setup_ros_state_interfaces(std::vector<hardware_interface::StateInterface> &state_interfaces) override;
    void setup_ros_command_interfaces(std::vector<hardware_interface::CommandInterface> &command_interfaces) override;

    void update_joint_state_from_telemetry(double &position_offset_rad, bool &position_offset_valid);

    double rotation_position() const override { return rotation_position_; }
    double velocity() const override { return velocity_; }
    double commanded_velocity() const override { return commanded_velocity_; }

  protected:
    double rotation_position_{0.0};
    double velocity_{0.0};
    double commanded_velocity_{0.0};
  };
}
#endif  // DIFFDRIVE_CANBUS__DIFFDRIVE_CANBUS_SYSTEM_HPP_