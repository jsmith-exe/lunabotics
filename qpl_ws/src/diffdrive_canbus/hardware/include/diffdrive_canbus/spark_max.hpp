#pragma once

#include "diffdrive_canbus/CAN_comms.hpp"
#include "diffdrive_canbus/Velocity_Controller.hpp"

#include <cstddef>
#include <cstdint>

namespace diffdrive_canbus
{

struct SparkMaxTelemetry
{
  bool has_encoder_velocity = false;
  float encoder_velocity_rpm = 0.0f;
  float motor_rad_per_sec = 0.0f;
  float wheel_rad_per_sec = 0.0f;

  bool has_encoder_position = false;
  float encoder_position_rotations = 0.0f;
  float wheel_position_rotations = 0.0f;

  bool has_applied_output = false;
  float applied_output = 0.0f;
};

class SparkMax
{
public:
  enum class VelocityCommandMode
  {
    SoftwareDutyPid,
    NativeSparkMaxPid
  };

  SparkMax(CANComms & can, uint8_t device_id, float gear_ratio = 1.0f);

  uint8_t device_id() const;
  float gear_ratio() const;
  void set_gear_ratio(float gear_ratio);

  bool send_heartbeats(bool print = false);
  bool clear_faults(bool print = false);

  bool set_duty_cycle(float duty, bool print = false);
  bool stop(bool print = false);

  bool read_telemetry(int max_frames = 20, bool print_status_frames = false);

  const SparkMaxTelemetry & telemetry() const;

  float encoder_velocity_rpm() const;
  float motor_rad_per_sec() const;
  float wheel_rad_per_sec() const;
  float encoder_position_rotations() const;
  float wheel_position_rotations() const;
  float applied_output() const;

  // Configure the outer software velocity PID controller.
  // This is only used in VelocityCommandMode::SoftwareDutyPid.
  void configure_velocity_pid(
    float Kp,
    float Ki,
    float Kd,
    float integral_reset,
    float d_tau);

  // Select how set_velocity_* commands are sent.
  //
  // SoftwareDutyPid:
  //   Local software PID calculates a duty-cycle output from encoder feedback.
  //
  // NativeSparkMaxPid:
  //   Sends a native velocity candidate frame directly to the SPARK MAX.
  //   This is still experimental on the raw CAN path.
  void set_velocity_command_mode(VelocityCommandMode mode);
  VelocityCommandMode velocity_command_mode() const;

  // SPARK MAX internal PID slot for native velocity commands.
  void set_native_velocity_pid_slot(uint8_t pid_slot);

  // Direct native velocity commands.
  // These bypass the local software PID.
  bool set_native_velocity_rpm(
    float target_motor_rpm,
    bool print = false);

  bool set_native_velocity_rad_per_sec(
    float target_wheel_rad_per_sec,
    bool print = false);

  // Debug only:
  // Send a raw setpoint-style frame to any API class/index.
  // Useful for sweeping possible REV setpoint endpoints.
  bool debug_send_setpoint_api(
    uint8_t api_class,
    uint8_t api_index,
    float setpoint,
    uint8_t pid_slot = 0,
    bool print = false);

  // Command wheel angular velocity in rad/s.
  // This accounts for gear_ratio_ internally.
  //
  // Behaviour depends on velocity_command_mode_:
  //   SoftwareDutyPid   -> local PID generates duty cycle
  //   NativeSparkMaxPid -> native velocity reference sent to SPARK MAX
  bool set_velocity_rad_per_sec(
    float target_wheel_rad_per_sec,
    bool print = false);

  // Command motor shaft speed in RPM.
  //
  // Behaviour depends on velocity_command_mode_:
  //   SoftwareDutyPid   -> converted to wheel rad/s, then local PID-to-duty
  //   NativeSparkMaxPid -> native motor RPM reference sent to SPARK MAX
  bool set_velocity_rpm(
    float target_motor_rpm,
    bool print = false);

  // Update only the software velocity controller output using the most recent
  // telemetry already stored in telemetry_.
  //
  // Important:
  //   This does NOT send CAN commands.
  //
  // Intended split-loop usage:
  //
  //   RX/read side:
  //     spark.read_telemetry(...)
  //
  //   TX/control side:
  //     spark.update_velocity_controller_only(target_wheel_rad_per_sec);
  //     spark.send_velocity_controller_output();
  //
  // This keeps telemetry reads from creating long gaps between duty commands,
  // which is likely what makes the SPARK MAX drop into coast.
  void update_velocity_controller_only(float target_wheel_rad_per_sec);

  // Sends the current software velocity controller output as a duty-cycle command.
  //
  // This is useful when the controller update and CAN transmit are separated.
  bool send_velocity_controller_output(bool print = false);

  void reset_velocity_controller();

  // Current software velocity controller duty output.
  // In SoftwareDutyPid mode, this is the commanded duty state.
  float velocity_controller_output() const;

private:
  static constexpr float PI = 3.14159265358979323846f;

  // Assumed free speed used only for rough conversions / sanity checks.
  static constexpr float MAX_MOTOR_RPM = 5800.0f;

  static constexpr uint8_t DEVICE_TYPE_MOTOR_CONTROLLER = 2;
  static constexpr uint8_t MANUFACTURER_REV = 5;

  // Setpoint command API IDs.
  //
  // FRC/REV API ID is effectively:
  //   api_id = (api_class << 4) | api_index
  //
  // Confirmed duty-cycle setpoint:
  //   api_id    = 0x002
  //   api_class = 0
  //   api_index = 2
  //
  // Reference setpoint candidate:
  //   api_id    = 0x004
  //   api_class = 0
  //   api_index = 4
  //
  // Testing strongly suggested this behaves like position/reference control,
  // not reliable velocity control, so the rover should use SoftwareDutyPid for now.
  static constexpr uint8_t API_CLASS_DUTY_CYCLE_SETPOINT = 0;
  static constexpr uint8_t API_INDEX_DUTY_CYCLE = 2;

  static constexpr uint8_t API_CLASS_REFERENCE_SETPOINT = 0;
  static constexpr uint8_t API_INDEX_REFERENCE_SETPOINT = 4;

  static constexpr uint8_t CONTROL_TYPE_POSITION_CANDIDATE = 0;
  static constexpr uint8_t CONTROL_TYPE_VELOCITY_CANDIDATE = 1;

  // Native velocity candidate:
  //
  // api_id    = 0x012
  // api_class = 1
  // api_index = 2
  //
  // This produced purple / no applied output in your tests, so it is retained
  // only for future raw-CAN testing.
  static constexpr uint8_t API_CLASS_NATIVE_VELOCITY_SETPOINT = 1;
  static constexpr uint8_t API_INDEX_NATIVE_VELOCITY = 2;

  // Confirmed from live bus frames.
  static constexpr uint8_t API_CLASS_PERIODIC_STATUS = 46;
  static constexpr uint8_t API_INDEX_STATUS_0 = 0;
  static constexpr uint8_t API_INDEX_STATUS_1 = 1;
  static constexpr uint8_t API_INDEX_STATUS_2 = 2;
  static constexpr uint8_t API_INDEX_STATUS_5 = 5;

  static constexpr uint8_t API_CLASS_CLEAR_FAULTS = 6;
  static constexpr uint8_t API_INDEX_CLEAR_FAULTS = 14;

  static constexpr uint8_t API_CLASS_ROBORIO = 9;
  static constexpr uint8_t API_INDEX_ROBORIO_HEARTBEAT = 2;

  static constexpr uint8_t API_CLASS_NON_RIO = 11;
  static constexpr uint8_t API_INDEX_NON_RIO_HEARTBEAT = 2;

  static constexpr uint8_t HEARTBEAT_DEVICE_ID = 0;

  CANComms & can_;
  uint8_t device_id_;
  float gear_ratio_;

  SparkMaxTelemetry telemetry_;

  PIDVelocity velocity_controller_;

  VelocityCommandMode velocity_command_mode_ =
    VelocityCommandMode::SoftwareDutyPid;

  uint8_t native_velocity_pid_slot_ = 0;

  static uint32_t make_sparkmax_id(
    uint8_t api_class,
    uint8_t api_index,
    uint8_t device_id);

  static float rpm_to_rad_per_sec(float rpm);
  static float rad_per_sec_to_rpm(float rad_per_sec);
  static float clamp_duty(float duty);

  static void float_to_le_bytes(float value, uint8_t bytes[4]);
  static float le_bytes_to_float(const uint8_t data[8], size_t offset);
  static int16_t le_bytes_to_i16(const uint8_t data[8], size_t offset);

  bool send_setpoint(
    uint8_t api_class,
    uint8_t api_index,
    float setpoint,
    uint8_t pid_slot,
    bool print);

  bool send_setpoint_with_control_type(
    uint8_t api_class,
    uint8_t api_index,
    float setpoint,
    uint8_t control_type,
    uint8_t pid_slot,
    bool print);

  bool parse_status_frame(const CANFrame & frame, bool print_status_frame);
};

}  // namespace diffdrive_canbus