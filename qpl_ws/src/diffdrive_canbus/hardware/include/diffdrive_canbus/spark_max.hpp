#pragma once

#include "diffdrive_canbus/CAN_comms.hpp"

#include <chrono>
#include <cstdint>

namespace diffdrive_canbus
{

class SparkMax
{
public:
  explicit SparkMax(CANComms & can, uint8_t device_id);

  uint8_t device_id() const;

  // Bus-wide heartbeat pair.
  // This should be called regularly, ideally every 10-20 ms.
  bool send_heartbeats(bool print = false);

  // Clears sticky faults on this SPARK MAX.
  bool clear_faults(bool print = false);

  // Open-loop duty cycle command.
  // Range: -1.0 to +1.0
  bool set_duty_cycle(float duty, bool print = false);

  // Closed-loop velocity command in RPM.
  // This depends on PID/config already being correct on the SPARK MAX.
  bool set_velocity_rpm(float rpm, bool print = false);

  // Sends heartbeat + duty in one call.
  // This is the function you probably want to call inside your ROS 2 hardware loop.
  bool update_duty(float duty, bool print = false);

  // Sends heartbeat + zero duty.
  bool stop(bool print = false);

  // Simple helper for testing.
  void run_duty_for(
    float duty,
    std::chrono::milliseconds duration,
    std::chrono::milliseconds period = std::chrono::milliseconds(10));

  // Simple helper for testing.
  void stop_for(
    std::chrono::milliseconds duration,
    std::chrono::milliseconds period = std::chrono::milliseconds(10));

private:
  static constexpr uint8_t DEVICE_TYPE_MOTOR_CONTROLLER = 2;
  static constexpr uint8_t MANUFACTURER_REV = 5;

  static constexpr uint8_t API_CLASS_SETPOINT = 0;
  static constexpr uint8_t API_INDEX_DUTY_CYCLE = 2;

  static constexpr uint8_t API_CLASS_VELOCITY = 1;
  static constexpr uint8_t API_INDEX_VELOCITY = 2;

  static constexpr uint8_t API_CLASS_PERIODIC_STATUS = 6;
  static constexpr uint8_t API_INDEX_CLEAR_FAULTS = 14;

  static constexpr uint8_t API_CLASS_ROBORIO = 9;
  static constexpr uint8_t API_INDEX_ROBORIO_HEARTBEAT = 2;

  static constexpr uint8_t API_CLASS_NON_RIO = 11;
  static constexpr uint8_t API_INDEX_NON_RIO_HEARTBEAT = 2;

  static constexpr uint8_t HEARTBEAT_DEVICE_ID = 0;

  CANComms & can_;
  uint8_t device_id_;

  static uint32_t make_sparkmax_id(
    uint8_t api_class,
    uint8_t api_index,
    uint8_t device_id);

  static std::vector<uint8_t> float_to_le_bytes(float value);

  bool send_setpoint_frame(
    uint8_t api_class,
    uint8_t api_index,
    float setpoint,
    bool print = false);
};

}  // namespace diffdrive_canbus