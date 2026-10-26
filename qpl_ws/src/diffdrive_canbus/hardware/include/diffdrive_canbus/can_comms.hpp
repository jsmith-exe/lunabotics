#ifndef DIFFDRIVE_CANBUS__CAN_COMMS_HPP_
#define DIFFDRIVE_CANBUS__CAN_COMMS_HPP_

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>
#include <linux/can.h>

namespace diffdrive_canbus
{

// can comms helpers
uint8_t get_frc_device_id_from_can_id(uint32_t can_id);
bool is_actuator_status3_id(uint32_t can_id, uint8_t device_id);
uint16_t le_u16_from_frame_data(const uint8_t data[8], std::size_t offset);

struct SparkMaxCanIdFields
{
  uint8_t device_type = 0;
  uint8_t manufacturer = 0;
  uint8_t api_class = 0;
  uint8_t api_index = 0;
  uint8_t device_id = 0;
};

uint32_t make_frc_extended_can_id(
  uint8_t device_type,
  uint8_t manufacturer,
  uint8_t api_class,
  uint8_t api_index,
  uint8_t device_id);

SparkMaxCanIdFields parse_frc_extended_can_id(uint32_t id);

// Device type 2 (motor controller) and manufacturer 5 (REV) occupy bits 28-16 of the ID.
constexpr uint32_t REV_MOTOR_CONTROLLER_ID_PREFIX = 0x02050000;
constexpr uint32_t REV_MOTOR_CONTROLLER_ID_MASK = 0x1FFF0000;

class SocketCanInterface
{
public:
  SocketCanInterface() = default;
  void connect(const std::string & interface_name_);
  void close();
  bool is_open() const { return file_descriptor_ >= 0; }
  ~SocketCanInterface();

  // Non-copyable: this object owns a raw file descriptor.
  SocketCanInterface(const SocketCanInterface &) = delete;
  SocketCanInterface & operator=(const SocketCanInterface &) = delete;

  bool send_frame(uint32_t can_id, const std::vector<uint8_t> & payload, bool is_extended = false);
  bool send_extended_frame(uint32_t can_id, const std::vector<uint8_t> & payload);
  bool read_frame(can_frame &received_frame);
  uint64_t failed_sends() const { return failed_sends_; }
private:
  void log_send_failure(int error_number);

  int file_descriptor_ = -1;
  uint64_t failed_sends_ = 0;
  std::chrono::steady_clock::time_point last_failure_log_time_{};
  std::optional<std::chrono::steady_clock::time_point> failing_since_; // Start of the current run of failed sends.
};
}  // namespace diffdrive_canbus

#endif  // DIFFDRIVE_CANBUS__CAN_COMMS_HPP_