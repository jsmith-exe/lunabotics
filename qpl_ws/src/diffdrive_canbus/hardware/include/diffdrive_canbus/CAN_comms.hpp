#ifndef DIFFDRIVE_CANBUS__CAN_COMMS_HPP_
#define DIFFDRIVE_CANBUS__CAN_COMMS_HPP_

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include <libserial/SerialPort.h>

namespace diffdrive_canbus
{

struct CANFrame
{
  uint32_t id = 0;
  uint8_t dlc = 0;
  uint8_t data[8] = {0};
  bool extended = true;
  bool remote = false;
};

enum class CANMode : uint8_t
{
  NORMAL = 0x00,
  SILENT = 0x01,
  LOOPBACK = 0x02,
  LOOPBACK_SILENT = 0x03
};

LibSerial::BaudRate convert_baud_rate(int baud_rate);
uint8_t can_baud_to_waveshare_code(int32_t can_baud_rate);

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

class CANComms
{
public:
  CANComms() = default;
  ~CANComms();

  void connect(
    const std::string & device,
    int32_t serial_baud_rate = 2000000,
    int32_t timeout_ms = 100);

  void disconnect();
  bool connected() const;

  bool configure_adapter(
    int32_t can_baud_rate,
    bool use_extended_filter = true,
    uint32_t filter_id = 0x00000000,
    uint32_t block_id = 0x00000000,
    CANMode mode = CANMode::NORMAL,
    bool disable_auto_retransmit = false,
    bool print_output = false);

  bool send_frame(const CANFrame & frame, bool print_output = false);

  bool send_extended_frame(
    uint32_t can_id,
    const std::vector<uint8_t> & data,
    bool print_output = false);

  bool read_frame(CANFrame & frame, bool print_output = false);

  // Compatibility wrappers for older code paths
  bool send_can_frame(const CANFrame & frame, bool print_output = false);
  bool read_can_frame(CANFrame & frame, bool print_output = false);
  bool read_can_frame_for_id(
    uint32_t expected_id,
    CANFrame & frame,
    bool print_output = false,
    int max_attempts = 100);

  // Temporary legacy helper so diffbot_system.cpp still builds.
  bool write_motor_command(int can_id, int command, bool print_output = false);

  // Temporary legacy helper so diffbot_system.cpp still builds.
  bool read_motor_encoder(int can_id, int & encoder_count, bool print_output = false);

  bool read_frame_for_id(
    uint32_t expected_id,
    CANFrame & frame,
    bool print_output = false,
    int max_attempts = 100);

  bool read_frame_matching(
    const std::function<bool(const CANFrame &)> & matcher,
    CANFrame & frame,
    bool print_output = false,
    int max_attempts = 100);

  std::vector<CANFrame> drain_frames(
    std::size_t max_frames = 256,
    bool print_output = false);

  static std::string frame_to_string(const CANFrame & frame);

private:
  std::vector<uint8_t> build_config_packet(
    int32_t can_baud_rate,
    bool use_extended_filter,
    uint32_t filter_id,
    uint32_t block_id,
    CANMode mode,
    bool disable_auto_retransmit) const;

  std::vector<uint8_t> encode_frame(const CANFrame & frame) const;
  bool decode_frame(const std::vector<uint8_t> & packet, CANFrame & frame) const;
  bool read_variable_packet(std::vector<uint8_t> & packet);
  bool read_byte(uint8_t & out);
  void write_bytes(const std::vector<uint8_t> & bytes);

  static uint8_t checksum_low8(
    const std::vector<uint8_t> & bytes,
    std::size_t start_idx,
    std::size_t end_idx_inclusive);

  static std::string bytes_to_hex(const std::vector<uint8_t> & bytes);

  LibSerial::SerialPort serial_conn_;
  int timeout_ms_ = 100;
};

}  // namespace diffdrive_canbus

#endif  // DIFFDRIVE_CANBUS__CAN_COMMS_HPP_