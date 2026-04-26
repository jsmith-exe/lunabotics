#include "diffdrive_canbus/CAN_comms.hpp"

#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace diffdrive_canbus
{

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  switch (baud_rate)
  {
    case 9600:    return LibSerial::BaudRate::BAUD_9600;
    case 19200:   return LibSerial::BaudRate::BAUD_19200;
    case 38400:   return LibSerial::BaudRate::BAUD_38400;
    case 115200:  return LibSerial::BaudRate::BAUD_115200;
    case 230400:  return LibSerial::BaudRate::BAUD_230400;
    case 460800:  return LibSerial::BaudRate::BAUD_460800;
    case 921600:  return LibSerial::BaudRate::BAUD_921600;
    case 2000000: return LibSerial::BaudRate::BAUD_2000000;
    default:
      throw std::runtime_error(
        "Unsupported serial baud rate: " + std::to_string(baud_rate));
  }
}

uint8_t can_baud_to_waveshare_code(int32_t can_baud_rate)
{
  switch (can_baud_rate)
  {
    case 1000000: return 0x01;
    case 800000:  return 0x02;
    case 500000:  return 0x03;
    case 400000:  return 0x04;
    case 250000:  return 0x05;
    case 200000:  return 0x06;
    case 125000:  return 0x07;
    case 100000:  return 0x08;
    case 50000:   return 0x09;
    case 20000:   return 0x0A;
    case 10000:   return 0x0B;
    case 5000:    return 0x0C;
    default:
      throw std::runtime_error(
        "Unsupported CAN baud rate for Waveshare USB-CAN-A: " +
        std::to_string(can_baud_rate));
  }
}

uint32_t make_frc_extended_can_id(
  uint8_t device_type,
  uint8_t manufacturer,
  uint8_t api_class,
  uint8_t api_index,
  uint8_t device_id)
{
  return
    ((static_cast<uint32_t>(device_type)  & 0x1F) << 24) |
    ((static_cast<uint32_t>(manufacturer) & 0xFF) << 16) |
    ((static_cast<uint32_t>(api_class)    & 0x3F) << 10) |
    ((static_cast<uint32_t>(api_index)    & 0x0F) <<  6) |
    ((static_cast<uint32_t>(device_id)    & 0x3F) <<  0);
}

SparkMaxCanIdFields parse_frc_extended_can_id(uint32_t id)
{
  SparkMaxCanIdFields fields;
  fields.device_type  = static_cast<uint8_t>((id >> 24) & 0x1F);
  fields.manufacturer = static_cast<uint8_t>((id >> 16) & 0xFF);
  fields.api_class    = static_cast<uint8_t>((id >> 10) & 0x3F);
  fields.api_index    = static_cast<uint8_t>((id >>  6) & 0x0F);
  fields.device_id    = static_cast<uint8_t>((id >>  0) & 0x3F);
  return fields;
}

CANComms::~CANComms()
{
  if (connected())
  {
    disconnect();
  }
}

void CANComms::connect(
  const std::string & device,
  int32_t serial_baud_rate,
  int32_t timeout_ms)
{
  timeout_ms_ = timeout_ms;

  if (!std::filesystem::exists(device))
  {
    throw std::runtime_error("Serial device does not exist: " + device);
  }

  try
  {
    if (serial_conn_.IsOpen())
    {
      serial_conn_.Close();
    }

    serial_conn_.Open(device);
    serial_conn_.SetBaudRate(convert_baud_rate(serial_baud_rate));
    serial_conn_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_conn_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    serial_conn_.SetParity(LibSerial::Parity::PARITY_NONE);
    serial_conn_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    serial_conn_.FlushIOBuffers();
  }
  catch (const std::exception & e)
  {
    throw std::runtime_error(
      "Failed to open/configure serial device '" + device + "': " + e.what());
  }
}

void CANComms::disconnect()
{
  if (serial_conn_.IsOpen())
  {
    serial_conn_.Close();
  }
}

bool CANComms::connected() const
{
  return serial_conn_.IsOpen();
}

bool CANComms::configure_adapter(
  int32_t can_baud_rate,
  bool use_extended_filter,
  uint32_t filter_id,
  uint32_t block_id,
  CANMode mode,
  bool disable_auto_retransmit,
  bool print_output)
{
  if (!connected())
  {
    std::cerr << "CAN adapter not connected." << std::endl;
    return false;
  }

  const auto packet = build_config_packet(
    can_baud_rate,
    use_extended_filter,
    filter_id,
    block_id,
    mode,
    disable_auto_retransmit);

  try
  {
    write_bytes(packet);

    if (print_output)
    {
      std::cerr << "Sent config: " << bytes_to_hex(packet) << std::endl;
    }

    return true;
  }
  catch (const std::exception & e)
  {
    std::cerr << "Failed to configure CAN adapter: " << e.what() << std::endl;
    return false;
  }
}

bool CANComms::send_frame(const CANFrame & frame, bool print_output)
{
  if (!connected())
  {
    std::cerr << "CAN adapter not connected." << std::endl;
    return false;
  }

  if (frame.dlc > 8)
  {
    std::cerr << "Invalid DLC " << static_cast<int>(frame.dlc) << std::endl;
    return false;
  }

  try
  {
    const auto packet = encode_frame(frame);
    write_bytes(packet);

    if (print_output)
    {
      std::cerr << "Sent frame: " << frame_to_string(frame)
                << " | raw=" << bytes_to_hex(packet) << std::endl;
    }

    return true;
  }
  catch (const std::exception & e)
  {
    std::cerr << "Failed sending CAN frame: " << e.what() << std::endl;
    return false;
  }
}

bool CANComms::send_extended_frame(
  uint32_t can_id,
  const std::vector<uint8_t> & data,
  bool print_output)
{
  if (data.size() > 8)
  {
    std::cerr << "Invalid DLC " << data.size() << std::endl;
    return false;
  }

  CANFrame frame;
  frame.id = can_id;
  frame.dlc = static_cast<uint8_t>(data.size());
  frame.extended = true;
  frame.remote = false;
  std::fill(std::begin(frame.data), std::end(frame.data), 0);

  for (size_t i = 0; i < data.size(); ++i)
  {
    frame.data[i] = data[i];
  }

  return send_frame(frame, print_output);
}

bool CANComms::read_frame(CANFrame & frame, bool print_output)
{
  if (!connected())
  {
    std::cerr << "CAN adapter not connected." << std::endl;
    return false;
  }

  try
  {
    std::vector<uint8_t> packet;
    if (!read_variable_packet(packet))
    {
      return false;
    }

    if (!decode_frame(packet, frame))
    {
      return false;
    }

    if (print_output)
    {
      std::cerr << "Recv frame: " << frame_to_string(frame)
                << " | raw=" << bytes_to_hex(packet) << std::endl;
    }

    return true;
  }
  catch (const std::exception & e)
  {
    std::cerr << "Failed reading CAN frame from adapter: " << e.what() << std::endl;
    return false;
  }
}

bool CANComms::send_can_frame(const CANFrame & frame, bool print_output)
{
  return send_frame(frame, print_output);
}

bool CANComms::read_can_frame(CANFrame & frame, bool print_output)
{
  return read_frame(frame, print_output);
}

bool CANComms::read_can_frame_for_id(
  uint32_t expected_id,
  CANFrame & frame,
  bool print_output,
  int max_attempts)
{
  return read_frame_for_id(expected_id, frame, print_output, max_attempts);
}

bool CANComms::write_motor_command(int can_id, int command, bool print_output)
{
  CANFrame frame;
  frame.id = static_cast<uint32_t>(can_id);
  frame.dlc = 4;
  frame.extended = true;
  frame.remote = false;

  std::fill(std::begin(frame.data), std::end(frame.data), 0);
  frame.data[0] = static_cast<uint8_t>(command & 0xFF);
  frame.data[1] = static_cast<uint8_t>((command >> 8) & 0xFF);
  frame.data[2] = static_cast<uint8_t>((command >> 16) & 0xFF);
  frame.data[3] = static_cast<uint8_t>((command >> 24) & 0xFF);

  return send_frame(frame, print_output);
}

bool CANComms::read_motor_encoder(int can_id, int & encoder_count, bool print_output)
{
  CANFrame response;
  if (!read_frame_for_id(static_cast<uint32_t>(can_id), response, print_output))
  {
    return false;
  }

  if (response.dlc < 4)
  {
    std::cerr << "Encoder response too short for CAN ID " << can_id << std::endl;
    return false;
  }

  encoder_count =
    static_cast<int>(response.data[0]) |
    (static_cast<int>(response.data[1]) << 8) |
    (static_cast<int>(response.data[2]) << 16) |
    (static_cast<int>(response.data[3]) << 24);

  return true;
}

bool CANComms::read_frame_for_id(
  uint32_t expected_id,
  CANFrame & frame,
  bool print_output,
  int max_attempts)
{
  return read_frame_matching(
    [&](const CANFrame & f) { return f.id == expected_id; },
    frame,
    print_output,
    max_attempts);
}

bool CANComms::read_frame_matching(
  const std::function<bool(const CANFrame &)> & matcher,
  CANFrame & frame,
  bool print_output,
  int max_attempts)
{
  for (int i = 0; i < max_attempts; ++i)
  {
    CANFrame temp;
    if (!read_frame(temp, print_output))
    {
      continue;
    }

    if (matcher(temp))
    {
      frame = temp;
      return true;
    }
  }

  return false;
}

std::vector<CANFrame> CANComms::drain_frames(
  std::size_t max_frames,
  bool print_output)
{
  std::vector<CANFrame> frames;
  frames.reserve(max_frames);

  for (std::size_t i = 0; i < max_frames; ++i)
  {
    CANFrame frame;
    if (!read_frame(frame, print_output))
    {
      break;
    }
    frames.push_back(frame);
  }

  return frames;
}

std::string CANComms::frame_to_string(const CANFrame & frame)
{
  std::ostringstream ss;
  ss << (frame.extended ? "EXT" : "STD")
     << " ID=0x" << std::hex << std::uppercase << frame.id
     << " DLC=" << std::dec << static_cast<int>(frame.dlc)
     << " DATA=[";
  for (uint8_t i = 0; i < frame.dlc; ++i)
  {
    ss << "0x" << std::hex << std::uppercase
       << std::setw(2) << std::setfill('0')
       << static_cast<int>(frame.data[i]);
    if (i + 1 < frame.dlc)
    {
      ss << ' ';
    }
  }
  ss << "]";
  if (frame.remote)
  {
    ss << " RTR";
  }
  return ss.str();
}

std::vector<uint8_t> CANComms::build_config_packet(
  int32_t can_baud_rate,
  bool use_extended_filter,
  uint32_t filter_id,
  uint32_t block_id,
  CANMode mode,
  bool disable_auto_retransmit) const
{
  std::vector<uint8_t> packet(20, 0x00);
  packet[0] = 0xAA;
  packet[1] = 0x55;
  packet[2] = 0x12;
  packet[3] = can_baud_to_waveshare_code(can_baud_rate);
  packet[4] = use_extended_filter ? 0x02 : 0x01;

  packet[5]  = static_cast<uint8_t>(filter_id & 0xFF);
  packet[6]  = static_cast<uint8_t>((filter_id >> 8) & 0xFF);
  packet[7]  = static_cast<uint8_t>((filter_id >> 16) & 0xFF);
  packet[8]  = static_cast<uint8_t>((filter_id >> 24) & 0xFF);

  packet[9]  = static_cast<uint8_t>(block_id & 0xFF);
  packet[10] = static_cast<uint8_t>((block_id >> 8) & 0xFF);
  packet[11] = static_cast<uint8_t>((block_id >> 16) & 0xFF);
  packet[12] = static_cast<uint8_t>((block_id >> 24) & 0xFF);

  packet[13] = static_cast<uint8_t>(mode);
  packet[14] = disable_auto_retransmit ? 0x01 : 0x00;
  packet[15] = 0x00;
  packet[16] = 0x00;
  packet[17] = 0x00;
  packet[18] = 0x00;
  packet[19] = checksum_low8(packet, 2, 18);

  return packet;
}

std::vector<uint8_t> CANComms::encode_frame(const CANFrame & frame) const
{
  std::vector<uint8_t> packet;
  packet.reserve(1 + 1 + (frame.extended ? 4 : 2) + frame.dlc + 1);

  packet.push_back(0xAA);

  uint8_t type = 0xC0;
  if (frame.extended) { type |= (1u << 5); }
  if (frame.remote)   { type |= (1u << 4); }
  type |= static_cast<uint8_t>(frame.dlc & 0x0F);
  packet.push_back(type);

  if (frame.extended)
  {
    packet.push_back(static_cast<uint8_t>( frame.id        & 0xFF));
    packet.push_back(static_cast<uint8_t>((frame.id >> 8)  & 0xFF));
    packet.push_back(static_cast<uint8_t>((frame.id >> 16) & 0xFF));
    packet.push_back(static_cast<uint8_t>((frame.id >> 24) & 0x1F));
  }
  else
  {
    if (frame.id > 0x7FF)
    {
      throw std::runtime_error("Standard CAN ID out of range: " + std::to_string(frame.id));
    }

    packet.push_back(static_cast<uint8_t>( frame.id       & 0xFF));
    packet.push_back(static_cast<uint8_t>((frame.id >> 8) & 0x07));
  }

  if (!frame.remote)
  {
    for (uint8_t i = 0; i < frame.dlc; ++i)
    {
      packet.push_back(frame.data[i]);
    }
  }

  packet.push_back(0x55);
  return packet;
}

bool CANComms::decode_frame(const std::vector<uint8_t> & packet, CANFrame & frame) const
{
  if (packet.size() < 4)
  {
    return false;
  }

  if (packet.front() != 0xAA || packet.back() != 0x55)
  {
    return false;
  }

  const uint8_t type = packet[1];
  if ((type & 0xC0) != 0xC0)
  {
    return false;
  }

  frame.extended = ((type >> 5) & 0x01) != 0;
  frame.remote   = ((type >> 4) & 0x01) != 0;
  frame.dlc      = static_cast<uint8_t>(type & 0x0F);

  if (frame.dlc > 8)
  {
    return false;
  }

  const std::size_t id_len = frame.extended ? 4 : 2;
  const std::size_t expected_size =
    1 + 1 + id_len + (frame.remote ? 0 : frame.dlc) + 1;

  if (packet.size() != expected_size)
  {
    return false;
  }

  if (frame.extended)
  {
    frame.id =
      (static_cast<uint32_t>(packet[2]) << 0) |
      (static_cast<uint32_t>(packet[3]) << 8) |
      (static_cast<uint32_t>(packet[4]) << 16) |
      ((static_cast<uint32_t>(packet[5]) & 0x1F) << 24);
  }
  else
  {
    frame.id =
      static_cast<uint32_t>(packet[2]) |
      (static_cast<uint32_t>(packet[3] & 0x07) << 8);
  }

  std::fill(std::begin(frame.data), std::end(frame.data), 0);

  if (!frame.remote)
  {
    const std::size_t data_offset = 2 + id_len;
    for (uint8_t i = 0; i < frame.dlc; ++i)
    {
      frame.data[i] = packet[data_offset + i];
    }
  }

  return true;
}

bool CANComms::read_variable_packet(std::vector<uint8_t> & packet)
{
  packet.clear();

  uint8_t byte = 0;

  while (true)
  {
    if (!read_byte(byte))
    {
      return false;
    }

    if (byte == 0xAA)
    {
      packet.push_back(byte);
      break;
    }
  }

  uint8_t type = 0;
  if (!read_byte(type))
  {
    return false;
  }
  packet.push_back(type);

  if ((type & 0xC0) != 0xC0)
  {
    return false;
  }

  const bool extended = ((type >> 5) & 0x01) != 0;
  const bool remote   = ((type >> 4) & 0x01) != 0;
  const uint8_t dlc   = static_cast<uint8_t>(type & 0x0F);

  if (dlc > 8)
  {
    return false;
  }

  const std::size_t id_len = extended ? 4 : 2;
  const std::size_t data_len = remote ? 0 : dlc;
  const std::size_t remaining = id_len + data_len + 1;

  for (std::size_t i = 0; i < remaining; ++i)
  {
    if (!read_byte(byte))
    {
      return false;
    }
    packet.push_back(byte);
  }

  if (packet.back() != 0x55)
  {
    return false;
  }

  return true;
}

bool CANComms::read_byte(uint8_t & out)
{
  try
  {
    char c = 0;
    serial_conn_.ReadByte(c, timeout_ms_);
    out = static_cast<uint8_t>(static_cast<unsigned char>(c));
    return true;
  }
  catch (const LibSerial::ReadTimeout &)
  {
    return false;
  }
  catch (const std::exception & e)
  {
    std::cerr << "Serial read error: " << e.what() << std::endl;
    return false;
  }
}

void CANComms::write_bytes(const std::vector<uint8_t> & bytes)
{
  const std::string raw(
    reinterpret_cast<const char *>(bytes.data()),
    bytes.size());
  serial_conn_.Write(raw);
}

uint8_t CANComms::checksum_low8(
  const std::vector<uint8_t> & bytes,
  std::size_t start_idx,
  std::size_t end_idx_inclusive)
{
  uint32_t sum = 0;
  for (std::size_t i = start_idx; i <= end_idx_inclusive; ++i)
  {
    sum += bytes[i];
  }
  return static_cast<uint8_t>(sum & 0xFF);
}

std::string CANComms::bytes_to_hex(const std::vector<uint8_t> & bytes)
{
  std::ostringstream ss;
  ss << std::hex << std::uppercase << std::setfill('0');

  for (std::size_t i = 0; i < bytes.size(); ++i)
  {
    ss << std::setw(2) << static_cast<int>(bytes[i]);
    if (i + 1 < bytes.size())
    {
      ss << ' ';
    }
  }

  return ss.str();
}

}  // namespace diffdrive_canbus