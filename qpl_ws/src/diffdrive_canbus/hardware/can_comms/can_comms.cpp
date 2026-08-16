/// Handles the lifecycle of the CAN serial communication

#include "diffdrive_canbus/can_comms.hpp"

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <stdexcept>

namespace diffdrive_canbus
{

/// Destructor: ensures the serial port is closed cleanly if the object is
/// destroyed while still connected, e.g. on exception during shutdown.
CANComms::~CANComms()
{
  if (connected())
  {
    disconnect();
  }
}

/// Open and configure the serial port for communication with the Waveshare adapter.
/// Sets fixed parameters required by the adapter: 8N1 (8 data bits, no parity,
/// 1 stop bit), no hardware flow control.
/// Flushes IO buffers after opening to discard any stale bytes from a previous session.
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
      std::cout << "Warning: Serial connection already open. Closing and reopening." << std::endl;
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

/// Send the 20-byte Waveshare config packet to initialise the adapter.
///
/// This must be called once after connect() before any CAN frames are sent.
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
      std::cout << "Sent config: " << bytes_to_hex(packet) << std::endl;
    }

    return true;
  }
  catch (const std::exception & e)
  {
    std::cerr << "Failed to configure CAN adapter: " << e.what() << std::endl;
    return false;
  }
}

/// Read a single byte from the serial port.
/// Returns false on timeout (LibSerial::ReadTimeout) or any other error.
/// timeout_ms_ controls how long to wait per byte — keep this small (1-5ms)
/// to avoid stalling the control loop when the bus is quiet.
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

/// Write a byte vector to the serial port as a raw binary string.
/// LibSerial::Write takes a std::string, so the bytes are reinterpreted
/// via a const char* cast — no encoding or translation occurs.
void CANComms::write_bytes(const std::vector<uint8_t> & bytes)
{
  const std::string raw(
    reinterpret_cast<const char *>(bytes.data()),
    bytes.size());

  serial_conn_.Write(raw);
}

/// Encode and transmit a CANFrame over the serial link.
/// The frame is validated (DLC <= 8) before encoding to avoid producing
/// malformed Waveshare packets.
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

/// Convenience wrapper: build and send an extended (29-bit) CAN frame from
/// a raw CAN ID and a data vector, without constructing a CANFrame manually.
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
  // Zero the full 8-byte data array so unused bytes beyond dlc are clean.
  std::fill(std::begin(frame.data), std::end(frame.data), 0);

  for (size_t i = 0; i < data.size(); ++i)
  {
    frame.data[i] = data[i];
  }

  return send_frame(frame, print_output);
}

/// Read one CAN frame from the adapter.
/// Internally calls read_variable_packet() which blocks byte-by-byte until a
/// complete Waveshare frame is received or the per-byte timeout expires.
/// Returns false on timeout, malformed packet, or decode failure.
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

std::vector<CANFrame> CANComms::poll_frames()
{
  serial_conn_.Read(buffer_, buffer_.size(), timeout_ms_);

  std::vector<CANFrame> frames = extract_frame();

  return frames;
}

/// Compute an 8-bit checksum over a slice of a byte vector.
/// Sums bytes[start_idx] through bytes[end_idx_inclusive] and returns the
/// low 8 bits of the result. Used for the Waveshare config packet checksum.
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

/// Format a byte vector as space-separated uppercase hex pairs.
/// Example: {0xAA, 0xE8, 0x41} -> "AA E8 41"
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