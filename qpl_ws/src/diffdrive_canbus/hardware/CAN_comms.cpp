#include "diffdrive_canbus/CAN_comms.hpp"

#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace diffdrive_canbus
{

// ---------------------------------------------------------------------------
// Baud rate conversion helpers
// ---------------------------------------------------------------------------

/// Convert an integer baud rate to the LibSerial enum value.
/// LibSerial requires an enum rather than a raw integer, so this maps the
/// common rates supported by the Waveshare adapter.
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
} // SERIAL

/// Convert a CAN bus baud rate (Hz) to the single-byte code the Waveshare
/// USB-CAN-A config packet expects at byte [3].
/// These codes are specific to this adapter model and are not standard CAN.
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
} // WAVESHARE

// ---------------------------------------------------------------------------
// FRC CAN ID layout
// ---------------------------------------------------------------------------
//
// SPARK MAX uses 29-bit extended CAN IDs following the FRC CAN specification.
// The 29 bits are packed as:
//
//   bits 28-24  device type  (5 bits)  — 0x02 for motor controllers
//   bits 23-16  manufacturer (8 bits)  — 0x05 for REV Robotics
//   bits 15-10  api class    (6 bits)  — command/status category
//   bits  9- 6  api index    (4 bits)  — specific command within the class
//   bits  5- 0  device id    (6 bits)  — which physical controller (1-63)
//
// ---------------------------------------------------------------------------

/// Pack the five FRC CAN ID fields into a single 29-bit value.
/// Each field is masked to its declared width before shifting to prevent
/// out-of-range values corrupting adjacent fields.
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
} // CAN

/// Unpack a 29-bit FRC CAN ID into its five named fields.
/// Used when parsing incoming status frames to identify which device and
/// message type a frame belongs to.
SparkMaxCanIdFields parse_frc_extended_can_id(uint32_t id)
{
  SparkMaxCanIdFields fields;
  fields.device_type  = static_cast<uint8_t>((id >> 24) & 0x1F);
  fields.manufacturer = static_cast<uint8_t>((id >> 16) & 0xFF);
  fields.api_class    = static_cast<uint8_t>((id >> 10) & 0x3F);
  fields.api_index    = static_cast<uint8_t>((id >>  6) & 0x0F);
  fields.device_id    = static_cast<uint8_t>((id >>  0) & 0x3F);
  return fields;
} // CAN

// ---------------------------------------------------------------------------
// CANComms lifecycle
// ---------------------------------------------------------------------------

/// Destructor: ensures the serial port is closed cleanly if the object is
/// destroyed while still connected, e.g. on exception during shutdown.
CANComms::~CANComms()
{
  if (connected())
  {
    disconnect();
  }
} // SERIAL/CAN

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
} // CAN

void CANComms::disconnect()
{
  if (serial_conn_.IsOpen())
  {
    serial_conn_.Close();
  }
} // CAN

bool CANComms::connected() const
{
  return serial_conn_.IsOpen();
} // CAN

// ---------------------------------------------------------------------------
// Adapter configuration
// ---------------------------------------------------------------------------

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
} // CAN

// ---------------------------------------------------------------------------
// Frame send / receive
// ---------------------------------------------------------------------------

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
} // CAN

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
} // CAN

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
} // CAN

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------

/// Format a CANFrame as a human-readable string for logging.
/// Example output: EXT ID=0x02051841 DLC=8 DATA=[0x39 0x8A 0xF8 0x43 ...]
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

// ---------------------------------------------------------------------------
// Waveshare packet codec (private)
// ---------------------------------------------------------------------------
//
// Waveshare USB-CAN-A binary protocol:
//
//   Config packet (20 bytes, host -> adapter, sent once on startup):
//     [0] 0xAA  — packet start
//     [1] 0x55  — identifies this as a config packet (not a CAN frame)
//     [2] 0x12  — config command byte
//     [3] baud code (see can_baud_to_waveshare_code)
//     [4] filter type: 0x01 = standard, 0x02 = extended
//     [5..8]  filter ID, 4 bytes little-endian
//     [9..12] block ID,  4 bytes little-endian
//     [13] mode: 0x00 = normal, 0x01 = loopback
//     [14] disable auto-retransmit: 0x01 = yes, 0x00 = no
//     [15..18] reserved, zeroed
//     [19] checksum: low 8 bits of sum of bytes [2..18]
//
//   CAN data frame (variable length, bidirectional):
//     [0]    0xAA — start-of-frame marker
//     [1]    type byte:
//              bits 7-6  = 0b11 (0xC0) — frame marker
//              bit  5    = 1 if extended 29-bit ID, 0 if standard 11-bit
//              bit  4    = 1 if RTR (remote frame), 0 if data frame
//              bits 3-0  = DLC (0-8)
//     [2..5] CAN ID, 4 bytes little-endian (extended) — top 3 bits unused
//     [2..3] CAN ID, 2 bytes little-endian (standard)
//     [...] DLC bytes of CAN payload (omitted for RTR frames)
//     [last] 0x55 — end-of-frame marker
//
// ---------------------------------------------------------------------------

/// Build the 20-byte Waveshare config packet.
/// The checksum covers bytes [2..18] (inclusive) as a simple sum, truncated
/// to 8 bits — it does not include the 0xAA/0x55 framing bytes.
std::vector<uint8_t> CANComms::build_config_packet(
  int32_t can_baud_rate,
  bool use_extended_filter,
  uint32_t filter_id,
  uint32_t block_id,
  CANMode mode,
  bool disable_auto_retransmit) const
{
  std::vector<uint8_t> packet(20, 0x00);
  packet[0] = 0xAA;                                          // packet start
  packet[1] = 0x55;                                          // config packet marker
  packet[2] = 0x12;                                          // config command
  packet[3] = can_baud_to_waveshare_code(can_baud_rate);     // CAN baud rate code
  packet[4] = use_extended_filter ? 0x02 : 0x01;             // filter type

  // Filter ID: frames matching this ID are passed through (0 = accept all)
  packet[5]  = static_cast<uint8_t>(filter_id & 0xFF);
  packet[6]  = static_cast<uint8_t>((filter_id >> 8) & 0xFF);
  packet[7]  = static_cast<uint8_t>((filter_id >> 16) & 0xFF);
  packet[8]  = static_cast<uint8_t>((filter_id >> 24) & 0xFF);

  // Block ID: frames matching this ID are blocked (0 = block nothing)
  packet[9]  = static_cast<uint8_t>(block_id & 0xFF);
  packet[10] = static_cast<uint8_t>((block_id >> 8) & 0xFF);
  packet[11] = static_cast<uint8_t>((block_id >> 16) & 0xFF);
  packet[12] = static_cast<uint8_t>((block_id >> 24) & 0xFF);

  packet[13] = static_cast<uint8_t>(mode);                   // operating mode
  packet[14] = disable_auto_retransmit ? 0x01 : 0x00;        // retransmit control
  // bytes [15..18] reserved, already zeroed by vector initialisation
  packet[19] = checksum_low8(packet, 2, 18);                  // checksum

  return packet;
}

/// Encode a CANFrame into the Waveshare binary frame format.
/// Handles both standard (11-bit) and extended (29-bit) IDs, and both
/// data and RTR frames. The CAN ID is packed little-endian; for extended
/// frames only the lower 29 bits are used (top 3 bits of byte 4 are masked).
std::vector<uint8_t> CANComms::encode_frame(const CANFrame & frame) const
{
  std::vector<uint8_t> packet;
  packet.reserve(1 + 1 + (frame.extended ? 4 : 2) + frame.dlc + 1);

  packet.push_back(0xAA);   // start-of-frame

  // Build the type byte from the frame properties
  uint8_t type = 0xC0;                                   // frame marker: bits 7-6 always set
  if (frame.extended) { type |= (1u << 5); }             // bit 5: extended ID
  if (frame.remote)   { type |= (1u << 4); }             // bit 4: RTR
  type |= static_cast<uint8_t>(frame.dlc & 0x0F);        // bits 3-0: DLC
  packet.push_back(type);

  if (frame.extended)
  {
    // 29-bit ID packed as 4 bytes little-endian; top 3 bits of byte 4 unused
    packet.push_back(static_cast<uint8_t>(frame.id & 0xFF));
    packet.push_back(static_cast<uint8_t>((frame.id >> 8) & 0xFF));
    packet.push_back(static_cast<uint8_t>((frame.id >> 16) & 0xFF));
    packet.push_back(static_cast<uint8_t>((frame.id >> 24) & 0x1F));
  }
  else
  {
    // 11-bit standard ID packed as 2 bytes little-endian; top 5 bits of byte 2 unused
    if (frame.id > 0x7FF)
    {
      throw std::runtime_error("Standard CAN ID out of range: " + std::to_string(frame.id));
    }
    packet.push_back(static_cast<uint8_t>(frame.id & 0xFF));
    packet.push_back(static_cast<uint8_t>((frame.id >> 8) & 0x07));
  }

  // RTR frames carry no data payload — only the ID and DLC are transmitted
  if (!frame.remote)
  {
    for (uint8_t i = 0; i < frame.dlc; ++i)
    {
      packet.push_back(frame.data[i]);
    }
  }

  packet.push_back(0x55);   // end-of-frame
  return packet;
}

/// Decode a raw Waveshare packet (already read from serial) into a CANFrame.
/// Validates start/end markers, type byte, DLC, and packet length before
/// populating the frame. Returns false if any check fails.
bool CANComms::decode_frame(const std::vector<uint8_t> & packet, CANFrame & frame) const
{
  if (packet.size() < 4)
  {
    return false;
  }

  if (packet.front() != 0xAA || packet.back() != 0x55)
  {
    return false;   // missing start or end marker
  }

  const uint8_t type = packet[1];

  if ((type & 0xC0) != 0xC0)
  {
    return false;   // not a valid frame type byte
  }

  frame.extended = ((type >> 5) & 0x01) != 0;
  frame.remote   = ((type >> 4) & 0x01) != 0;
  frame.dlc      = static_cast<uint8_t>(type & 0x0F);

  if (frame.dlc > 8)
  {
    return false;   // CAN DLC cannot exceed 8
  }

  // Verify the total packet length matches what the type byte implies
  const std::size_t id_len = frame.extended ? 4 : 2;
  const std::size_t expected_size =
    1 + 1 + id_len + (frame.remote ? 0 : frame.dlc) + 1;  // AA + type + id + data + 55

  if (packet.size() != expected_size)
  {
    return false;
  }

  if (frame.extended)
  {
    // Reconstruct 29-bit ID from 4 little-endian bytes; mask top byte to 29 bits
    frame.id =
      (static_cast<uint32_t>(packet[2]) <<  0) |
      (static_cast<uint32_t>(packet[3]) <<  8) |
      (static_cast<uint32_t>(packet[4]) << 16) |
      ((static_cast<uint32_t>(packet[5]) & 0x1F) << 24);
  }
  else
  {
    // Reconstruct 11-bit standard ID from 2 little-endian bytes
    frame.id =
      static_cast<uint32_t>(packet[2]) |
      (static_cast<uint32_t>(packet[3] & 0x07) << 8);
  }

  // Zero the full data array so bytes beyond DLC are always clean
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

/// Read bytes from serial until a complete Waveshare frame is assembled.
///
/// Scans byte-by-byte for 0xAA start marker, then reads the type byte to
/// determine how many more bytes to expect (ID length + data length + 0x55).
/// Reads exactly the right number of bytes rather than searching for 0x55,
/// since 0x55 can appear legitimately in the payload.
///
/// Returns false if any byte times out (timeout_ms_ per byte).
bool CANComms::read_variable_packet(std::vector<uint8_t> & packet)
{
  packet.clear();

  uint8_t byte = 0;

  // Scan forward until we find a start-of-frame byte
  while (true)
  {
    if (!read_byte(byte))
    {
      return false;   // timeout with no data
    }

    if (byte == 0xAA)
    {
      packet.push_back(byte);
      break;
    }
    // Any other byte is discarded — we are out of sync with the frame boundary
  }

  // Read the type byte which tells us the frame geometry
  uint8_t type = 0;
  if (!read_byte(type))
  {
    return false;
  }
  packet.push_back(type);

  if ((type & 0xC0) != 0xC0)
  {
    return false;   // not a valid type byte
  }

  const bool extended  = ((type >> 5) & 0x01) != 0;
  const bool remote    = ((type >> 4) & 0x01) != 0;
  const uint8_t dlc    = static_cast<uint8_t>(type & 0x0F);

  if (dlc > 8)
  {
    return false;
  }

  const std::size_t id_len   = extended ? 4 : 2;
  const std::size_t data_len = remote ? 0 : dlc;
  const std::size_t remaining = id_len + data_len + 1;   // id + data + 0x55

  for (std::size_t i = 0; i < remaining; ++i)
  {
    if (!read_byte(byte))
    {
      return false;   // timeout mid-frame
    }
    packet.push_back(byte);
  }

  // The last byte we read should be the end-of-frame marker
  if (packet.back() != 0x55)
  {
    return false;
  }

  return true;
}

// ---------------------------------------------------------------------------
// Low-level serial I/O (private)
// ---------------------------------------------------------------------------

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