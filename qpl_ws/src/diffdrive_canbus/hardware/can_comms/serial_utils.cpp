/// Helper functions for reading and parsing bytes from the serial port

#include "diffdrive_canbus/can_comms.hpp"

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace diffdrive_canbus {

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

} // namespace diffdrive_canbus