#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "diffdrive_canbus/CAN_comms.hpp"

using diffdrive_canbus::CANComms;
using diffdrive_canbus::CANFrame;
using diffdrive_canbus::CANMode;
using diffdrive_canbus::SparkMaxCanIdFields;
using diffdrive_canbus::parse_frc_extended_can_id;

namespace
{
volatile std::sig_atomic_t g_running = 1;

void signal_handler(int)
{
  g_running = 0;
}

uint32_t parse_u32(const std::string & s)
{
  size_t idx = 0;
  const unsigned long value = std::stoul(s, &idx, 0);
  if (idx != s.size())
  {
    throw std::runtime_error("Invalid integer: " + s);
  }
  return static_cast<uint32_t>(value);
}

uint8_t parse_u8(const std::string & s)
{
  const uint32_t value = parse_u32(s);
  if (value > 0xFF)
  {
    throw std::runtime_error("Byte out of range: " + s);
  }
  return static_cast<uint8_t>(value);
}

std::string id_fields_to_string(const SparkMaxCanIdFields & f)
{
  std::ostringstream ss;
  ss << "device_type=" << static_cast<int>(f.device_type)
     << " manufacturer=" << static_cast<int>(f.manufacturer)
     << " api_class=" << static_cast<int>(f.api_class)
     << " api_index=" << static_cast<int>(f.api_index)
     << " device_id=" << static_cast<int>(f.device_id);
  return ss.str();
}

std::string frame_with_fields_to_string(const CANFrame & frame)
{
  std::ostringstream ss;
  ss << CANComms::frame_to_string(frame);

  if (frame.extended)
  {
    const auto fields = parse_frc_extended_can_id(frame.id);
    ss << " | " << id_fields_to_string(fields);
  }

  return ss.str();
}

void print_usage(const char * prog)
{
  std::cerr
    << "\nUsage:\n"
    << "  " << prog << " listen <serial_device> [can_baud]\n"
    << "  " << prog << " listen-filter <serial_device> <device_id> [can_baud]\n"
    << "  " << prog << " send <serial_device> <can_id> <ext|std> <byte0> [byte1 ... byte7]\n"
    << "  " << prog << " spam <serial_device> <can_id> <ext|std> <period_ms> <byte0> [byte1 ... byte7]\n"
    << "\nExamples:\n"
    << "  " << prog << " listen /dev/ttyUSB0 1000000\n"
    << "  " << prog << " listen-filter /dev/ttyUSB0 1 1000000\n"
    << "  " << prog << " send /dev/ttyUSB0 0x2051801 ext 0x00 0x00 0x00 0x00\n"
    << "  " << prog << " spam /dev/ttyUSB0 0x2051801 ext 100 0x00 0x00 0x80 0x3F\n"
    << "\nNotes:\n"
    << "  - SPARK MAX traffic should normally use ext.\n"
    << "  - can_id accepts decimal or hex.\n"
    << "  - byte values accept decimal or hex.\n"
    << "  - listen-filter matches the FRC/SPARK device_id field (0-63).\n\n";
}

CANFrame build_frame(
  uint32_t can_id,
  bool extended,
  const std::vector<uint8_t> & data)
{
  if (data.size() > 8)
  {
    throw std::runtime_error("CAN data length cannot exceed 8 bytes");
  }

  CANFrame frame;
  frame.id = can_id;
  frame.extended = extended;
  frame.remote = false;
  frame.dlc = static_cast<uint8_t>(data.size());
  std::memset(frame.data, 0, sizeof(frame.data));

  for (size_t i = 0; i < data.size(); ++i)
  {
    frame.data[i] = data[i];
  }

  return frame;
}

void run_listen(CANComms & comms)
{
  std::cout << "Listening... Press Ctrl+C to stop.\n";

  while (g_running)
  {
    CANFrame frame;
    if (comms.read_frame(frame, false))
    {
      std::cout << frame_with_fields_to_string(frame) << std::endl;
    }
  }
}

void run_listen_filter(CANComms & comms, uint8_t wanted_device_id)
{
  std::cout << "Listening for device_id=" << static_cast<int>(wanted_device_id)
            << "... Press Ctrl+C to stop.\n";

  while (g_running)
  {
    CANFrame frame;
    if (!comms.read_frame(frame, false))
    {
      continue;
    }

    if (!frame.extended)
    {
      continue;
    }

    const auto fields = parse_frc_extended_can_id(frame.id);
    if (fields.device_id == wanted_device_id)
    {
      std::cout << frame_with_fields_to_string(frame) << std::endl;
    }
  }
}

void run_send(CANComms & comms, uint32_t can_id, bool extended, const std::vector<uint8_t> & data)
{
  const CANFrame frame = build_frame(can_id, extended, data);

  std::cout << "Sending one frame:\n";
  std::cout << frame_with_fields_to_string(frame) << std::endl;

  if (!comms.send_frame(frame, true))
  {
    throw std::runtime_error("Failed to send frame");
  }
}

void run_spam(
  CANComms & comms,
  uint32_t can_id,
  bool extended,
  int period_ms,
  const std::vector<uint8_t> & data)
{
  if (period_ms <= 0)
  {
    throw std::runtime_error("period_ms must be > 0");
  }

  const CANFrame frame = build_frame(can_id, extended, data);

  std::cout << "Spamming frame every " << period_ms << " ms:\n";
  std::cout << frame_with_fields_to_string(frame) << std::endl;
  std::cout << "Press Ctrl+C to stop.\n";

  while (g_running)
  {
    if (!comms.send_frame(frame, false))
    {
      std::cerr << "Send failed\n";
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  std::signal(SIGINT, signal_handler);

  if (argc < 3)
  {
    print_usage(argv[0]);
    return 1;
  }

  try
  {
    const std::string mode = argv[1];
    const std::string serial_device = argv[2];

    constexpr int32_t serial_baud = 2000000;
    constexpr int32_t timeout_ms = 50;
    int32_t can_baud = 1000000;

    CANComms comms;
    comms.connect(serial_device, serial_baud, timeout_ms);

    if (mode == "listen")
    {
      if (argc >= 4)
      {
        can_baud = static_cast<int32_t>(parse_u32(argv[3]));
      }

      if (!comms.configure_adapter(
            can_baud,
            true,   // extended filter mode
            0x00000000,
            0x00000000,
            CANMode::NORMAL,
            false,
            true))
      {
        throw std::runtime_error("Failed to configure CAN adapter");
      }

      std::cout << "Connected to " << serial_device
                << " at serial baud " << serial_baud
                << ", CAN baud " << can_baud << "\n";

      run_listen(comms);
      return 0;
    }

    if (mode == "listen-filter")
    {
      if (argc < 4)
      {
        print_usage(argv[0]);
        return 1;
      }

      const uint8_t device_id = parse_u8(argv[3]);
      if (argc >= 5)
      {
        can_baud = static_cast<int32_t>(parse_u32(argv[4]));
      }

      if (!comms.configure_adapter(
            can_baud,
            true,
            0x00000000,
            0x00000000,
            CANMode::NORMAL,
            false,
            true))
      {
        throw std::runtime_error("Failed to configure CAN adapter");
      }

      std::cout << "Connected to " << serial_device
                << " at serial baud " << serial_baud
                << ", CAN baud " << can_baud << "\n";

      run_listen_filter(comms, device_id);
      return 0;
    }

    if (mode == "send")
    {
      if (argc < 6)
      {
        print_usage(argv[0]);
        return 1;
      }

      const uint32_t can_id = parse_u32(argv[3]);
      const std::string id_type = argv[4];
      const bool extended = (id_type == "ext");

      if (!(extended || id_type == "std"))
      {
        throw std::runtime_error("ID type must be 'ext' or 'std'");
      }

      std::vector<uint8_t> data;
      for (int i = 5; i < argc; ++i)
      {
        data.push_back(parse_u8(argv[i]));
      }

      if (!comms.configure_adapter(
            can_baud,
            true,
            0x00000000,
            0x00000000,
            CANMode::NORMAL,
            false,
            true))
      {
        throw std::runtime_error("Failed to configure CAN adapter");
      }

      std::cout << "Connected to " << serial_device
                << " at serial baud " << serial_baud
                << ", CAN baud " << can_baud << "\n";

      run_send(comms, can_id, extended, data);
      return 0;
    }

    if (mode == "spam")
    {
      if (argc < 7)
      {
        print_usage(argv[0]);
        return 1;
      }

      const uint32_t can_id = parse_u32(argv[3]);
      const std::string id_type = argv[4];
      const bool extended = (id_type == "ext");

      if (!(extended || id_type == "std"))
      {
        throw std::runtime_error("ID type must be 'ext' or 'std'");
      }

      const int period_ms = static_cast<int>(parse_u32(argv[5]));

      std::vector<uint8_t> data;
      for (int i = 6; i < argc; ++i)
      {
        data.push_back(parse_u8(argv[i]));
      }

      if (!comms.configure_adapter(
            can_baud,
            true,
            0x00000000,
            0x00000000,
            CANMode::NORMAL,
            false,
            true))
      {
        throw std::runtime_error("Failed to configure CAN adapter");
      }

      std::cout << "Connected to " << serial_device
                << " at serial baud " << serial_baud
                << ", CAN baud " << can_baud << "\n";

      run_spam(comms, can_id, extended, period_ms, data);
      return 0;
    }

    print_usage(argv[0]);
    return 1;
  }
  catch (const std::exception & e)
  {
    std::cerr << "ERROR: " << e.what() << std::endl;
    return 1;
  }
}