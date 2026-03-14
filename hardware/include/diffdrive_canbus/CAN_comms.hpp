#ifndef DIFFDRIVE_CANBUS__CAN_COMMS_HPP_
#define DIFFDRIVE_CANBUS__CAN_COMMS_HPP_

#include <cstdint>
#include <cstring>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <filesystem>


#include <libserial/SerialPort.h>

namespace diffdrive_canbus
{

struct CANFrame
{
  uint32_t id = 0;
  uint8_t dlc = 0;
  uint8_t data[8] = {0};
  bool extended = false;
  bool remote = false;
};

inline LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  switch (baud_rate)
  {
    case 1200:   return LibSerial::BaudRate::BAUD_1200;
    case 1800:   return LibSerial::BaudRate::BAUD_1800;
    case 2400:   return LibSerial::BaudRate::BAUD_2400;
    case 4800:   return LibSerial::BaudRate::BAUD_4800;
    case 9600:   return LibSerial::BaudRate::BAUD_9600;
    case 19200:  return LibSerial::BaudRate::BAUD_19200;
    case 38400:  return LibSerial::BaudRate::BAUD_38400;
    case 57600:  return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cerr << "Unsupported serial baud rate " << baud_rate
                << ", defaulting to 115200" << std::endl;
      return LibSerial::BaudRate::BAUD_115200;
  }
}

class CANComms
{
public:
  CANComms() = default;

  ~CANComms()
  {
    if (connected())
    {
      disconnect();
    }
  }

  void connect(const std::string & device, int32_t serial_baud_rate, int32_t timeout_ms)
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
        "Failed to open CAN adapter serial device '" + device + "': " + e.what());
    }

  }

  void disconnect()
  {
    if (serial_conn_.IsOpen())
    {
      serial_conn_.Close();
    }
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }

  bool configure_adapter(int32_t can_baud_rate, bool print_output = false)
  {
    const std::string cmd = build_config_command(can_baud_rate);
    const std::string response = send_command(cmd, print_output);
    return !response.empty();
  }

  bool send_can_frame(const CANFrame & frame, bool print_output = false)
  {
    if (!connected())
    {
      std::cerr << "CAN adapter not connected." << std::endl;
      return false;
    }

    const std::string cmd = encode_frame(frame);
    const std::string response = send_command(cmd, print_output);
    return !response.empty();
  }

  bool read_can_frame(CANFrame & frame, bool print_output = false)
  {
    if (!connected())
    {
      std::cerr << "CAN adapter not connected." << std::endl;
      return false;
    }

    std::string response;
    try
    {
      serial_conn_.ReadLine(response, '\n', timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout &)
    {
      return false;
    }
    catch (const std::exception & e)
    {
      std::cerr << "Failed reading CAN frame from adapter: " << e.what() << std::endl;
      return false;
    }

    if (print_output)
    {
      std::cout << "Recv: " << response << std::endl;
    }

    return decode_frame(response, frame);
  }

  // --------------------------------------------------------------------------
  // High-level helpers used by diffbot_system.cpp
  // These are currently placeholder wrappers until you implement the actual
  // Waveshare USB-CAN-A serial protocol.
  // --------------------------------------------------------------------------

  bool write_motor_command(int can_id, int command, bool print_output = false)
  {
    CANFrame frame;
    frame.id = static_cast<uint32_t>(can_id);
    frame.dlc = 4;

    // Encode signed 32-bit command little-endian as placeholder
    frame.data[0] = static_cast<uint8_t>(command & 0xFF);
    frame.data[1] = static_cast<uint8_t>((command >> 8) & 0xFF);
    frame.data[2] = static_cast<uint8_t>((command >> 16) & 0xFF);
    frame.data[3] = static_cast<uint8_t>((command >> 24) & 0xFF);

    return send_can_frame(frame, print_output);
  }

  bool read_motor_encoder(int can_id, int & encoder_count, bool print_output = false)
  {
    CANFrame request;
    request.id = static_cast<uint32_t>(can_id);
    request.dlc = 0;

    if (!send_can_frame(request, print_output))
    {
      return false;
    }

    CANFrame response;
    if (!read_can_frame(response, print_output))
    {
      return false;
    }

    if (response.dlc < 4)
    {
      std::cerr << "Encoder response too short for CAN ID " << can_id << std::endl;
      return false;
    }

    // Placeholder decode: signed 32-bit little-endian
    encoder_count =
      static_cast<int>(response.data[0]) |
      (static_cast<int>(response.data[1]) << 8) |
      (static_cast<int>(response.data[2]) << 16) |
      (static_cast<int>(response.data[3]) << 24);

    return true;
  }

private:
  std::string send_command(const std::string & cmd, bool print_output = false)
  {
    if (!connected())
    {
      throw std::runtime_error("Attempted to send command while CAN adapter is disconnected.");
    }

    serial_conn_.FlushIOBuffers();
    serial_conn_.Write(cmd);

    std::string response;
    try
    {
      serial_conn_.ReadLine(response, '\n', timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout &)
    {
      std::cerr << "Timed out waiting for CAN adapter response." << std::endl;
      return "";
    }
    catch (const std::exception & e)
    {
      std::cerr << "Error while communicating with CAN adapter: " << e.what() << std::endl;
      return "";
    }

    if (print_output)
    {
      std::cout << "Sent: " << cmd << " Recv: " << response << std::endl;
    }

    return response;
  }

  std::string build_config_command(int32_t can_baud_rate) const
  {
    // TODO: Replace with real Waveshare USB-CAN-A config command
    return "SET_CAN_BAUD " + std::to_string(can_baud_rate) + "\r\n";
  }

  std::string encode_frame(const CANFrame & frame) const
  {
    // TODO: Replace with real Waveshare USB-CAN-A frame encoding
    std::ostringstream ss;
    ss << "SEND " << frame.id << " " << static_cast<int>(frame.dlc);

    for (uint8_t i = 0; i < frame.dlc; ++i)
    {
      ss << " " << static_cast<int>(frame.data[i]);
    }

    ss << "\r\n";
    return ss.str();
  }

  bool decode_frame(const std::string & response, CANFrame & frame) const
  {
    // TODO: Replace with real Waveshare USB-CAN-A frame decoding
    //
    // Placeholder expected format:
    // RECV <id> <dlc> <b0> <b1> ...
    std::istringstream ss(response);
    std::string tag;
    int id = 0;
    int dlc = 0;

    ss >> tag >> id >> dlc;
    if (!ss || tag != "RECV" || dlc < 0 || dlc > 8)
    {
      return false;
    }

    frame.id = static_cast<uint32_t>(id);
    frame.dlc = static_cast<uint8_t>(dlc);

    for (int i = 0; i < dlc; ++i)
    {
      int byte_val = 0;
      ss >> byte_val;
      if (!ss)
      {
        return false;
      }
      frame.data[i] = static_cast<uint8_t>(byte_val);
    }

    return true;
  }

  LibSerial::SerialPort serial_conn_;
  int timeout_ms_ = 100;
};

}  // namespace diffdrive_canbus

#endif  // DIFFDRIVE_CANBUS__CAN_COMMS_HPP_