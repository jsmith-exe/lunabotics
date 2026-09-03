#pragma once
#include <vector>

#include <linux/can.h>

class SocketCanInterface
{
public:
  SocketCanInterface() = default;
  void connect(const std::string & interface_name_);
  void close() const;
  ~SocketCanInterface();

  // Non-copyable: this object owns a raw file descriptor.
  SocketCanInterface(const SocketCanInterface &) = delete;
  SocketCanInterface & operator=(const SocketCanInterface &) = delete;

  bool send_frame(uint32_t can_id, const std::vector<uint8_t> & payload, bool is_extended = false);
  bool send_extended_frame(uint32_t can_id, const std::vector<uint8_t> & payload);
  bool receive_frame(can_frame & received_frame, int timeout_ms = 1000);
private:
  int file_descriptor_ = -1;
};