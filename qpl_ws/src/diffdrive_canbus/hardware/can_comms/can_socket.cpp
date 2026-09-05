#include <iostream>
#include <cstring>
#include <stdexcept>
#include <vector>

#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <linux/can.h>

#include "diffdrive_canbus/can_socket.hpp"

#include "diffdrive_canbus/can_comms.hpp"

void SocketCanInterface::connect(const std::string & interface_name_)
{
  file_descriptor_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (file_descriptor_ < 0)
  {
    throw std::runtime_error("Failed to open CAN socket: " + std::string(std::strerror(errno)));
  }

  ifreq interface_request {};
  std::strncpy(interface_request.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1); // Sets interface request name
  if (ioctl(file_descriptor_, SIOCGIFINDEX, &interface_request) < 0)
  {
    ::close(file_descriptor_);
    throw std::runtime_error("Failed to find interface " + interface_name_ + ": " + std::strerror(errno));
  }

  sockaddr_can address {};
  address.can_family = AF_CAN;
  address.can_ifindex = interface_request.ifr_ifindex;
  if (bind(file_descriptor_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0)
  {
    ::close(file_descriptor_);
    throw std::runtime_error("Failed to bind CAN socket: " + std::string(std::strerror(errno)));
  }
}

void SocketCanInterface::close() const {
  if (file_descriptor_ >= 0)
  {
    ::close(file_descriptor_);
  }
}

SocketCanInterface::~SocketCanInterface()
{
  close();
}

bool SocketCanInterface::send_frame(uint32_t can_id, const std::vector<uint8_t> & payload, bool is_extended)
{
  if (payload.size() > 8)
  {
    std::cerr << "CAN send failed: payload of " << payload.size() << " bytes exceeds 8-byte max.\n";
    return false;
  }

  can_frame frame {}; // value-initialized: frame.data is already zeroed
  frame.can_id = is_extended ? (can_id | CAN_EFF_FLAG) : can_id;
  frame.can_dlc = static_cast<uint8_t>(payload.size());
  std::copy(payload.begin(), payload.end(), frame.data);

  const ssize_t bytes_written = write(file_descriptor_, &frame, sizeof(frame));
  const bool success = bytes_written == sizeof(frame);

  return success;
}

bool SocketCanInterface::send_extended_frame(uint32_t can_id, const std::vector<uint8_t> & payload)
{
  return send_frame(can_id, payload, /*is_extended=*/true);
}

// Blocking receive with timeout. Returns false on timeout or error.
bool SocketCanInterface::read_frame(can_frame & received_frame, int timeout_ms)
{
  timeval timeout {};
  timeout.tv_sec = timeout_ms / 1000;
  timeout.tv_usec = (timeout_ms % 1000) * 1000;
  setsockopt(file_descriptor_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

  const ssize_t bytes_read = read(file_descriptor_, &received_frame, sizeof(received_frame));
  return bytes_read == sizeof(received_frame);
}
