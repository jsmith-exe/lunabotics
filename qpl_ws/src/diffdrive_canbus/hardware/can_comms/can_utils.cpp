#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <cstring>

#include "diffdrive_canbus/diffdrive_interface.hpp"
#include "diffdrive_canbus/can_comms.hpp"

namespace diffdrive_canbus
{

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

uint8_t get_frc_device_id_from_can_id(uint32_t can_id)
{
  // FRC extended CAN IDs store the device ID in the lowest 6 bits.
  return static_cast<uint8_t>(can_id & 0x3F);
}

bool is_actuator_status3_id(uint32_t can_id, uint8_t device_id)
{
  const uint32_t clean_id = can_id & 0x1FFFFFFF;
  return clean_id == (SPARKMAX_PERIODIC_STATUS_3_BASE_ID + static_cast<uint32_t>(device_id));
}

uint16_t le_u16_from_frame_data(const uint8_t data[8], std::size_t offset)
{
  if (offset + sizeof(uint16_t) > 8)
  {
    return 0;
  }

  uint16_t value = 0;
  std::memcpy(&value, data + offset, sizeof(uint16_t));
  return value;
}

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
}

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
}

/// Format a CANFrame as a human-readable string for logging.
/// Example output: EXT ID=0x02051841 DLC=8 DATA=[0x39 0x8A 0xF8 0x43 ...]
std::string frame_to_string(const CANFrame & frame)
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

}  // namespace diffdrive_canbus