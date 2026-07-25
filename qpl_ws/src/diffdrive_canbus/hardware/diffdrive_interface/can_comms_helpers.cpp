#include <cstdint>
#include <cstring>
#include <string>
#include <iomanip>

constexpr uint32_t SPARKMAX_PERIODIC_STATUS_3_BASE_ID = 0x020518C0; // TODO rename to be clearer

uint8_t get_frc_device_id_from_can_id(uint32_t can_id)
{
  // FRC extended CAN IDs store the device ID in the lowest 6 bits.
  return static_cast<uint8_t>(can_id & 0x3F);
}

uint8_t get_frc_api_index_from_can_id(uint32_t can_id)
{
  // The API index is the nibble immediately above the 6-bit device ID.
  return static_cast<uint8_t>((can_id >> 6) & 0x0F);
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

std::string can_data_to_hex_string(const uint8_t data[8], uint8_t dlc)
{
  std::ostringstream oss;
  oss << std::hex << std::uppercase << std::setfill('0');

  for (uint8_t i = 0; i < dlc && i < 8; ++i)
  {
    if (i > 0)
    {
      oss << ' ';
    }

    oss << "0x" << std::setw(2) << static_cast<int>(data[i]);
  }

  return oss.str();
}
