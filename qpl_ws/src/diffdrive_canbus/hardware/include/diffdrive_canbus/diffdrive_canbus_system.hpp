#ifndef DIFFDRIVE_CANBUS__DIFFDRIVE_CANBUS_SYSTEM_HPP_
#define DIFFDRIVE_CANBUS__DIFFDRIVE_CANBUS_SYSTEM_HPP_

#include <string>

constexpr uint32_t SPARKMAX_PERIODIC_STATUS_3_BASE_ID = 0x020518C0; // TODO rename to be clearer  (can comms helpers)

namespace diffdrive_canbus {
  // helpers
  bool string_to_bool(const std::string & value);
  double apply_deadband(double value, double deadband);
  double clean_command(double value, double deadband);
  double clamp_throttle(double value);
  double apply_throttle_deadband(double value, double deadband);

  // can comms helpers
  uint8_t get_frc_device_id_from_can_id(uint32_t can_id);
  uint8_t get_frc_api_index_from_can_id(uint32_t can_id);
  bool is_actuator_status3_id(uint32_t can_id, uint8_t device_id);
  uint16_t le_u16_from_frame_data(const uint8_t data[8], std::size_t offset);
  std::string can_data_to_hex_string(const uint8_t data[8], uint8_t dlc);
}
#endif  // DIFFDRIVE_CANBUS__DIFFDRIVE_CANBUS_SYSTEM_HPP_