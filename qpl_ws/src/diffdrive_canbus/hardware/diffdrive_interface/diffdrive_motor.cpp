#include "diffdrive_canbus/diffdrive_interface.hpp"
#include "diffdrive_canbus/can_device.hpp"

namespace diffdrive_canbus {
  void Motor::set_velocity(double velocity) {
      this->set_velocity_rad_per_sec(static_cast<float>(velocity));
  }
}