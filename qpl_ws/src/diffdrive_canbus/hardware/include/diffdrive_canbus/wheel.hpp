#ifndef DIFFDRIVE_CANBUS__WHEEL_HPP_
#define DIFFDRIVE_CANBUS__WHEEL_HPP_

#include <cmath>
#include <cstdint>
#include <string>

namespace diffdrive_canbus
{

class Wheel
{
public:
  std::string name = "";
  int32_t enc = 0;
  int32_t prev_enc = 0;

  double cmd = 0.0;   // commanded angular velocity [rad/s]
  double pos = 0.0;   // wheel position [rad]
  double vel = 0.0;   // wheel velocity [rad/s]

  double rads_per_count = 0.0;

  Wheel() = default;

  Wheel(const std::string & wheel_name, int counts_per_rev)
  {
    setup(wheel_name, counts_per_rev);
  }

  void setup(const std::string & wheel_name, int counts_per_rev)
  {
    name = wheel_name;

    if (counts_per_rev <= 0)
    {
      rads_per_count = 0.0;
      return;
    }

    rads_per_count = (2.0 * M_PI) / static_cast<double>(counts_per_rev);
  }

  double calc_enc_angle() const
  {
    return static_cast<double>(enc) * rads_per_count;
  }
};

}  // namespace diffdrive_canbus

#endif  // DIFFDRIVE_CANBUS__WHEEL_HPP_