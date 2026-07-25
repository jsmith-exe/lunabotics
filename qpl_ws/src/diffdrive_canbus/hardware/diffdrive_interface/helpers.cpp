#include <string>
#include <cmath>
#include <algorithm>

namespace diffdrive_canbus {
  bool string_to_bool(const std::string & value)
  {
    return value == "true" ||
           value == "True" ||
           value == "TRUE" ||
           value == "1" ||
           value == "yes" ||
           value == "on";
  }

  double apply_deadband(double value, double deadband)
  {
    if (std::fabs(value) <= deadband)
    {
      return 0.0;
    }

    return value;
  }

  double clean_command(double value, double deadband)
  {
    if (!std::isfinite(value))
    {
      return 0.0;
    }

    return apply_deadband(value, deadband);
  }

  // Clamp duty-cycle/throttle commands to the safe SPARK MAX range.
  double clamp_throttle(double value)
  {
    if (!std::isfinite(value))
    {
      return 0.0;
    }

    return std::clamp(value, -1.0, 1.0);
  }

  double apply_throttle_deadband(double value, double deadband)
  {
    const double safe_deadband = std::max(0.0, deadband);

    if (std::fabs(value) <= safe_deadband)
    {
      return 0.0;
    }

    return value;
  }
}