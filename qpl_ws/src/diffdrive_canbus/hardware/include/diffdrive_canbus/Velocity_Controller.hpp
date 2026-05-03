#pragma once

#include <chrono>
#include <utility>

namespace diffdrive_canbus
{

class PIDVelocity
{
public:
  PIDVelocity(
    float Kp,
    float Ki,
    float Kd,
    float integral_reset,
    float out_min,
    float out_max,
    float d_tau);

  std::pair<float, float> update(float setpoint, float measurement);

  void reset();

  void setGains(float Kp, float Ki, float Kd);
  void setIntegralReset(float integral_reset);
  void setDerivativeFilterTau(float tau_sec);
  void setOutputState(float u);

  float getOutputState() const;

private:
  float Kp_;
  float Ki_;
  float Kd_;

  float integral_;
  float prev_meas_;
  float dMeas_filt_;

  std::chrono::steady_clock::time_point last_time_;

  float integral_reset_;
  float out_min_;
  float out_max_;
  float d_tau_;

  float prev_P_;
  float prev_I_;
  float prev_D_;

  float output_state_;

  static float clampf(float x, float lo, float hi);
};

}  // namespace diffdrive_canbus