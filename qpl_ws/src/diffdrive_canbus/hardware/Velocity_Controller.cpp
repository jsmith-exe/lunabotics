#include "diffdrive_canbus/Velocity_Controller.hpp"

#include <cmath>

namespace diffdrive_canbus
{

PIDVelocity::PIDVelocity(
  float Kp,
  float Ki,
  float Kd,
  float integral_reset,
  float out_min,
  float out_max,
  float d_tau)
: Kp_(Kp),
  Ki_(Ki),
  Kd_(Kd),
  integral_(0.0f),
  prev_meas_(0.0f),
  dMeas_filt_(0.0f),
  last_time_(std::chrono::steady_clock::now()),
  integral_reset_(integral_reset),
  out_min_(out_min),
  out_max_(out_max),
  d_tau_(d_tau),
  prev_P_(0.0f),
  prev_I_(0.0f),
  prev_D_(0.0f),
  output_state_(0.0f)
{
}

std::pair<float, float> PIDVelocity::update(float setpoint, float measurement)
{
  const auto tic = std::chrono::steady_clock::now();
  const auto now = tic;

  const std::chrono::duration<float> dt_duration = now - last_time_;
  const float dt = dt_duration.count();

  if (dt <= 1e-9f)
  {
    const auto toc = std::chrono::steady_clock::now();
    const std::chrono::duration<float, std::milli> latency = toc - tic;

    last_time_ = now;
    prev_meas_ = measurement;

    return {0.0f, latency.count()};
  }

  const float error = setpoint - measurement;

  if (integral_reset_ >= 0.0f && std::fabs(error) <= integral_reset_)
  {
    integral_ = 0.0f;
  }

  const float dMeas = (measurement - prev_meas_) / dt;

  if (d_tau_ > 0.0f)
  {
    const float a = d_tau_ / (d_tau_ + dt);
    dMeas_filt_ = a * dMeas_filt_ + (1.0f - a) * dMeas;
  }
  else
  {
    dMeas_filt_ = dMeas;
  }

  const float integral_candidate = integral_ + error * dt;

  const float P = Kp_ * error;
  const float I = Ki_ * integral_candidate;
  const float D = -Kd_ * dMeas_filt_;

  const float delta_unclamped =
    (P - prev_P_) +
    (I - prev_I_) +
    (D - prev_D_);

  const float predicted_unclamped = output_state_ + delta_unclamped;
  const float predicted_clamped = clampf(predicted_unclamped, out_min_, out_max_);

  const float delta_output = predicted_clamped - output_state_;
  const bool saturated = (predicted_clamped != predicted_unclamped);

  if (!saturated)
  {
    integral_ = integral_candidate;
    prev_I_ = I;
  }
  else
  {
    prev_I_ = Ki_ * integral_;
  }

  prev_P_ = P;
  prev_D_ = D;

  output_state_ = predicted_clamped;

  prev_meas_ = measurement;
  last_time_ = now;

  const auto toc = std::chrono::steady_clock::now();
  const std::chrono::duration<float, std::milli> latency = toc - tic;

  return {delta_output, latency.count()};
}

void PIDVelocity::reset()
{
  integral_ = 0.0f;
  prev_meas_ = 0.0f;
  dMeas_filt_ = 0.0f;

  prev_P_ = 0.0f;
  prev_I_ = 0.0f;
  prev_D_ = 0.0f;

  output_state_ = 0.0f;
  last_time_ = std::chrono::steady_clock::now();
}

void PIDVelocity::setGains(float Kp, float Ki, float Kd)
{
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
}

void PIDVelocity::setIntegralReset(float integral_reset)
{
  integral_reset_ = integral_reset;
}

void PIDVelocity::setDerivativeFilterTau(float tau_sec)
{
  d_tau_ = tau_sec;
}

void PIDVelocity::setOutputState(float u)
{
  output_state_ = clampf(u, out_min_, out_max_);
}

float PIDVelocity::getOutputState() const
{
  return output_state_;
}

float PIDVelocity::clampf(float x, float lo, float hi)
{
  if (x < lo)
  {
    return lo;
  }

  if (x > hi)
  {
    return hi;
  }

  return x;
}

}  // namespace diffdrive_canbus