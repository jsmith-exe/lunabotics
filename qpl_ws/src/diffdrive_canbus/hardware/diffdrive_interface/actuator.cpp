#include <string>
#include <iomanip>
#include <chrono>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include "diffdrive_canbus/spark_max.hpp"
#include "diffdrive_canbus/diffdrive_canbus_system.hpp"

namespace diffdrive_canbus {
  void reset_actuator_state(LinearActuatorHW & act, const rclcpp::Logger & logger)
  {
    act.logger = logger;
    act.command = std::clamp(act.test_position_command, 0.0, 1.0);
    act.position = 0.0;
    act.voltage = 0.0;
    act.has_voltage = false;
    act.command_count = 0;
    act.last_sent_output = 999.0;
    act.has_raw_can_frame = false;
    act.has_status3_frame = false;
    act.has_analog_voltage_candidate = false;
    act.analog_voltage_candidate = 0.0;
    act.analog_voltage_source = "NONE";
    act.last_raw_can_id = 0;
    act.last_raw_dlc = 0;
    act.last_raw_data.fill(0);
    act.last_status3_can_id = 0;
    act.last_status3_dlc = 0;
    act.last_status3_data.fill(0);
    act.raw_can_frame_count = 0;
    act.status3_frame_count = 0;
    act.currently_commanded = false;
    act.last_feedback_print_time =
      std::chrono::steady_clock::now() + LINEAR_ACTUATOR_FEEDBACK_PRINT_PERIOD;
    act.last_feedback_time = std::chrono::steady_clock::time_point{};
    act.stall_latched = false;
    act.stall_latched_command = act.command;
    act.watchdog_initialised = false;
    act.watchdog_ref_position = 0.0;
    act.watchdog_ref_time = std::chrono::steady_clock::time_point{};
  }

  void request_actuator_status3_period(LinearActuatorHW & act, const std::string & label, CANComms & can_)
  {
    const uint32_t status3_id = SPARKMAX_PERIODIC_STATUS_3_BASE_ID + static_cast<uint32_t>(act.can_id);

    std::vector<uint8_t> data(2, 0x00);
    data[0] = static_cast<uint8_t>(LINEAR_ACTUATOR_STATUS3_PERIOD_MS & 0xFF);
    data[1] = static_cast<uint8_t>((LINEAR_ACTUATOR_STATUS3_PERIOD_MS >> 8) & 0xFF);

    const bool ok = can_.send_extended_frame(status3_id, data, true);

    if (ok)
    {
      RCLCPP_WARN(
        act.logger,
        "Requested SPARK MAX Status 3 period for %s linear actuator: id=%u can_id=0x%08X period=%u ms",
        label.c_str(),
        act.can_id,
        status3_id,
        LINEAR_ACTUATOR_STATUS3_PERIOD_MS);
    }
    else
    {
      RCLCPP_WARN(
        act.logger,
        "Failed to request SPARK MAX Status 3 period for %s linear actuator: id=%u can_id=0x%08X",
        label.c_str(),
        act.can_id,
        status3_id);
    }

    sleep_bus_gap();
  }

  // Closed-loop position servo for one linear actuator.
  //
  // The actuator is an open-loop duty device (0.0 -> -1.0 duty retract,
  // 0.5 -> stop, 1.0 -> +1.0 duty extend), but ros2_control presents it as a
  // position command in 0..1. We honour that contract by driving toward the
  // commanded position using the analog feedback and stopping once we get there,
  // so a held command parks the drum instead of stalling at an end-stop.
  void write_actuator_closed_loop(LinearActuatorHW & act, const std::string & label)
  {
    if (!act.spark)
    {
      RCLCPP_WARN_THROTTLE(
        act.logger,
        *rclcpp::Clock::make_shared(),
        1000,
        "Cannot write to %s linear actuator because SparkMax object is null",
        label.c_str());

      return;
    }

    // Target position requested through the ros2_control position interface.
    const double target =
      std::clamp(std::isfinite(act.command) ? act.command : 0.5, 0.0, 1.0);

    const auto now = std::chrono::steady_clock::now();

    // A new/changed target clears any latched stall so we retry the move.
    if (std::fabs(target - act.stall_latched_command) > act.position_tolerance)
    {
      act.stall_latched = false;
      act.watchdog_initialised = false;
    }

    // Feedback-loss failsafe: these actuators have no end-stop limit switches, so
    // we must never drive them open-loop/blind. If the analog position feedback is
    // missing (feedback disabled, wiring fault) or stale (frames stopped arriving),
    // hold the motor stopped rather than risk a runaway or a sustained stall.
    const bool feedback_fresh =
      act.has_voltage &&
      (now - act.last_feedback_time) <= ACTUATOR_FEEDBACK_TIMEOUT;

    if (!feedback_fresh)
    {
      send_actuator_duty(act, 0.0, label);
      RCLCPP_WARN_THROTTLE(
        act.logger,
        *rclcpp::Clock::make_shared(),
        1000,
        "%s LINEAR ACTUATOR: no fresh position feedback (has_voltage=%s) - holding stopped",
        label.c_str(),
        act.has_voltage ? "true" : "false");
      act.watchdog_initialised = false;
      return;
    }

    const double measured = std::clamp(act.position, 0.0, 1.0);
    const double error = target - measured;

    // Bang-bang: drive at full duty toward the target, stop inside the tolerance
    // band. The actuator cannot servo intermediate speeds, so there is no point
    // in a proportional term.
    double duty = 0.0;
    if (error > act.position_tolerance)
    {
      duty = 1.0;   // measured below target: extend
    }
    else if (error < -act.position_tolerance)
    {
      duty = -1.0;  // measured above target: retract
    }
    else
    {
      duty = 0.0;   // within tolerance: reached, stop
      act.stall_latched = false;
      act.watchdog_initialised = false;
    }

    // Stall/jam watchdog: while commanding motion we expect the measured position
    // to keep changing. If it stops progressing (jam, hard end-stop, or frozen
    // feedback) latch the motor off until the target changes, so we never sit at
    // full duty against a blocked load.
    if (duty != 0.0 && !act.stall_latched)
    {
      if (!act.watchdog_initialised)
      {
        act.watchdog_initialised = true;
        act.watchdog_ref_position = measured;
        act.watchdog_ref_time = now;
      }
      else if (std::fabs(measured - act.watchdog_ref_position) >= ACTUATOR_STALL_MOTION_EPSILON)
      {
        // Still making progress - reset the watchdog reference.
        act.watchdog_ref_position = measured;
        act.watchdog_ref_time = now;
      }
      else if ((now - act.watchdog_ref_time) >= ACTUATOR_STALL_TIMEOUT)
      {
        act.stall_latched = true;
        act.stall_latched_command = target;
        RCLCPP_WARN(
          act.logger,
          "%s LINEAR ACTUATOR: stall watchdog tripped (target=%.3f measured=%.3f) - "
          "holding stopped until target changes",
          label.c_str(),
          target,
          measured);
      }
    }

    if (act.stall_latched)
    {
      duty = 0.0;
    }

    send_actuator_duty(act, duty, label);
  }

  // Clamp/deadband a duty command and send it to one actuator's SPARK MAX.
  void send_actuator_duty(LinearActuatorHW & act, double duty, const std::string & label)
  {
    double safe_throttle =
      apply_throttle_deadband(clamp_throttle(duty), act.deadband);

    if (std::fabs(safe_throttle) <= act.deadband)
    {
      safe_throttle = 0.0;
    }

    RCLCPP_WARN_THROTTLE(
      act.logger,
      *rclcpp::Clock::make_shared(),
      500,
      "%s LINEAR ACTUATOR: id=%u target_pos=%.3f measured_pos=%.3f duty_sent=%.3f",
      label.c_str(),
      act.can_id,
      std::clamp(act.command, 0.0, 1.0),
      act.position,
      safe_throttle);

    const bool ok = act.spark->set_duty_cycle(
      static_cast<float>(safe_throttle),
      true);

    sleep_bus_gap();

    if (ok)
    {
      ++act.command_count;
      act.last_sent_output = safe_throttle;
      act.currently_commanded = std::fabs(safe_throttle) > 0.0;
    }
    else
    {
      RCLCPP_WARN_THROTTLE(
        act.logger,
        *rclcpp::Clock::make_shared(),
        1000,
        "Failed to send duty command to %s linear actuator on CAN ID %u",
        label.c_str(),
        act.can_id);
    }
  }
}
