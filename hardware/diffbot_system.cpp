#include "diffdrive_canbus/diffbot_system.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diffdrive_canbus
{

hardware_interface::CallbackReturn DiffDriveCANBusHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto logger = rclcpp::get_logger("DiffDriveCANBusHardware");

  auto get_string_param = [&](const std::string & key) -> std::string
  {
    const auto it = info_.hardware_parameters.find(key);
    if (it == info_.hardware_parameters.end()) {
      throw std::runtime_error("Missing required hardware parameter: " + key);
    }
    return it->second;
  };

  auto get_int_param = [&](const std::string & key) -> int
  {
    const auto it = info_.hardware_parameters.find(key);
    if (it == info_.hardware_parameters.end()) {
      throw std::runtime_error("Missing required hardware parameter: " + key);
    }

    try {
      return std::stoi(it->second);
    } catch (const std::exception &) {
      throw std::runtime_error(
        "Invalid integer for hardware parameter '" + key + "': '" + it->second + "'");
    }
  };

  try
  {
    cfg_.front_left_wheel_name  = get_string_param("front_left_wheel_name");
    cfg_.front_right_wheel_name = get_string_param("front_right_wheel_name");
    cfg_.rear_left_wheel_name   = get_string_param("rear_left_wheel_name");
    cfg_.rear_right_wheel_name  = get_string_param("rear_right_wheel_name");

    cfg_.serial_device     = get_string_param("serial_device");
    cfg_.serial_baud_rate  = get_int_param("serial_baud_rate");
    cfg_.can_baud_rate     = get_int_param("can_baud_rate");
    cfg_.timeout_ms        = get_int_param("timeout_ms");

    cfg_.front_left_can_id  = get_int_param("front_left_can_id");
    cfg_.front_right_can_id = get_int_param("front_right_can_id");
    cfg_.rear_left_can_id   = get_int_param("rear_left_can_id");
    cfg_.rear_right_can_id  = get_int_param("rear_right_can_id");

    cfg_.enc_counts_per_rev = get_int_param("enc_counts_per_rev");
  }
  catch (const std::exception & e)
  {
    RCLCPP_FATAL(logger, "%s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (cfg_.enc_counts_per_rev <= 0)
  {
    RCLCPP_FATAL(logger, "enc_counts_per_rev must be > 0, got %d", cfg_.enc_counts_per_rev);
    return hardware_interface::CallbackReturn::ERROR;
  }

  wheel_fl_.setup(cfg_.front_left_wheel_name, cfg_.enc_counts_per_rev);
  wheel_fr_.setup(cfg_.front_right_wheel_name, cfg_.enc_counts_per_rev);
  wheel_rl_.setup(cfg_.rear_left_wheel_name, cfg_.enc_counts_per_rev);
  wheel_rr_.setup(cfg_.rear_right_wheel_name, cfg_.enc_counts_per_rev);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        logger,
        "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        logger,
        "Joint '%s' has command interface '%s'. Expected '%s'.",
        joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        logger,
        "Joint '%s' has %zu state interfaces found. 2 expected.",
        joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        logger,
        "Joint '%s' has first state interface '%s'. Expected '%s'.",
        joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        logger,
        "Joint '%s' has second state interface '%s'. Expected '%s'.",
        joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(logger, "DiffDriveCANBusHardware successfully initialised.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
DiffDriveCANBusHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(
    wheel_fl_.name, hardware_interface::HW_IF_POSITION, &wheel_fl_.pos);
  state_interfaces.emplace_back(
    wheel_fl_.name, hardware_interface::HW_IF_VELOCITY, &wheel_fl_.vel);

  state_interfaces.emplace_back(
    wheel_fr_.name, hardware_interface::HW_IF_POSITION, &wheel_fr_.pos);
  state_interfaces.emplace_back(
    wheel_fr_.name, hardware_interface::HW_IF_VELOCITY, &wheel_fr_.vel);

  state_interfaces.emplace_back(
    wheel_rl_.name, hardware_interface::HW_IF_POSITION, &wheel_rl_.pos);
  state_interfaces.emplace_back(
    wheel_rl_.name, hardware_interface::HW_IF_VELOCITY, &wheel_rl_.vel);

  state_interfaces.emplace_back(
    wheel_rr_.name, hardware_interface::HW_IF_POSITION, &wheel_rr_.pos);
  state_interfaces.emplace_back(
    wheel_rr_.name, hardware_interface::HW_IF_VELOCITY, &wheel_rr_.vel);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
DiffDriveCANBusHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(
    wheel_fl_.name, hardware_interface::HW_IF_VELOCITY, &wheel_fl_.cmd);
  command_interfaces.emplace_back(
    wheel_fr_.name, hardware_interface::HW_IF_VELOCITY, &wheel_fr_.cmd);
  command_interfaces.emplace_back(
    wheel_rl_.name, hardware_interface::HW_IF_VELOCITY, &wheel_rl_.cmd);
  command_interfaces.emplace_back(
    wheel_rr_.name, hardware_interface::HW_IF_VELOCITY, &wheel_rr_.cmd);

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffDriveCANBusHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = rclcpp::get_logger("DiffDriveCANBusHardware");
  RCLCPP_INFO(logger, "Configuring hardware interface...");

  try
  {
    if (comms_.connected())
    {
      comms_.disconnect();
    }

    comms_.connect(cfg_.serial_device, cfg_.serial_baud_rate, cfg_.timeout_ms);

    if (!comms_.configure_adapter(cfg_.can_baud_rate))
    {
      RCLCPP_ERROR(logger, "Failed to configure CAN adapter.");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(logger, "Exception during configure: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger, "Hardware interface configured successfully.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveCANBusHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = rclcpp::get_logger("DiffDriveCANBusHardware");
  RCLCPP_INFO(logger, "Cleaning up hardware interface...");

  if (comms_.connected())
  {
    comms_.disconnect();
  }

  RCLCPP_INFO(logger, "Cleanup complete.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveCANBusHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = rclcpp::get_logger("DiffDriveCANBusHardware");
  RCLCPP_INFO(logger, "Activating hardware interface...");

  if (!comms_.connected())
  {
    RCLCPP_ERROR(logger, "Cannot activate: CAN adapter is not connected.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  wheel_fl_.cmd = 0.0;
  wheel_fr_.cmd = 0.0;
  wheel_rl_.cmd = 0.0;
  wheel_rr_.cmd = 0.0;

  wheel_fl_.vel = 0.0;
  wheel_fr_.vel = 0.0;
  wheel_rl_.vel = 0.0;
  wheel_rr_.vel = 0.0;

  RCLCPP_INFO(logger, "Hardware interface activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveCANBusHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = rclcpp::get_logger("DiffDriveCANBusHardware");
  RCLCPP_INFO(logger, "Deactivating hardware interface...");

  if (comms_.connected())
  {
    // Send zero commands on deactivate for safety
    (void)comms_.write_motor_command(cfg_.front_left_can_id, 0);
    (void)comms_.write_motor_command(cfg_.front_right_can_id, 0);
    (void)comms_.write_motor_command(cfg_.rear_left_can_id, 0);
    (void)comms_.write_motor_command(cfg_.rear_right_can_id, 0);
  }

  RCLCPP_INFO(logger, "Hardware interface deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveCANBusHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  auto update_wheel_from_encoder = [&](Wheel & wheel, int can_id) -> bool
  {
    int enc = 0;
    if (!comms_.read_motor_encoder(can_id, enc))
    {
      return false;
    }

    const double previous_pos = wheel.pos;
    wheel.enc = enc;
    wheel.pos = wheel.calc_enc_angle();

    const double dt = period.seconds();
    if (dt > 0.0)
    {
      wheel.vel = (wheel.pos - previous_pos) / dt;
    }
    else
    {
      wheel.vel = 0.0;
    }

    return true;
  };

  if (!update_wheel_from_encoder(wheel_fl_, cfg_.front_left_can_id)) {
    return hardware_interface::return_type::ERROR;
  }
  if (!update_wheel_from_encoder(wheel_fr_, cfg_.front_right_can_id)) {
    return hardware_interface::return_type::ERROR;
  }
  if (!update_wheel_from_encoder(wheel_rl_, cfg_.rear_left_can_id)) {
    return hardware_interface::return_type::ERROR;
  }
  if (!update_wheel_from_encoder(wheel_rr_, cfg_.rear_right_can_id)) {
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffDriveCANBusHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  auto rad_s_to_counts_per_cycle = [](const Wheel & wheel, double cmd_rad_s) -> int
  {
    if (wheel.rads_per_count <= 0.0)
    {
      return 0;
    }

    // This is a placeholder scaling.
    // Replace with your actual motor-controller command units if needed.
    return static_cast<int>(std::round(cmd_rad_s / wheel.rads_per_count));
  };

  const int fl_cmd = rad_s_to_counts_per_cycle(wheel_fl_, wheel_fl_.cmd);
  const int fr_cmd = rad_s_to_counts_per_cycle(wheel_fr_, wheel_fr_.cmd);
  const int rl_cmd = rad_s_to_counts_per_cycle(wheel_rl_, wheel_rl_.cmd);
  const int rr_cmd = rad_s_to_counts_per_cycle(wheel_rr_, wheel_rr_.cmd);

  if (!comms_.write_motor_command(cfg_.front_left_can_id, fl_cmd)) {
    return hardware_interface::return_type::ERROR;
  }
  if (!comms_.write_motor_command(cfg_.front_right_can_id, fr_cmd)) {
    return hardware_interface::return_type::ERROR;
  }
  if (!comms_.write_motor_command(cfg_.rear_left_can_id, rl_cmd)) {
    return hardware_interface::return_type::ERROR;
  }
  if (!comms_.write_motor_command(cfg_.rear_right_can_id, rr_cmd)) {
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace diffdrive_canbus

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  diffdrive_canbus::DiffDriveCANBusHardware,
  hardware_interface::SystemInterface)