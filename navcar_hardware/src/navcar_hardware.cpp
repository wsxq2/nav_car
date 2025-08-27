#include "navcar_hardware/navcar_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace navcar_hardware
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("NavCarHardware");

hardware_interface::CallbackReturn NavCarHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get TCP configuration
  tcp_host_ = info_.hardware_parameters.find("tcp_host") != info_.hardware_parameters.end() 
              ? info_.hardware_parameters["tcp_host"] : "0.0.0.0";
  tcp_port_ = info_.hardware_parameters.find("tcp_port") != info_.hardware_parameters.end()
              ? std::stoi(info_.hardware_parameters["tcp_port"]) : 3011;
  
  // Get wheel radius
  wheel_radius_ = info_.hardware_parameters.find("wheel_radius") != info_.hardware_parameters.end()
                  ? std::stod(info_.hardware_parameters["wheel_radius"]) : 0.16;

  // Initialize hardware states and commands
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);

  // Store joint names
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    joint_names_.push_back(joint.name);
  }

  // Create TCP server
  tcp_server_ = std::make_unique<TcpServer>(tcp_host_, tcp_port_);

  RCLCPP_INFO(LOGGER, "NavCar hardware interface initialized");
  RCLCPP_INFO(LOGGER, "TCP server configured for %s:%d", tcp_host_.c_str(), tcp_port_);
  RCLCPP_INFO(LOGGER, "Wheel radius: %.3f m", wheel_radius_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> NavCarHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> NavCarHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn NavCarHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(LOGGER, "Activating hardware interface...");

  // Start TCP server
  if (!tcp_server_->start())
  {
    RCLCPP_ERROR(LOGGER, "Failed to start TCP server");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize commands to zero
  for (auto i = 0u; i < hw_commands_.size(); i++)
  {
    if (std::isnan(hw_commands_[i]))
    {
      hw_commands_[i] = 0;
    }
  }

  RCLCPP_INFO(LOGGER, "Hardware interface activated successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn NavCarHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(LOGGER, "Deactivating hardware interface...");

  // Stop TCP server with timeout
  if (tcp_server_)
  {
    try
    {
      tcp_server_->stop();
      RCLCPP_INFO(LOGGER, "TCP server stopped successfully");
    }
    catch (const std::exception& e)
    {
      RCLCPP_WARN(LOGGER, "Exception during TCP server stop: %s", e.what());
    }
  }

  RCLCPP_INFO(LOGGER, "Hardware interface deactivated successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type NavCarHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!tcp_server_->is_connected())
  {
    // If not connected, keep last known values
    return hardware_interface::return_type::OK;
  }

  // Get motor state from TCP server
  MotorState state = tcp_server_->get_motor_state();

  // Update hardware state based on joint order
  for (auto i = 0u; i < joint_names_.size(); i++)
  {
    if (joint_names_[i].find("left") != std::string::npos)
    {
      hw_velocities_[i] = state.left_velocity;
      hw_positions_[i] = state.left_position;
    }
    else if (joint_names_[i].find("right") != std::string::npos)
    {
      hw_velocities_[i] = state.right_velocity;
      hw_positions_[i] = state.right_position;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type NavCarHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!tcp_server_->is_connected())
  {
    // If not connected, commands cannot be sent
    return hardware_interface::return_type::OK;
  }

  MotorCommand command;
  command.reset_odom = false;  // Usually false, could be made configurable

  // Map joint commands to motor commands based on joint names
  for (auto i = 0u; i < joint_names_.size(); i++)
  {
    if (joint_names_[i].find("left") != std::string::npos)
    {
      command.left_velocity = hw_commands_[i];
    }
    else if (joint_names_[i].find("right") != std::string::npos)
    {
      command.right_velocity = hw_commands_[i];
    }
  }

  // Send command to hardware via TCP
  tcp_server_->set_motor_command(command);

  return hardware_interface::return_type::OK;
}

}  // namespace navcar_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  navcar_hardware::NavCarHardware, hardware_interface::SystemInterface)
