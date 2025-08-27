#ifndef NAVCAR_HARDWARE__NAVCAR_HARDWARE_HPP_
#define NAVCAR_HARDWARE__NAVCAR_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "navcar_hardware/tcp_server.hpp"

namespace navcar_hardware
{
class NavCarHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(NavCarHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // TCP server for communication
  std::unique_ptr<TcpServer> tcp_server_;

  // Store the command and state values for the wheels
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  // Configuration parameters
  std::string tcp_host_;
  int tcp_port_;
  double wheel_radius_;
  
  // Joint names
  std::vector<std::string> joint_names_;
};

}  // namespace navcar_hardware

#endif  // NAVCAR_HARDWARE__NAVCAR_HARDWARE_HPP_
