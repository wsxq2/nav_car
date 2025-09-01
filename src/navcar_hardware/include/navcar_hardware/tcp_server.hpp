#ifndef NAVCAR_HARDWARE__TCP_SERVER_HPP_
#define NAVCAR_HARDWARE__TCP_SERVER_HPP_

#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <functional>
#include <memory>
#include <cstdint>

namespace navcar_hardware
{

struct MotorState
{
  double left_velocity;   // rad/s
  double right_velocity;  // rad/s
  double left_position;   // rad
  double right_position;  // rad
};

struct MotorCommand
{
  double left_velocity;   // rad/s
  double right_velocity;  // rad/s
  bool reset_odom;
};

class TcpServer
{
public:
  explicit TcpServer(const std::string& host = "0.0.0.0", int port = 3011);
  ~TcpServer();

  bool start();
  void stop();
  bool is_connected() const;

  void set_motor_command(const MotorCommand& command);
  MotorState get_motor_state() const;

private:
  void server_thread();
  void client_thread(int client_socket);
  
  bool send_command(int socket, const MotorCommand& command);
  bool receive_state(int socket, MotorState& state);
  
  uint16_t calculate_crc16(const std::vector<uint8_t>& data) const;
  void log_info(const std::string& message) const;
  void log_error(const std::string& message) const;
  void log_debug(const std::string& message) const;

  std::string host_;
  int port_;
  int server_socket_;
  int client_socket_;
  
  std::atomic<bool> running_;
  std::atomic<bool> connected_;
  
  std::thread server_thread_;
  std::thread client_thread_;
  
  MotorState current_state_;
  MotorCommand current_command_;
  
  mutable std::mutex state_mutex_;
  mutable std::mutex command_mutex_;
};

} // namespace navcar_hardware

#endif // NAVCAR_HARDWARE__TCP_SERVER_HPP_
