#include "navcar_hardware/tcp_server.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <chrono>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace navcar_hardware
{

constexpr uint16_t FRAME_HEAD = 0x55AA;
constexpr uint8_t CMD_MOTION_CONTROL = 0x01;
constexpr uint8_t CMD_STATUS_REPORT = 0x02;
constexpr size_t STATUS_FRAME_SIZE = 23;
constexpr size_t CONTROL_FRAME_SIZE = 16;

TcpServer::TcpServer(const std::string& host, int port)
  : host_(host), port_(port), server_socket_(-1), client_socket_(-1),
    running_(false), connected_(false)
{
  current_state_ = {};
  current_command_ = {};
}

TcpServer::~TcpServer()
{
  stop();
}

bool TcpServer::start()
{
  if (running_)
  {
    log_error("TCP server is already running");
    return false;
  }

  running_ = true;
  server_thread_ = std::thread(&TcpServer::server_thread, this);
  
  log_info("TCP server starting on " + host_ + ":" + std::to_string(port_));
  return true;
}

void TcpServer::stop()
{
  running_ = false;
  connected_ = false;

  if (client_socket_ != -1)
  {
    close(client_socket_);
    client_socket_ = -1;
  }

  if (server_socket_ != -1)
  {
    close(server_socket_);
    server_socket_ = -1;
  }

  if (server_thread_.joinable())
  {
    server_thread_.join();
  }

  if (client_thread_.joinable())
  {
    client_thread_.join();
  }

  log_info("TCP server stopped");
}

bool TcpServer::is_connected() const
{
  return connected_;
}

void TcpServer::set_motor_command(const MotorCommand& command)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  current_command_ = command;
}

MotorState TcpServer::get_motor_state() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return current_state_;
}

void TcpServer::server_thread()
{
  while (running_)
  {
    // Create socket
    server_socket_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_socket_ < 0)
    {
      log_error("Failed to create socket");
      std::this_thread::sleep_for(std::chrono::seconds(1));
      continue;
    }

    // Set socket options
    int opt = 1;
    if (setsockopt(server_socket_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0)
    {
      log_error("Failed to set socket options");
    }

    // Bind socket
    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port_);
    if (inet_pton(AF_INET, host_.c_str(), &server_addr.sin_addr) <= 0)
    {
      server_addr.sin_addr.s_addr = INADDR_ANY;
    }

    if (bind(server_socket_, (sockaddr*)&server_addr, sizeof(server_addr)) < 0)
    {
      log_error("Failed to bind socket to " + host_ + ":" + std::to_string(port_));
      close(server_socket_);
      server_socket_ = -1;
      std::this_thread::sleep_for(std::chrono::seconds(1));
      continue;
    }

    // Listen
    if (listen(server_socket_, 1) < 0)
    {
      log_error("Failed to listen on socket");
      close(server_socket_);
      server_socket_ = -1;
      std::this_thread::sleep_for(std::chrono::seconds(1));
      continue;
    }

    log_info("TCP server listening on " + host_ + ":" + std::to_string(port_));

    // Accept connections
    while (running_)
    {
      sockaddr_in client_addr{};
      socklen_t client_addr_len = sizeof(client_addr);
      
      client_socket_ = accept(server_socket_, (sockaddr*)&client_addr, &client_addr_len);
      if (client_socket_ < 0)
      {
        if (running_)
        {
          log_error("Failed to accept client connection");
        }
        break;
      }

      char client_ip[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
      log_info("Client connected from " + std::string(client_ip));

      connected_ = true;
      
      // Start client handling thread
      if (client_thread_.joinable())
      {
        client_thread_.join();
      }
      client_thread_ = std::thread(&TcpServer::client_thread, this, client_socket_);
      
      // Wait for client thread to finish
      client_thread_.join();
      
      connected_ = false;
      close(client_socket_);
      client_socket_ = -1;
      log_info("Client disconnected");
    }

    close(server_socket_);
    server_socket_ = -1;
  }
}

void TcpServer::client_thread(int client_socket)
{
  MotorState state;
  
  while (running_ && connected_)
  {
    // Try to receive status from hardware
    if (!receive_state(client_socket, state))
    {
      log_error("Failed to receive state from hardware");
      break;
    }

    // Update current state
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_state_ = state;
    }

    // Send command to hardware
    MotorCommand command;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      command = current_command_;
    }

    if (!send_command(client_socket, command))
    {
      log_error("Failed to send command to hardware");
      break;
    }

    // Small delay to prevent busy waiting
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

bool TcpServer::send_command(int socket, const MotorCommand& command)
{
  std::vector<uint8_t> frame(CONTROL_FRAME_SIZE);
  
  // Convert velocities from rad/s to 0.001 rpm
  // rad/s to rpm: rad/s * 60 / (2*pi)
  // rpm to 0.001 rpm: rpm * 1000
  int32_t left_rpm_millis = static_cast<int32_t>(command.left_velocity * 60000.0 / (2.0 * M_PI));
  int32_t right_rpm_millis = static_cast<int32_t>(command.right_velocity * 60000.0 / (2.0 * M_PI));

  size_t offset = 0;
  
  // Frame header (big endian)
  frame[offset++] = (FRAME_HEAD >> 8) & 0xFF;
  frame[offset++] = FRAME_HEAD & 0xFF;
  
  // Frame length (big endian)
  frame[offset++] = (CONTROL_FRAME_SIZE >> 8) & 0xFF;
  frame[offset++] = CONTROL_FRAME_SIZE & 0xFF;
  
  // Command
  frame[offset++] = CMD_MOTION_CONTROL;
  
  // Left motor speed (big endian)
  frame[offset++] = (left_rpm_millis >> 24) & 0xFF;
  frame[offset++] = (left_rpm_millis >> 16) & 0xFF;
  frame[offset++] = (left_rpm_millis >> 8) & 0xFF;
  frame[offset++] = left_rpm_millis & 0xFF;
  
  // Right motor speed (big endian)
  frame[offset++] = (right_rpm_millis >> 24) & 0xFF;
  frame[offset++] = (right_rpm_millis >> 16) & 0xFF;
  frame[offset++] = (right_rpm_millis >> 8) & 0xFF;
  frame[offset++] = right_rpm_millis & 0xFF;
  
  // Reset odometry flag
  frame[offset++] = command.reset_odom ? 1 : 0;
  
  // Calculate CRC16 (excluding header and CRC itself)
  std::vector<uint8_t> crc_data(frame.begin() + 2, frame.begin() + offset);
  uint16_t crc = calculate_crc16(crc_data);
  
  // CRC16 (big endian)
  frame[offset++] = (crc >> 8) & 0xFF;
  frame[offset++] = crc & 0xFF;

  // Send frame
  ssize_t bytes_sent = send(socket, frame.data(), frame.size(), 0);
  return bytes_sent == static_cast<ssize_t>(frame.size());
}

bool TcpServer::receive_state(int socket, MotorState& state)
{
  std::vector<uint8_t> frame(STATUS_FRAME_SIZE);
  
  // Receive complete frame
  size_t bytes_received = 0;
  while (bytes_received < STATUS_FRAME_SIZE && running_)
  {
    ssize_t result = recv(socket, frame.data() + bytes_received, 
                         STATUS_FRAME_SIZE - bytes_received, 0);
    if (result <= 0)
    {
      return false;
    }
    bytes_received += result;
  }

  // Parse frame
  size_t offset = 0;
  
  // Check frame header
  uint16_t header = (static_cast<uint16_t>(frame[offset]) << 8) | frame[offset + 1];
  offset += 2;
  if (header != FRAME_HEAD)
  {
    log_error("Invalid frame header: 0x" + std::to_string(header));
    return false;
  }
  
  // Check frame length
  uint16_t length = (static_cast<uint16_t>(frame[offset]) << 8) | frame[offset + 1];
  offset += 2;
  if (length != STATUS_FRAME_SIZE)
  {
    log_error("Invalid frame length: " + std::to_string(length));
    return false;
  }
  
  // Check command
  uint8_t cmd = frame[offset++];
  if (cmd != CMD_STATUS_REPORT)
  {
    log_error("Invalid command: " + std::to_string(cmd));
    return false;
  }
  
  // Left motor speed (0.001 rpm, big endian)
  int32_t left_rpm_millis = (static_cast<int32_t>(frame[offset]) << 24) |
                           (static_cast<int32_t>(frame[offset + 1]) << 16) |
                           (static_cast<int32_t>(frame[offset + 2]) << 8) |
                           static_cast<int32_t>(frame[offset + 3]);
  offset += 4;
  
  // Right motor speed (0.001 rpm, big endian)
  int32_t right_rpm_millis = (static_cast<int32_t>(frame[offset]) << 24) |
                            (static_cast<int32_t>(frame[offset + 1]) << 16) |
                            (static_cast<int32_t>(frame[offset + 2]) << 8) |
                            static_cast<int32_t>(frame[offset + 3]);
  offset += 4;
  
  // Left motor position (0.001 degrees, big endian)
  int32_t left_pos_millideg = (static_cast<int32_t>(frame[offset]) << 24) |
                             (static_cast<int32_t>(frame[offset + 1]) << 16) |
                             (static_cast<int32_t>(frame[offset + 2]) << 8) |
                             static_cast<int32_t>(frame[offset + 3]);
  offset += 4;
  
  // Right motor position (0.001 degrees, big endian)
  int32_t right_pos_millideg = (static_cast<int32_t>(frame[offset]) << 24) |
                              (static_cast<int32_t>(frame[offset + 1]) << 16) |
                              (static_cast<int32_t>(frame[offset + 2]) << 8) |
                              static_cast<int32_t>(frame[offset + 3]);
  offset += 4;
  
  // Verify CRC16
  uint16_t received_crc = (static_cast<uint16_t>(frame[offset]) << 8) | frame[offset + 1];
  std::vector<uint8_t> crc_data(frame.begin() + 2, frame.begin() + offset);
  uint16_t calculated_crc = calculate_crc16(crc_data);
  
  if (received_crc != calculated_crc)
  {
    log_error("CRC mismatch: received=0x" + std::to_string(received_crc) + 
              ", calculated=0x" + std::to_string(calculated_crc));
    return false;
  }

  // Convert units and update state
  // 0.001 rpm to rad/s: (rpm/1000) * (2*pi) / 60
  state.left_velocity = left_rpm_millis * (2.0 * M_PI) / 60000.0;
  state.right_velocity = right_rpm_millis * (2.0 * M_PI) / 60000.0;
  
  // 0.001 degrees to radians: (deg/1000) * pi / 180
  state.left_position = left_pos_millideg * M_PI / 180000.0;
  state.right_position = right_pos_millideg * M_PI / 180000.0;

  return true;
}

uint16_t TcpServer::calculate_crc16(const std::vector<uint8_t>& data) const
{
  uint16_t crc = 0xFFFF;
  
  for (uint8_t byte : data)
  {
    crc ^= byte;
    
    for (int i = 0; i < 8; i++)
    {
      if (crc & 0x0001)
      {
        crc >>= 1;
        crc ^= 0xA001;  // MODBUS RTU polynomial
      }
      else
      {
        crc >>= 1;
      }
    }
  }
  
  return crc;
}

void TcpServer::log_info(const std::string& message) const
{
  std::cout << "[NavCar TCP] INFO: " << message << std::endl;
}

void TcpServer::log_error(const std::string& message) const
{
  std::cerr << "[NavCar TCP] ERROR: " << message << std::endl;
}

} // namespace navcar_hardware
