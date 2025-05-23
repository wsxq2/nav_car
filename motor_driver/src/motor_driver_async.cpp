#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <boost/asio.hpp>
#include <iostream>
#include <vector>
#include <cstring>
#include <diffbot_msgs/WheelsCmdStamped.h>
#include <sensor_msgs/JointState.h>
#include <mutex>

#define TCP_IP "0.0.0.0"
#define TCP_PORT 3011
#define FRAME_HEAD 0x55AA
#define CMD_STATUS_QUERY 0x02
#define CMD_MOTION_CTRL  0x01
#define RECV_FRAME_LEN 11
#define SEND_FRAME_LEN 11
#define RECV_BUFFER_MAX_LEN 4096

using boost::asio::ip::tcp;

uint16_t crc16_modbus_rtu(const uint8_t* buf, uint16_t buflen)
{
    uint16_t crc = 0xFFFF;
    for (int pos = 0; pos < buflen; pos++) {
        crc ^= (uint16_t)buf[pos];
        for (int i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else
                crc >>= 1;
        }
    }
    return crc;
}

int16_t rad_to_rpm(float rad) {
    return static_cast<int16_t>(rad * 60.0 / (2.0 * M_PI));
}

float rpm_to_rad(int16_t rpm) {
    return rpm * (2.0 * M_PI) / 60.0;
}

class MotorDriverAsync {
public:
    MotorDriverAsync(ros::NodeHandle& nh)
        : io_service_(),
          acceptor_(io_service_, tcp::endpoint(boost::asio::ip::address::from_string(TCP_IP), TCP_PORT)),
          socket_(io_service_),
          buffer_(),
          pos_left_(0), pos_right_(0),
          left_cmd_(0), right_cmd_(0)
    {
        encoder_pub_ = nh.advertise<sensor_msgs::JointState>("measured_joint_states", 10);
        sub_wheel_ = nh.subscribe<diffbot_msgs::WheelsCmdStamped>("wheel_cmd_velocities", 10,
            &MotorDriverAsync::wheelCmdCallback, this);
        last_time_ = ros::Time::now();
        start_accept();
        io_thread_ = std::thread([this]() { io_service_.run(); });
    }

    ~MotorDriverAsync() {
        io_service_.stop();
        if (io_thread_.joinable()) io_thread_.join();
    }

private:
    // 添加互斥锁
    std::mutex cmd_mutex_;

    void wheelCmdCallback(const diffbot_msgs::WheelsCmdStamped::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        left_cmd_ = msg->wheels_cmd.angular_velocities.joint[0];
        right_cmd_ = msg->wheels_cmd.angular_velocities.joint[1];
    }

    void start_accept() {
        ROS_INFO("Waiting for TCP client on %s:%d...", TCP_IP, TCP_PORT);
        acceptor_.async_accept(socket_,
            [this](const boost::system::error_code& ec) {
                if (!ec) {
                    ROS_INFO("TCP client connected.");
                    buffer_.clear();
                    start_read();
                } else {
                    ROS_ERROR("Accept error: %s", ec.message().c_str());
                    start_accept();
                }
            });
    }

    void start_read() {
        socket_.async_read_some(boost::asio::buffer(temp_buf_),
            [this](const boost::system::error_code& ec, std::size_t n) {
                if (!ec) {
                    buffer_.insert(buffer_.end(), temp_buf_, temp_buf_ + n);
                    process_buffer();
                    start_read();
                } else {
                    ROS_ERROR("Read error: %s", ec.message().c_str());
                    socket_.close();
                    start_accept();
                }
            });
    }

    void process_buffer() {
        while (buffer_.size() >= RECV_FRAME_LEN) {
            // 查找帧头（大端）
            size_t pos = 0;
            for (; pos + 1 < buffer_.size(); ++pos) {
                if (buffer_[pos] == ((FRAME_HEAD >> 8) & 0xFF) && buffer_[pos + 1] == (FRAME_HEAD & 0xFF))
                    break;
            }
            if (pos + RECV_FRAME_LEN > buffer_.size()) {
                if (buffer_.size() > RECV_BUFFER_MAX_LEN) {
                    ROS_WARN("Buffer overflow, clearing buffer.");
                    buffer_.clear();
                }
                break;
            }

            uint8_t* recv_buf = buffer_.data() + pos;
            uint16_t head = (recv_buf[0] << 8) | recv_buf[1];
            uint16_t len = (recv_buf[2] << 8) | recv_buf[3];
            uint8_t cmd = recv_buf[4];
            int16_t motor_left = (recv_buf[5] << 8) | recv_buf[6];
            int16_t motor_right = (recv_buf[7] << 8) | recv_buf[8];
            uint16_t crc_recv = (recv_buf[9] << 8) | recv_buf[10];
            uint16_t crc_calc = crc16_modbus_rtu(&recv_buf[0], RECV_FRAME_LEN - 2);

            if (head == FRAME_HEAD && len == RECV_FRAME_LEN && cmd == CMD_STATUS_QUERY && crc_recv == crc_calc) {
                // 计算实际dt
                ros::Time now = ros::Time::now();
                double dt = (now - last_time_).toSec();
                if (dt <= 0.0) {
                    ROS_WARN("Invalid dt: %f, using default value", dt);
                    dt = 0.02; // 防止异常
                }
                last_time_ = now;

                sensor_msgs::JointState msg;
                msg.header.stamp = now;
                msg.name.resize(2);
                msg.name[0] = "left_wheel";
                msg.name[1] = "right_wheel";
                msg.velocity.resize(2);
                float rad_left = rpm_to_rad(motor_left);
                float rad_right = rpm_to_rad(motor_right);
                msg.velocity[0] = rad_left;
                msg.velocity[1] = rad_right;
                msg.position.resize(2);
                pos_left_ += rad_left * dt;
                pos_right_ += rad_right * dt;
                msg.position[0] = pos_left_;
                msg.position[1] = pos_right_;
                encoder_pub_.publish(msg);

                // 读取命令时加锁
                float left_cmd, right_cmd;
                {
                    std::lock_guard<std::mutex> lock(cmd_mutex_);
                    left_cmd = left_cmd_;
                    right_cmd = right_cmd_;
                }

                // 发送运动控制帧
                int16_t left_rpm = rad_to_rpm(left_cmd);
                int16_t right_rpm = rad_to_rpm(right_cmd);
                uint8_t send_buf[SEND_FRAME_LEN] = {0};
                send_buf[0] = (FRAME_HEAD >> 8) & 0xFF;
                send_buf[1] = FRAME_HEAD & 0xFF;
                send_buf[2] = (SEND_FRAME_LEN >> 8) & 0xFF;
                send_buf[3] = SEND_FRAME_LEN & 0xFF;
                send_buf[4] = CMD_MOTION_CTRL;
                send_buf[5] = (left_rpm >> 8) & 0xFF;
                send_buf[6] = left_rpm & 0xFF;
                send_buf[7] = (right_rpm >> 8) & 0xFF;
                send_buf[8] = right_rpm & 0xFF;
                uint16_t crc = crc16_modbus_rtu(&send_buf[0], SEND_FRAME_LEN - 2);
                send_buf[9] = (crc >> 8) & 0xFF;
                send_buf[10] = crc & 0xFF;
                async_write(send_buf, SEND_FRAME_LEN);
            } else {
                ROS_WARN("Invalid frame received: head=0x%04X, len=%d, cmd=0x%02X, crc_recv=0x%04X, crc_calc=0x%04X",
                         head, len, cmd, crc_recv, crc_calc);
            }
            // 移除已处理数据
            buffer_.erase(buffer_.begin(), buffer_.begin() + pos + RECV_FRAME_LEN);
        }
    }

    void async_write(const uint8_t* data, size_t len) {
        auto buf = std::make_shared<std::vector<uint8_t>>(data, data + len);
        boost::asio::async_write(socket_, boost::asio::buffer(*buf),
            [buf](const boost::system::error_code& ec, std::size_t /*n*/) {
                if (ec) {
                    ROS_ERROR("Write error: %s", ec.message().c_str());
                }
            });
    }

    boost::asio::io_service io_service_;
    tcp::acceptor acceptor_;
    tcp::socket socket_;
    std::vector<uint8_t> buffer_;
    uint8_t temp_buf_[64];

    ros::Publisher encoder_pub_;
    ros::Subscriber sub_wheel_;

    float pos_left_, pos_right_;
    float left_cmd_, right_cmd_;
    ros::Time last_time_;
    std::thread io_thread_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "motor_driver_async");
    ros::NodeHandle nh;
    MotorDriverAsync driver(nh);
    ros::spin(); // 使用ROS自带的spin
    return 0;
}