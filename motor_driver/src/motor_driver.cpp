#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <boost/asio.hpp>
#include <iostream>
#include <vector>
#include <cstring>
#include <diffbot_msgs/WheelsCmdStamped.h>
#include <sensor_msgs/JointState.h>

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
        crc ^= (uint16_t)buf[pos]; // XOR byte into least sig. byte of crc

        for (int i = 8; i != 0; i--) { // Loop over each bit
            if ((crc & 0x0001) != 0) { // If the LSB is set
                crc >>= 1; // Shift right and XOR 0xA001
                crc ^= 0xA001;
            } else // Else LSB is not set
                crc >>= 1; // Just shift right
        }
    }
    // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
    return crc;
}

int16_t rad_to_rpm(float rad) {
    int16_t rpm = rad * 60.0 / (2.0 * M_PI);
    return rpm;
}

float rpm_to_rad(int16_t rpm) {
    float rad = rpm * (2.0 * M_PI) / 60.0;
    return rad;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "motor_driver");
    ros::NodeHandle nh;

    ros::Publisher encoder_pub = nh.advertise<sensor_msgs::JointState>("measured_joint_states", 10);

    float left_cmd = 0, right_cmd = 0;

    auto wheel_cb = [&](const diffbot_msgs::WheelsCmdStamped::ConstPtr& msg) {
         left_cmd = msg->wheels_cmd.angular_velocities.joint[0];
         right_cmd = msg->wheels_cmd.angular_velocities.joint[1];
     };
    ros::Subscriber sub_wheel = nh.subscribe<diffbot_msgs::WheelsCmdStamped>("wheel_cmd_velocities", 10, wheel_cb);

    boost::asio::io_service io_service;
    tcp::acceptor acceptor(io_service, tcp::endpoint(boost::asio::ip::address::from_string(TCP_IP), TCP_PORT));

    std::vector<uint8_t> buffer;

    float dt = 0.02f; // 20ms
    ros::Rate rate(1/dt);

    float pos_left = 0, pos_right = 0;

    while (ros::ok()) {
        ROS_INFO("Waiting for TCP client on %s:%d...", TCP_IP, TCP_PORT);
        tcp::socket socket(io_service);
        boost::system::error_code ec;
        acceptor.accept(socket, ec);
        if (ec) {
            ROS_ERROR("Accept error: %s", ec.message().c_str());
            continue;
        }
        ROS_INFO("TCP client connected.");

        try {
            buffer.clear();
            socket.non_blocking(true); // 设置为非阻塞模式
            while (ros::ok() && socket.is_open()) {
                // 1. 读取数据到缓冲区
                uint8_t temp[64];
                size_t n = socket.read_some(boost::asio::buffer(temp), ec);
                if (ec) {
                    if (ec == boost::asio::error::would_block) {
                        continue;
                    }
                    else {
                        ROS_ERROR("Read error: %s", ec.message().c_str());
                        break;
                    }
                }
                buffer.insert(buffer.end(), temp, temp + n);

                // 2. 拆包处理
                while (buffer.size() >= RECV_FRAME_LEN) {
                    // 查找帧头
                    size_t pos = 0;
                    for (; pos + 1 < buffer.size(); ++pos) {
                        if (buffer[pos] == ((FRAME_HEAD >> 8) & 0xFF) && buffer[pos + 1] == (FRAME_HEAD & 0xFF))
                            break;
                    }
                    if (pos + RECV_FRAME_LEN > buffer.size()) {
                        if(buffer.size() > RECV_BUFFER_MAX_LEN) {
                            ROS_WARN("Buffer overflow, clearing buffer.");
                            buffer.clear();
                        }
                        break; // 不够一帧
                    }

                    // 检查帧
                    uint8_t* recv_buf = buffer.data() + pos;
                    uint16_t head = (recv_buf[0] << 8) | recv_buf[1];
                    uint16_t len = (recv_buf[2] << 8) | recv_buf[3];
                    uint8_t cmd = recv_buf[4];
                    int16_t motor_left = (recv_buf[5] << 8) | recv_buf[6];
                    int16_t motor_right = (recv_buf[7] << 8) | recv_buf[8];
                    uint16_t crc_recv = (recv_buf[9] << 8) | recv_buf[10];
                    uint16_t crc_calc = crc16_modbus_rtu(&recv_buf[0], RECV_FRAME_LEN - 2);

                    if (head == FRAME_HEAD && len == RECV_FRAME_LEN && cmd == CMD_STATUS_QUERY && crc_recv == crc_calc) {
                        sensor_msgs::JointState msg;
                        msg.header.stamp = ros::Time::now();
                        msg.name.resize(2);
                        msg.name[0] = "left_wheel";
                        msg.name[1] = "right_wheel";
                        msg.velocity.resize(2);
                        float rad_left = rpm_to_rad(motor_left);
                        float rad_right = rpm_to_rad(motor_right);
                        msg.velocity[0] = rad_left;
                        msg.velocity[1] = rad_right;
                        msg.position.resize(2);
                        pos_left += rad_left * dt;
                        pos_right += rad_right * dt;
                        msg.position[0] = pos_left;
                        msg.position[1] = pos_right;
                        encoder_pub.publish(msg);

                        // rad/s to rpm
                        int16_t left_rpm = rad_to_rpm(left_cmd);
                        int16_t right_rpm = rad_to_rpm(right_cmd);

                        // 发送运动控制帧
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
                        boost::asio::write(socket, boost::asio::buffer(send_buf, SEND_FRAME_LEN));
                    } else {
                        ROS_WARN("Invalid frame received: head=0x%04X, len=%d, cmd=0x%02X, crc_recv=0x%04X, crc_calc=0x%04X",
                                 head, len, cmd, crc_recv, crc_calc);
                    }
                    // 移除已处理数据
                    buffer.erase(buffer.begin(), buffer.begin() + pos + RECV_FRAME_LEN);
                }
                ros::spinOnce();
                rate.sleep(); // 增加这一行，降低CPU占用
            }
        } catch (std::exception& e) {
            ROS_ERROR("Socket exception: %s", e.what());
        }
        boost::system::error_code ignore_ec;
        socket.close(ignore_ec);
    }
    return 0;
}