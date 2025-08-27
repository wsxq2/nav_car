# NavCar Hardware Interface

这个包实现了NavCar机器人的ROS2 hardware interface，通过TCP通信与硬件控制器进行交互。

## 功能特性

- TCP服务器监听硬件连接 (默认端口3011)
- 自动处理网络连接和断线重连
- 支持两轮差动驱动控制
- 实时状态反馈和命令发送
- MODBUS RTU CRC16校验

## 通信协议

### 状态上报 (从硬件接收)
- 帧头: 0x55AA
- 命令: 0x02
- 包含左右电机转速和位置信息
- 单位: 转速(0.001rpm), 位置(0.001度)

### 运动控制 (发送到硬件)
- 帧头: 0x55AA  
- 命令: 0x01
- 包含左右电机转速命令
- 单位: 转速(0.001rpm)

## 配置参数

在URDF中可以配置以下参数:
- `tcp_host`: TCP服务器监听地址 (默认: "0.0.0.0")
- `tcp_port`: TCP服务器监听端口 (默认: 3011)
- `wheel_radius`: 车轮半径，用于单位转换 (默认: 0.16m)

## 使用方法

1. 编译包:
```bash
colcon build --packages-select navcar_hardware
```

2. 测试硬件接口:
```bash
ros2 launch navcar_hardware test_hardware.launch.py use_mock_hardware:=false
```

3. 发送速度命令:
```bash
ros2 topic pub /mobile_base_controller/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

## 故障排除

- 检查TCP端口是否被占用
- 确认硬件客户端能连接到服务器
- 查看日志输出了解连接状态
- 验证CRC校验是否正确

## 依赖项

- hardware_interface
- pluginlib  
- rclcpp
- rclcpp_lifecycle
