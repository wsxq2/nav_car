# NavCar ROS2 Package

一个简单的ROS2导航车机器人包，包含描述文件和控制配置。

## 包结构

- `nav_car`: 主包（元包）
- `navcar_description`: 机器人描述包，包含URDF/Xacro文件
- `navcar_control`: 机器人控制包，包含ros2_control配置

## 快速开始

### 构建工作空间

```bash
cd /home/wsxq2/work/nav_car_ws
colcon build
source install/setup.bash
```

### 启动机器人

#### 1. 仅在RViz中查看机器人模型

```bash
ros2 launch navcar_description nav_car_rviz.launch.py
```

#### 2. 启动完整的控制系统

```bash
ros2 launch navcar_control navcar_control.launch.py
```

或者使用主launch文件：

```bash
ros2 launch nav_car nav_car.launch.py
```

### 控制机器人

启动控制系统后，可以使用以下命令控制机器人：

```bash
ros2 topic pub /mobile_base_controller/cmd_vel geometry_msgs/msg/Twist "
linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.3"
```

### 查看关节状态

```bash
ros2 topic echo /joint_states
```

## 机器人参数

- 轮距: 0.446m
- 轮半径: 0.16m
- 底盘尺寸: 0.69m × 0.315m × 0.102m

## 依赖项

确保已安装以下ROS2包：
- ros2_control
- ros2_controllers
- diff_drive_controller
- joint_state_broadcaster
- robot_state_publisher
