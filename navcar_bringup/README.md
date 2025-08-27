# NavCar Bringup Package

This package provides launch files for bringing up the NavCar robot in different configurations.

## Launch Files

### 1. Complete Bringup
启动完整的机器人系统，包括硬件控制器和RViz可视化：
```bash
ros2 launch navcar_bringup navcar_bringup.launch.py
```

参数：
- `use_mock_hardware`: 使用模拟硬件（默认: true）
- `start_rviz`: 启动RViz（默认: true）

### 2. Robot Only
仅启动机器人控制器，不启动RViz：
```bash
ros2 launch navcar_bringup robot.launch.py
```

参数：
- `use_mock_hardware`: 使用模拟硬件（默认: true）

### 3. Display Only
仅启动机器人描述和RViz可视化，不启动控制器：
```bash
ros2 launch navcar_bringup display.launch.py
```

参数：
- `description_package`: 描述包名（默认: navcar_description）
- `description_file`: URDF文件名（默认: nav_car.urdf.xacro）
- `use_ros2_control`: 是否包含ros2_control（默认: false）
- `use_mock_hardware`: 使用模拟硬件（默认: true）

## 使用示例

### 开发和调试
```bash
# 仅查看机器人模型
ros2 launch navcar_bringup display.launch.py

# 启动机器人控制器进行测试
ros2 launch navcar_bringup robot.launch.py use_mock_hardware:=true
```

### 完整运行
```bash
# 启动完整系统
ros2 launch navcar_bringup navcar_bringup.launch.py

# 使用真实硬件
ros2 launch navcar_bringup navcar_bringup.launch.py use_mock_hardware:=false
```

## 控制机器人

启动后，可以通过以下话题控制机器人：
```bash
# 发送速度命令
ros2 topic pub /mobile_base_controller/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"

# 查看关节状态
ros2 topic echo /joint_states

# 查看里程计信息
ros2 topic echo /mobile_base_controller/odom
```
