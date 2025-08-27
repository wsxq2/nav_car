# NavCar Joystick Package

这个包提供了使用游戏手柄/摇杆来控制NavCar机器人的功能。

## 功能

- **PS3控制器支持**: 支持PS3无线手柄控制
- **Xbox控制器支持**: 支持Xbox无线手柄控制  
- **灵活配置**: 可配置按键映射和速度参数
- **安全控制**: 需要按住使能按钮才能控制机器人

## 支持的控制器

### PS3 控制器
- **使能按钮**: L1 (按住才能控制)
- **加速按钮**: R1 (按住获得更高速度)
- **移动控制**: 左摇杆
- **转向控制**: 左摇杆

### Xbox 控制器  
- **使能按钮**: LB (按住才能控制)
- **加速按钮**: RB (按住获得更高速度)
- **移动控制**: 左摇杆
- **转向控制**: 右摇杆

## 使用方法

### 1. 仅启动手柄控制

```bash
# 使用PS3控制器
ros2 launch navcar_joytick navcar_joystick.launch.py joy_config:=ps3

# 使用Xbox控制器  
ros2 launch navcar_joytick navcar_joystick.launch.py joy_config:=xbox

# 指定手柄设备路径
ros2 launch navcar_joytick navcar_joystick.launch.py joy_dev:=/dev/input/js1
```

### 2. 启动机器人和手柄控制

```bash
# 同时启动机器人和手柄控制
ros2 launch navcar_joytick navcar_joystick_bringup.launch.py

# 使用模拟硬件
ros2 launch navcar_joytick navcar_joystick_bringup.launch.py use_mock_hardware:=true

# 同时启动RViz
ros2 launch navcar_joytick navcar_joystick_bringup.launch.py start_rviz:=true
```

## 配置参数

### 速度配置
- `scale_linear`: 常规线速度缩放 (默认: 0.3)
- `scale_linear_turbo`: 加速线速度缩放 (默认: 0.6)
- `scale_angular`: 常规角速度缩放 (默认: 0.5) 
- `scale_angular_turbo`: 加速角速度缩放 (默认: 1.0)

### 按键配置
- `enable_button`: 使能按钮ID
- `enable_turbo_button`: 加速按钮ID
- `axis_linear`: 线速度控制轴
- `axis_angular`: 角速度控制轴

## 文件结构

```
navcar_joytick/
├── config/
│   ├── ps3_config.yaml         # PS3控制器配置
│   └── xbox_config.yaml        # Xbox控制器配置
├── launch/
│   ├── navcar_joystick.launch.py        # 仅手柄控制启动文件
│   └── navcar_joystick_bringup.launch.py # 机器人+手柄完整启动文件
└── README.md
```

## 故障排除

### 1. 找不到手柄设备

```bash
# 查看可用的输入设备
ls /dev/input/js*

# 查看设备信息
sudo dmesg | grep -i "input\|usb"
```

### 2. 手柄无响应

- 确保手柄已正确连接并配对
- 检查设备权限: `sudo chmod 666 /dev/input/js0`
- 测试手柄: `ros2 run joy joy_node`

### 3. 机器人不动

- 确保按住使能按钮 (L1/LB)
- 检查cmd_vel话题: `ros2 topic echo /mobile_base_controller/cmd_vel`
- 确保机器人控制器已启动

### 4. 速度太快/太慢

修改配置文件中的速度参数，或在启动时指定：

```bash
ros2 launch navcar_joytick navcar_joystick.launch.py \
  scale_linear:=0.2 scale_angular:=0.3
```

## 依赖

- joy: ROS2手柄驱动包
- teleop_twist_joy: ROS2手柄遥控包
- geometry_msgs: 几何消息类型
- navcar_bringup: NavCar机器人启动包

## 安全提示

- 使用前确保周围环境安全
- 始终准备按下急停按钮
- 建议先在开阔场地测试
- 控制时需要按住使能按钮，松开即停止
