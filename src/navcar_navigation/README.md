# NavCar Navigation

这个包提供了基于Nav2框架的NavCar机器人导航功能。

## 包含的文件

### Launch文件

- `navigation_launch.py` - 完整的导航launch文件，包含地图服务器、AMCL定位和导航
- `localization_launch.py` - 仅启动AMCL定位和地图服务器
- `nav_only_launch.py` - 仅启动导航功能（假设定位已经运行）
- `rviz_launch.py` - 启动RViz进行可视化

### 配置文件

- `nav2_params.yaml` - Nav2的主要参数配置文件
- `map.yaml` - 地图配置文件示例

## 使用方法

### 1. 完整导航启动

启动完整的导航系统（包含地图、定位和导航）：

```bash
ros2 launch navcar_navigation navigation_launch.py
```

可选参数：
- `map:=/path/to/your/map.yaml` - 指定地图文件路径
- `params_file:=/path/to/your/params.yaml` - 指定参数文件
- `use_sim_time:=true` - 使用仿真时间

### 2. 分离式启动

如果您想分别启动定位和导航：

启动定位：
```bash
ros2 launch navcar_navigation localization_launch.py map:=/path/to/your/map.yaml
```

启动导航：
```bash
ros2 launch navcar_navigation nav_only_launch.py
```

### 3. 可视化

启动RViz进行可视化：
```bash
ros2 launch navcar_navigation rviz_launch.py
```

## 配置说明

### 机器人参数

在`nav2_params.yaml`中，您可能需要根据您的机器人调整以下参数：

- `robot_radius`: 机器人半径（当前设置为0.22m）
- `max_vel_x`: 最大线速度（当前设置为0.26m/s）
- `max_vel_theta`: 最大角速度（当前设置为1.0rad/s）
- 激光雷达话题：`scan_topic: scan`

### 坐标系

确保以下坐标系设置正确：
- `base_frame_id: "base_footprint"`
- `global_frame_id: "map"`
- `odom_frame_id: "odom"`
- `robot_base_frame: base_link`

## 依赖

这个包依赖以下ROS2包：
- nav2_bringup
- nav2_bt_navigator
- nav2_controller
- nav2_planner
- nav2_recoveries
- nav2_lifecycle_manager
- nav2_map_server
- nav2_amcl
- 其他nav2相关包

## 故障排除

1. 确保您的激光雷达发布到`/scan`话题
2. 确保您的机器人发布正确的TF变换
3. 检查地图文件路径是否正确
4. 确保机器人的参数（半径、速度限制等）与实际机器人匹配

## 自定义

您可以通过修改`nav2_params.yaml`文件来自定义导航行为：
- 调整成本地图参数
- 修改规划器参数
- 调整控制器参数
- 配置行为参数
