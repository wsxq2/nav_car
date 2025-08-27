# NavCar SLAM Package

这个包提供了基于Cartographer的SLAM功能，用于NavCar机器人的建图和定位。

## 功能

- **建图 (SLAM)**: 使用Cartographer进行实时建图
- **定位 (Localization)**: 在已知地图中进行定位
- **地图保存**: 提供便捷的地图保存工具

## 文件结构

```
navcar_slam/
├── config/
│   ├── diffbot_lds_2d.lua              # SLAM配置文件
│   └── diffbot_lds_2d_localization.lua # 定位配置文件
├── launch/
│   ├── slam.launch.py                  # SLAM启动文件
│   └── localization.launch.py         # 定位启动文件
├── rviz/
│   ├── slam.rviz                       # SLAM RViz配置
│   └── localization.rviz               # 定位RViz配置
└── scripts/
    └── save_map.py                     # 地图保存脚本
```

## 使用方法

### 1. SLAM建图

启动SLAM进行建图：

```bash
ros2 launch navcar_slam slam.launch.py
```

可选参数：
- `use_mock_hardware:=true/false` - 是否使用模拟硬件（默认：false）
- `start_rviz:=true/false` - 是否启动RViz（默认：true）
- `use_sim_time:=true/false` - 是否使用仿真时间（默认：false）

### 2. 保存地图

建图完成后，保存地图：

```bash
# 使用时间戳作为文件名
./src/nav_car/navcar_slam/scripts/save_map.py

# 或指定地图名称
./src/nav_car/navcar_slam/scripts/save_map.py my_map
```

地图将保存在 `~/maps/` 目录下。

### 3. 定位

使用已保存的地图进行定位：

```bash
ros2 launch navcar_slam localization.launch.py map_file:=~/maps/map_20231127_143022.pbstream
```

可选参数：
- `use_mock_hardware:=true/false` - 是否使用模拟硬件（默认：false）
- `start_rviz:=true/false` - 是否启动RViz（默认：true）
- `use_sim_time:=true/false` - 是否使用仿真时间（默认：false）

## 配置说明

### Cartographer配置

主要配置文件位于 `config/` 目录：

- `diffbot_lds_2d.lua`: SLAM建图配置
  - 配置了2D LIDAR扫描参数
  - 设置了里程计使用
  - 优化了扫描匹配参数

- `diffbot_lds_2d_localization.lua`: 定位配置
  - 基于SLAM配置
  - 启用纯定位模式
  - 优化定位参数

### 坐标系配置

- `map_frame`: map
- `tracking_frame`: base_footprint
- `published_frame`: odom
- `odom_frame`: odom

## 依赖

- cartographer_ros
- cartographer_rviz
- navcar_description
- navcar_bringup

## 故障排除

1. **激光雷达数据异常**：检查 `/scan` 话题是否正常发布
2. **里程计异常**：检查 `/odom` 话题是否正常发布
3. **TF树异常**：使用 `ros2 run tf2_tools view_frames` 检查TF树
4. **地图保存失败**：确保cartographer节点正在运行，且已完成建图

## 话题

### 订阅的话题
- `/scan` (sensor_msgs/LaserScan): 激光雷达扫描数据
- `/odom` (nav_msgs/Odometry): 里程计数据

### 发布的话题
- `/map` (nav_msgs/OccupancyGrid): 栅格地图
- `/trajectory_node_list` (visualization_msgs/MarkerArray): 轨迹节点列表
