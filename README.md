# NavCar ROS2 Package

一个完整的ROS2导航车机器人项目，包含机器人描述、控制、SLAM、激光雷达驱动等功能模块。

## 包结构

- `navcar_bringup`: 启动文件包
- `navcar_description`: 机器人描述包，包含URDF/Xacro文件
- `navcar_control`: 机器人控制包，包含ros2_control配置
- `navcar_hardware`: 硬件接口包
- `navcar_joytick`: 手柄控制包
- `navcar_slam`: SLAM算法包（使用Cartographer）
- `lidar_driver`: 激光雷达驱动包

## Docker 开发环境

本项目支持使用Docker容器进行开发，包含完整的ROS2开发环境。

本项目支持 x86_64 和 ARM64 架构。在 ARM64 平台（如树莓派、Jetson）上运行时，Docker配置会自动适配。

### 准备工作

使用前请根据实际情况修改 `docker/.env` 文件。其中的 `USER_UID` 一定要修改为当前用户 ID，`PROXY_HOST` 和 `DISPLAY` 也要正确设置。

### 使用Docker构建和运行

```bash
# 构建Docker镜像
cd docker
docker compose build

# 运行开发容器
docker compose up -d

# 进入容器
docker compose exec navcar-dev bash
```

### VS Code Dev Container

项目配置了VS Code Dev Container，可直接在VS Code中打开容器环境进行开发：

1. 安装 "Remote - Containers" 扩展
2. 在VS Code中打开项目目录
3. 选择 "Reopen in Container"

### 启动机器人

#### 启动基础包

```bash
ros2 launch navcar_bringup bringup_with_laser.launch.py
```

这包括以下内容：

- 启动摇杆手柄驱动包
- 启动 ros2 control 包
- 启动激光雷达驱动

详见对应文件内容。

#### 仅在RViz中查看机器人模型

```bash
ros2 launch navcar_description nav_car_rviz.launch.py
```

#### 启动SLAM

启动 SLAM 前要要先启动基础包。然后执行以下命令：

```bash
ros2 launch navcar_slam slam.launch.py
```

### 控制机器人

直接使用遥控手柄控制即可。

需要先按住 L1，再拖动两个摇杆，左边控制前进后退，右边控制转向。
