# NavCar ROS2 Package

一个完整的ROS2导航车机器人项目，包含机器人描述、控制、SLAM、激光雷达驱动等功能模块。

详见 [ROS2 建图和导航实战 \| 小强的博客](https://wsxq2.55555.io/blog/2025/09/02/ROS2%E5%BB%BA%E5%9B%BE%E5%92%8C%E5%AF%BC%E8%88%AA%E5%AE%9E%E6%88%98/)

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

使用前请根据实际情况修改 `docker/.env` 文件，如果没有则手动创建，内容如下：

```bash
PROXY_HOST=192.168.3.107
PROXY_PORT=7890
DISPLAY=192.168.3.107:0.0
USER_UID=1001
```

其中的 `USER_UID` 一定要修改为当前用户 ID（可使用`id`命令查看），其他变量也要正确设置。

此外，务必确保在 `~/.bashrc` 中设置了 `http_proxy` 环境变量，因为 `.devcontainer/.devcontainer.json`中会用到该环境变量。

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
