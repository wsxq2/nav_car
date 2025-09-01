# 基于官方 ROS2 镜像，适合开发环境
FROM ros:humble

# Accept build arguments for proxy settings
ARG PROXY_HOST=host.docker.internal
ARG PROXY_PORT=7890
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Set proxy environment variables if provided
ENV https_proxy=http://${PROXY_HOST}:${PROXY_PORT}
ENV http_proxy=$https_proxy
ENV no_proxy=localhost,127.0.0.1,::1,192.168.0.0/16

RUN echo 'Asia/Shanghai' > /etc/timezone && ln -sf /usr/share/zoneinfo/Asia/Shanghai /etc/localtime

# Update the apt sources to use Tsinghua University's mirror
RUN <<EOF
#!/bin/bash
[[ $(uname -m) == x86_64 ]] && port_string="" || port_string="-ports"
cat > /etc/apt/sources.list <<EOG
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu${port_string}/ jammy main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu${port_string}/ jammy-updates main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu${port_string}/ jammy-backports main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu${port_string}/ jammy-security main restricted universe multiverse
EOG
EOF

# Add ROS 2 apt repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN mv /etc/apt/sources.list.d/ros2.sources /etc/apt/sources.list.d/ros2.sources.bak && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu jammy main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 手动模拟 rosdep init
RUN mkdir -p /etc/ros/rosdep/sources.list.d/ && curl -o /etc/ros/rosdep/sources.list.d/20-default.list -L https://mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master/rosdep/sources.list.d/20-default.list

# set rosdep mirror to Tsinghua University
ENV ROSDISTRO_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml


# 安装常用工具和依赖
RUN apt-get update && apt-get install -y \
    python3-pip \
    git \
    cmake \
    vim \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-vcstool \
    build-essential \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-ros2controlcli \
    ros-humble-rviz2 \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-rosidl-default-generators \
    sudo \
    locales \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    unzip \
    curl \
    ros-humble-diagnostic-updater \
    libpcap-dev \
    clangd \
    joystick \
    ros-humble-joy-linux \
    ros-humble-teleop-twist-joy \
    iproute2 \
    command-not-found \
    usbutils \
    python3-colcon-clean \
    && rm -rf /var/lib/apt/lists/*
    
SHELL ["/bin/bash", "-c"]

RUN <<EOF
#!/bin/bash

if [[ $(uname -m) == x86_64 ]]; then
# 安装最新版本的 clangd
CLANGD_VERSION=$(curl -s https://api.github.com/repos/clangd/clangd/releases/latest | grep '"tag_name":' | sed -E 's/.*"([^"]+)".*/\1/') && \
    curl -L -o /tmp/clangd-linux.zip "https://github.com/clangd/clangd/releases/download/${CLANGD_VERSION}/clangd-linux-${CLANGD_VERSION}.zip" && \
    unzip /tmp/clangd-linux.zip -d /tmp/clangd-linux && \
    cp -r /tmp/clangd-linux/*/{bin,lib} /usr/local && \
    chmod +x /usr/local/bin/clangd && \
    rm -rf /tmp/clangd-linux.zip /tmp/clangd-linux
elif [[ $(uname -m) == aarch64 ]]; then
# 安装 21.1.0 版本的 clangd
curl -L -o /tmp/clangd.tar.gz "https://wsxq21.55555.io/clangd.tar.gz" && \
    tar -xzf /tmp/clangd.tar.gz -C /tmp/ && \
    cp -r /tmp/clangd/{bin,lib} /usr/local && \
    chmod +x /usr/local/bin/clangd && \
    rm -rf /tmp/clangd.tar.gz /tmp/clangd
fi
EOF

# 配置开发用户
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && usermod -aG sudo,input $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# 切换到开发用户
USER $USERNAME
ENV MY_WORKSPACE=/home/$USERNAME/workspace 
WORKDIR $MY_WORKSPACE

# 下载个人配置文件
RUN git clone https://github.com/wsxq2/profile.git ~/.MyProfile && cd ~/.MyProfile && ./deploy.sh ${PROXY_HOST}

# Set up environment
RUN echo "source ${MY_WORKSPACE}/install/setup.bash" >> ~/.bashrc

CMD ["bash"]
