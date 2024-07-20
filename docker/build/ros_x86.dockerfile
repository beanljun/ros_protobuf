
# Base image
FROM UBUNTU:20.04 
# 非交互式安装
ENV DEBIAN_FRONTEND=noninteractive
# 清理缓存
RUN apt-get clean && apt-get autoclean

# Install ROS
# 安装依赖项
RUN apt update && \ 
    apt install  -y \
    curl \
    lsb-release \
    gnupg gnupg1 gnupg2 \
    gdb 

# 下载ROS密钥
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# 设置ROS源
RUN  sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'

# 安装ROS
RUN apt update && \
    apt install -y  ros-noetic-desktop-full \
    python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# 将ROS添加到容器环境变量
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Install protobuf
# 安装依赖项
RUN apt update && \
    apt install -y \
    vim \
    htop \
    apt-utils \
    cmake \
    net-tools

# 将本地目录下的install文件夹拷贝到容器的/tmp目录下
COPY install /tmp/install

# 运行install文件夹下的脚本，安装abseil和protobuf
RUN /tmp/install/abseil/install_abseil.sh
RUN /tmp/install/protobuf/install_protobuf.sh

# 设置工作目录，进入容器时的默认目录
WORKDIR /work
