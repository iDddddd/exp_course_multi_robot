FROM ros:noetic

# 设置环境变量为非交互式
ENV DEBIAN_FRONTEND=noninteractive

# 换源
RUN sed -i 's/http:\/\/archive.ubuntu.com/http:\/\/mirror.sjtu.edu.cn/g' /etc/apt/sources.list
RUN apt-get update && apt-get install -y --reinstall ca-certificates
RUN sed -i 's/http:\/\/mirror.sjtu.edu.cn/https:\/\/mirror.sjtu.edu.cn/g' /etc/apt/sources.list
RUN apt-get update
RUN apt-get install -y gnupg
RUN echo "deb https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ focal main" | tee /etc/apt/sources.list.d/ros-latest.list
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update


# 更新包列表并安装必要的工具
RUN apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common

# 生成中文区域设置
RUN apt-get update && apt-get install -y locales && \
    locale-gen zh_CN.UTF-8

# 设置环境变量
ENV LANG zh_CN.UTF-8
ENV LANGUAGE zh_CN:zh
ENV LC_ALL zh_CN.UTF-8

# 预设时区为上海
RUN echo "Asia/Shanghai" > /etc/timezone && \
    ln -fs /usr/share/zoneinfo/Asia/Shanghai /etc/localtime && \
    apt-get install -y tzdata


# 安装 ROS1
RUN apt install -y ros-noetic-desktop
RUN apt install -y ros-noetic-gazebo-ros ros-noetic-gazebo-plugins

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# 使得 docker 内创建的文件夹/文件在宿主机上可读写
RUN echo "umask 000" >> ~/.bashrc
