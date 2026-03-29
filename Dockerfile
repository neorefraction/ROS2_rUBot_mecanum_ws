FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# --------------------
# Base + locales + OpenGL
# --------------------
RUN apt-get update && apt-get install -y \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    build-essential \
    mesa-utils \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# --------------------
# ROS 2 repository
# --------------------
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

# --------------------
# ROS 2 + Gazebo + RViz
# --------------------
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-desktop \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-image-transport-plugins \
    ros-$ROS_DISTRO-xacro \
    gazebo \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    python3-pip \
    python3-dev \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# --------------------
# PyTorch (CUDA, pero sin duplicar CUDA)
# --------------------
RUN pip3 install --no-cache-dir \
    torch torchvision

# --------------------
# Ultralytics
# --------------------
RUN pip3 install --no-cache-dir ultralytics --no-deps

# --------------------
# rosdep
# --------------------
RUN rosdep init || true

# --------------------
# Auto source
# --------------------
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc

WORKDIR /home
CMD ["bash"]
