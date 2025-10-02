FROM osrf/ros:humble-desktop-full

# Fix expired ROS 2 GPG key (evita GPG errors a apt)
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros2-latest-archive-keyring.gpg

# Paquets extra: RViz2 + Gazebo Classic + utils
RUN apt-get update && apt-get install -y --no-install-recommends \
    x11-apps \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libglu1-mesa \
    mesa-utils \
    libqt5x11extras5 \
    libxkbcommon-x11-0 \
    libx11-6 \
    libxext6 \
    libxrender1 \
    libxtst6 \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    python3-pip \
    wget unzip git iputils-ping \
    # ROS 2 add-ons
    ros-humble-rviz2 \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-plugins \
    ros-humble-gazebo-ros2-control \
    ros-humble-nav2-bringup \
    ros-humble-nav2-simple-commander \
    ros-humble-tf-transformations \
    ros-humble-cartographer-ros \
    ros-humble-teleop-twist-keyboard \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# YOLO (compatibilitat amb ultralytics)
#RUN pip3 install --no-cache-dir ultralytics "numpy<1.24"

# Entorn ROS a bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Manté el contenidor viu amb shell
CMD ["bash"]
