# Utilitzem una imatge base més adient, que ja porta eines bàsiques de ROS2
FROM ros:humble-ros-base

# Canviem a l'intèrpret de comandes a bash per a més consistència
SHELL ["/bin/bash", "-c"]

# Instal·la dependències essencials per construir i executar
# Combinem les dues comandes apt-get en una per optimitzar les capes de la imatge
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Eines de construcció
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    python3-pip \
    wget \
    unzip \
    git \
    iputils-ping \
    # Eina per gestionar dependències de ROS des del codi font
    python3-rosdep \
    # Dependències del teu robot (llista mínima)
    libusb-1.0-0-dev \
    libeigen3-dev \
    ros-humble-xacro \
    ros-humble-cv-bridge \
    ros-humble-vision-msgs \
    ros-humble-image-geometry \
    ros-humble-image-publisher \
    ros-humble-image-transport \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-tf2-ros \
    ros-humble-usb-cam \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-demo-nodes-cpp \
    ros-humble-teleop-twist-keyboard \
    ros-humble-rosbridge-server \
    # Eines gràfiques de ROS2
    ros-humble-rviz2 \
    ros-humble-rqt-graph \
    ros-humble-rqt-plot \
    # Paquets per a Navegació (Nav2) i SLAM (Cartographer)
    ros-humble-nav2-bringup \
    ros-humble-nav2-simple-commander \
    ros-humble-tf-transformations \
    ros-humble-cartographer-ros \
    # Dependències per a visualització remota (X11 Forwarding)
    x11-apps \
    libgl1-mesa-glx \
    mesa-utils \
    libqt5x11extras5 \
    libxkbcommon-x11-0 \
    && rm -rf /var/lib/apt/lists/*

# Install Yolo
RUN pip3 install ultralytics "numpy<1.24"

# Install Serial comunication module for arduino nano
RUN pip install pyserial

# Creació i construcció del Workspace

# 1. Clona el teu repositori principal
WORKDIR /root
RUN git clone https://github.com/manelpuig/ROS2_rUBot_mecanum_ws.git

# 2. Entra al directori 'src' del workspace per afegir el paquet del LIDAR
WORKDIR /root/ROS2_rUBot_mecanum_ws/src
RUN git clone https://github.com/Slamtec/rplidar_ros.git -b ros2

# 3. Torna a l'arrel del workspace i construeix tot el projecte
WORKDIR /root/ROS2_rUBot_mecanum_ws

# S'afegeix 'apt-get update' i s'elimina 'rosdep init'
RUN source /opt/ros/humble/setup.bash && \
    # Actualitza les llistes de paquets d'APT abans d'utilitzar rosdep
    apt-get update && \
    # Actualitza les fonts de paquets de rosdep
    rosdep update && \
    # Instal·la les dependències dels paquets que tens a 'src'
    rosdep install --from-paths src --ignore-src -r -y --skip-keys="gazebo_ros" && \
    # Finalment, compila tot el workspace
    colcon build --symlink-install && \
    # Neteja la memòria cau d'APT per mantenir la imatge petita
    rm -rf /var/lib/apt/lists/*

# Source and .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /root/ROS2_rUBot_mecanum_ws/install/setup.bash" >> /root/.bashrc && \
    echo "cd /root/ROS2_rUBot_mecanum_ws" >> /root/.bashrc

# Set the default command to run when the container starts
CMD ["/bin/bash"]