# Install container

To install Docker and create a ROS2 container in your SSD disc, copy the `Docker_PC_ubuntu` folder to your PC and follow instructions:

- Allow access X11 to container
    ```shell
    xhost +local:root
    ```
- Make executable the entrypoint script
    ```shell
    cd Docker_PC_ubuntu
    chmod +x entrypoint.sh
    ```
- Create the image:
    ```shell
    docker compose build
    ```
- Save the image in actual folder
    ```shell
    docker save -o pc_humble_image.tar pc_humble_image
    ```
- Load the image later in another PC
    ```shell
    docker load -i pc_humble_image.tar
    ```
- Create the container pc_humble:
    ```shell
    docker compose up -d
    ```
- Open a new terminal in a container:
    ```shell
    docker exec -it pc_humble bash
    ```
- Verify that ROS2 is working with graphical interface:
    ``shell
    rviz2
    gazebo
    ```
    > Gazebo is working at 15-20 FPS
- Stop the container:
    ```shell
    docker compose down
    ```
- Remove the container:
    ```shell
    docker rm -f pc_humble
    ```

To use VScode to work inside the container, install the extension "Remote Development" and follow instructions:
- Open VScode
- In "Dev-Containers" right-click on the "docker_pc_ubuntu pc_humble" container and select `Attach in a new window`
- Open folder in root
- clone your ROS2 project:
    ```shell
    git clone https://github.com/manelpuig/ROS2_rUBot_mecanum_ws.git
    ```
- Verify the `.bashrc` file:
    ```shell
    export ROS_DOMAIN_ID=0
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export GAZEBO_MODEL_PATH=/home/user/ROS2_rUBot_mecanum_ws/src/my_robot_bringup/models:$GAZEBO_MODEL_PATH
    source /opt/ros/humble/setup.bash
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
    source /root/ROS2_rUBot_tutorial_ws/install/setup.bash
    source /root/ROS2_rUBot_mecanum_ws/install/setup.bash
    #cd /root/ROS2_rUBot_tutorial_ws
    cd /root/ROS2_rUBot_mecanum_ws
    git config --global user.email "xxx@alumnes.ub.edu"
    git config --global user.name "your_github_username"
    ```
- Test the robot bringup
    ```shell
    ros2 launch my_robot_bringup my_robot_bringup_sw.launch.xml use_sim_time:=true robot:=rubot/rubot_mecanum.urdf custom_world:=square3m_walls.world x0:=0.0 y0:=0.0 yaw0:=1.57
    ```
- Control the robot with teleop:
    ```shell
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```
    > Gazebo is working at 3-4 FPS 