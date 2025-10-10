# Create a Dashboard to Control Your ROS2 Robot from a Web Browser
Web Dashboard provides a framework for controlling and monitoring robotic systems remotely.

![](./Images/06_Web/Dashboard.png)

Most robots used in companies come with web dashboards, allowing operators to easily control the robot's core functions and view diagnostic information without needing to rely on development tools like RViz2 or ROS commands to interact with the robot.

For this, we will be using:
- **rosbridge** → A WebSocket ↔︎ ROS 2 bridge, allowing a web page to talk with the ROS graph.
- **roslibjs** → A JavaScript library that uses rosbridge to publish/subscribe to topics and call services from the browser.
- **Custom code** in `index.html`file integrating code languages:
    - HTML (HyperText Markup Language) → Defines the structure of the web page (buttons, text fields, joystick, etc.).
    - CSS (Cascading Style Sheets) → Provides the visual style and layout (colors, sizes, fonts, responsive design for mobile/tablet).
    - JavaScript → Implements the logic/behavior (e.g., when you press a button, it publishes a Twist message).

Some of the main functions of `rosbridge_server` include:

1. Creating a WebSocket connection, which allows bidirectional communication between the robot and web browsers⁠⁠

2. Exposing ROS's publish/subscribe functionality, enabling web applications to publish and subscribe to ROS topics⁠⁠

3. Stablish communication between ROS and web browsers

You need first to add a “web” folder to your ROS 2 workspace:

````lua
ros2_ws/
├─ src/
│  └─ … your robot packages …
└─ web/
   ├─ index.html
   └─ (optional) js/roslib.min.js   <-- vendor the file to avoid CDN issues
````

# 🚀 Setup ROS2 Web Control for rUBot 

This guide explains how to control your ROS2 robot from a **web browser**  
when the **Robot (real or virtual)** and the **PC (Windows 11)** are on the same WiFi/LAN.

## 1. On the **Robot**

In our robot we will proceed with all the necessary installation packages to be controlled within a webpage on the PC. 

Open a terminal and make sure rosbridge is installed:
```bash
sudo apt update
sudo apt install ros-humble-rosbridge-server -y
````
Then you proceed with:
- Let's first bringup the robot:
    - in virtual environment
    ````shell
    ros2 launch my_robot_bringup my_robot_bringup_sw.launch.xml
    ````
    - in Real environment
    ````shell
    ros2 launch my_robot_bringup my_robot_bringup_hw.launch.py
    ````
- Start rosbridge websocket (port 9090)
    ````shell
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml
    ````
    This starts a WebSocket server at: `ws://<ROBOT_IP>:9090`
        
- Run a simple HTTP server. Open PowerShell or CMD and run:
    ````python
    cd ~/ROS2_rUBot_mecanum_ws/web
    python3 -m http.server 8000
    ````
    This starts a simple HTTP server on port 8000.  

## 2. On the **PC Browser**

- Open Chrome / Edge / Firefox.

- Go to the generated `webpage_address`:
    - Real robot the address will be `http://<ROBOT_IP>:8000/`
    - Virtual robot you will obtain this address typing `webpage_address` in a new terminal.
    The page (index.html) will be loaded.
- In the connection field we have to write the address where our RosBridge is run. 
    - Real robot the address will be `ws://<ROBOT_IP>:9090`
    - Virtual robot you will obtain this address typing `rosbridge_address` in a new terminal
- Click Connect → it should show Connected.

You will be ready to control your robot from the web page!

Let's see some control examples of how to use this web dashboard.

# 🚀 Exemples ROS2 Web Control for rUBot 
### Dashboard to update the Position of the Robot

This example shows how to create a simple web dashboard to update the position of the robot. The `index_position.html` file contains a simple form to update the position of the robot.

You can find the code in the `web` folder of the repository.

Remember to copy this file as `index.html` in the `web` folder of PC

The updated page should look something like this:
![](./Images/06_Web/PosDashboard.png)

You can move the robot around with the following program:
````shell
ros2 run teleop_twist_keyboard teleop_twist_keyboard
````
You should see that the odom data updates in real time in the webpage.

### Dashboard to move the robot and update its Position
This example shows how to create a simple web dashboard to move your robot and visualize our position in the webpage.

To be able to move the robot through a virtual joystick we will be using another small library called nippleJS (https://github.com/yoannmoinet/nipplejs) which provides an easy way to add joystick to any website.

The `index_joycontrol.html` file contains a simple form to update the position of the robot.

You can find the code in the `web` folder of the repository.

Remember to copy this file as `index.html` in the `web` folder of PC

The updated page should look something like this:
![](./Images/06_Web/JoyDashboard.png)

### Dashboard to move the robot, update its Position and view the camera
This example shows how to create a simple web dashboard to to move your robot with a virtual joystick, visualize our position and view the camera streaming images in the webpage.

The `index_JoyPosCam.html` file contains a simple form to update the position of the robot.

You can find the code in the `web` folder of the repository.

Remember to copy this file as `index.html` in the `web` folder of PC

The updated page should look something like this:
![](./Images/06_Web/JoyPosCamDashboard.png)
