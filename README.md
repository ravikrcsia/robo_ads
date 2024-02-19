# robo_ads_assignment

## Objective:

Write custom ROS nodes for Autonomous Docking/Charging of
Turtlebot3 Robot, within a simulated Gazebo world environment. The system should enable the robot to autonomously navigate back to the docking station, when the battery level is below 30%. Utilize ArUco Marker/AR Tag Transformation for localization of the docking station within the map. Utilize a depth camera and laser scanner for precise positioning of the robot with the center of the docking station.

### Hardware Used
  - i5 8th Gen
  - Nvidia GPU - GTX or RTX series

### Software Used
  -  Ubuntu 20.04
  -  ROS Noetic Ninjemys
  -  Gazebo


### Setup
Follow the steps.
- Open the terminal and run the following commands
```bash
mkdir (your_workspace_name)
cd (your_workspace_name)
mkdir src
cd src
git clone https://github.com/ravikrcsia/robo_ads.git
cd ~/(your_workspace_name)
catkin build
```


### TO START THE TASK

- Start the simulation of the world with other nodes like `dummy_battery`, `aruco_detect`, `dock_controller` with the world file `office.world`. Also it will start the image view of the back camera.
```bash
roslaunch turtlebot3_gazebo turtlebot3_house.launch
```

- Running the `move_base` nodes all the paras
```bash
roslaunch turtlebot3_navigation turtlebot3_navigation.launch
```

- Running the `robo_nav` node to start the bot
```bash
rosrun turtlebot3_navigation robo_nav.py
```

### Docking Station Model used

<img src="docking_station.jpg">

You can find the model file in `turtlebot3_gazebo/models/aruco_marker`


### Explanation of the Nodes

#### dummy_battery.py (`battery_simulator`)

The `dummy_battery.py` script is a ROS Node that simulates a battery system. It initializes with a starting battery level, discharge rate, and charge rate, and subscribes to the `battery_status` topic to receive charge control commands. The Node includes functions to discharge and charge the battery based on the received commands, and it continuously publishes the battery level and status. The script is designed to run indefinitely, simulating the behavior of a battery system in a ROS environment.

#### aruco_detect.py (`aruco_marker_tracker`)

The `aruco_detect.py` script is a ROS (Robot Operating System) node that detects and tracks Aruco markers in real-time. It uses the OpenCV library and the Aruco marker detection library to find Aruco markers in an image, estimate their pose relative to the camera, and publish the pose information as transforms in the ROS TF (Transform) tree. Additionally, it uses the TF2_ROS library to publish transforms and the pose of the marker with respect to the `/odom` frame.

#### dock_controller.py (`dock_controller_node`)

The `dock_controller.py` script is designed for autonomous docking of a robot using a back camera and the lidar sensor. The script utilizes various ROS messages and services to receive and process data from sensors, calculate errors, and generate velocity commands to guide the robot to a charging dock.

The script uses a PID-like controller to calculate the linear and angular errors based on the robot's current position and orientation, and the goal position and orientation of the charging dock. The linear and angular gains (`kp_linear` and `kp_angular`) are used to adjust the velocity commands.

<b>The docking process is divided into two stages: moving to a pre-dock pose and then performing the actual docking. The script uses a proportional control method to move the robot to the pre-dock pose and then uses a more precise control method to orient the robot towards the dock and maintain a minimum distance from the dock.</b>

The script also includes a normalization function for angles and a function to calculate the distance and heading error between the robot and the goal.

The docking process is triggered by a ROS service call, and the script returns the docking time and the distance to the goal after docking.

#### robo_nav.py (`robo_nav`)

The script is designed to navigate the robot, monitor its battery level, and perform docking for charging. It uses the `move_base` action client to send navigation goals to the robot, and it subscribes to various topics such as battery level, battery status, charging dock pose, and odometry. 

The script also includes a main loop that checks the current navigation mode and calls the appropriate function based on the mode. Overall, the script provides a simple and modular structure for controlling a robot, monitoring its battery level, and performing docking for charging.