# Autonomous Quadruped 
The goal of the project is passing a parkour with your robot in minimal time, while not hitting any of the cones.

## Table of contents 
* [Prerequisites](#prerequisites)
* [Getting Started](#getting-started)
* [Packages](#packages)
   * [autonomous_quadruped](#autonomous_quadruped)
   * [keyboard_control](#keyboard_control)
   * [simulation](#simulation)
   * [perception](#perception)
   * [navigation](#navigation)
   * [controller](#controller)
   * [quadruped_rviz](#quadruped_rviz)
   * [teleop_actuators_keyboard](#teleop_actuators_keyboard)
* [Source](#source)

## Prerequisites
This project uses [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) and runs on [Ubuntu 20.04 (Focal)](https://releases.ubuntu.com/focal/)

## Dependencies
The following packages need to be installed:
* **depth_image_proc**

  ````
  sudo apt-get install ros-noetic-depth-image-proc
  ````
* **octomap_server**
  
  ````
  sudo apt-get install ros-noetic-octomap-server
  ````

## Getting Started
1. Copy the src-folder to your workspace and build it. </br>
   Inside your workspace directory:
   ````
   catkin build
   ````
3. Download the Unity Environment: [AutonomousQuadruped](https://www.dropbox.com/scl/fi/ur7qtlxuc4q75zl5jgfi8/AutonomousQuadruped.zip?rlkey=9973zkjr8mfln7c2cmrgts19b&st=09o0mxjm&dl=0)
4. Unzip the Unity file and copy the files to .../devel/lib/simulation/ </br>
   ‼️**IMPORTANT**‼️: After copying, make sure that the file **RoboDog_Build.x86_64** is executable. 
   ````
   sudo chmod +x RoboDog_Build.x86_64
   ````
5. ‼️**IMPORTANT**‼️: After the project is built, source the terminal first before any further ros command!  
Inside your workspace directory:
   
    ````
    source devel/setup.bash
    ````

## Packages

### autonomous_quadruped
This package launches the neccessary packages for the autonomous parkour. The neccessary packages are: [simulation](#simulation), [perception](#perception), [navigation](#navigation), [controller](#controller), and [quadruped_rviz](#quadruped_rviz).
* **Launch**  
   Open a new terminal and enter the following command:
   ````
   roslaunch autonomous_quadruped autonomous_quadruped.launch
   ````

### keyboard_control
This package launches the neccessary packages for the keyboard control of the robot. The neccessary packages are: [simulation](#simulation), [perception](#perception), [teleop_actuators_keyboard](#teleop_actuators_keyboard), and [quadruped_rviz](#quadruped_rviz).
* **Launch**  
   Open a new terminal and enter the following command:
   ````
   roslaunch keyboard_control keyboard_control.launch
   ````

### simulation
Run the unity simulation
* **Launch**  
   Open a new terminal and enter the following command:
   ````
   roslaunch simulation simulation.launch
   ````

### perception
Generate point clouds and use them for occupancy grid mapping
* **Launch**  
   Open a new terminal and enter the following command:
   ````
   roslaunch perception perception.launch
   ````

### navigation
Generate cost maps and based on that publish desired velocity values
* **Launch**  
   Open a new terminal and enter the following command:
   ````
   roslaunch navigation move_base.launch
   ````

### controller
Listen to the topic /cmd_vel and publish to the topic /commands the according parameters
* **Launch**  
   Open a new terminal and enter the following command:
   ````
   rosrun controller controller
   ````

### quadruped_rviz
Store the configuration file for rviz simulation.
* **Launch**  
   Open a new terminal and enter the following command:
   ```
   rosrun rviz rviz -d `rospack find quadruped_rviz`/rviz/path_planning.rviz
   ```

### teleop_actuators_keyboard
Keyboard control for the quadruped robot
* **Launch**  
   Open a new terminal and enter the following command:
   ```
   rosrun teleop_actuators_keyboard teleop_actuators_keyboard.py
   ```
* **Usage**
   ````
   Reading from the keyboard and publishing to Actuators!
   ---------------------------
   Moving with aswd:
   s/w - move backward/forward
   a/d - rotate left/right
   
   Special keys:
   e - Obstacles passing mode
   q - Slow obstacles passing mode
   x - Stop
   
   CTRL-C to quit
   ````

## Source
The framework of this project and the Unity Environment were provided by [Prof. Dr. Markus Ryll](https://www.professoren.tum.de/ryll-markus) as part of the course [Introduction To ROS](https://www.moodle.tum.de/course/info.php?id=88252). The initial framework for this project to build upon can be found in the branch [framework](https://github.com/lagessiehcs/autonomous-quadruped/tree/framework)
