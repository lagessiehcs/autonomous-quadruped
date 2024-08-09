# Autonomous Quadruped 
The goal of the project is passing a parkour with your robot in minimal time, while not hitting any of the cones.

## Table of contents 
* [Prerequisites](#prerequisites)
* [Getting Started](#getting-started)
* [Packages](#packages)
   * [simulation](#simulation)
   * [controller_pkg](#controller_pkg)
   * [teleop_actuators_keyboard](#teleop_actuators_keyboard)
   * [quadruped_rviz](#quadruped_rviz)
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
5. ‼️**IMPORTANT**‼️: Every time a new terminal is opened, remember to source first! Inside your workspace directory:
   
    ````
    source devel/setup.bash
    ````
6. To use any of the packages, the simulation environment has to be launched first!
   
    ````
    roslaunch simulation simulation.launch
    ````
7. Run a test (in a new terminal)
   
    ````
    rosrun controller_pkg controller_node
    ````
  
You will see the quadruped in front of you and it is walking forward. The goal is to navigate the quadruped through the parkour. If you take a look to the source-code of the controller_node you will see how to control the quadruped with the inputs Phase, SkewPhase, Amplitude change, Amplitude back and Frequency (of the legs).

## Packages

### simulation
Run the unity simulation
* **Launch**  
   Open a new terminal and enter the following command:
   ````
   roslaunch simulation simulation.launch
   ````

### Perception
Generate point clouds and use them for occupancy grid mapping
* **Launch**  
   Open a new terminal and enter the following command:
   ````
   roslaunch perception perception.launch
   ````

### controller
Listen to the topic /cmd_vel and publish to the topic /commands the according parameters
* **Launch**  
   Open a new terminal and enter the following command:
   ````
   rosrun controller controller_node
   ````
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

### quadruped_rviz
Store the configuration file for rviz simulation.
* **Launch**  
   Open a new terminal and enter the following command:
   ```
   rosrun rviz rviz -d `rospack find quadruped_rviz`/rviz/path_planning.rviz
   ```

## Source
The framework of this project and the Unity Environment were provided by [Prof. Dr. Markus Ryll](https://www.professoren.tum.de/ryll-markus) as part of the course [Introduction To ROS](https://www.moodle.tum.de/course/info.php?id=88252). The initial framework for this project to build upon can be found in the branch [framework](https://github.com/lagessiehcs/autonomous-quadruped/tree/framework)
