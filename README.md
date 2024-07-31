# Autonomous Quadruped 

## Prerequisites

This project uses [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) and runs on [Ubuntu 20.04 (Focal)](https://releases.ubuntu.com/focal/)

## Getting Started

1. Copy the src-folder to your workspace and build it
2. Download the Unity Environment: [AutonomousQuadruped](https://www.dropbox.com/scl/fi/pqou4svk0j5vsj3f3ph6l/AutonomousQuadruped.zip?rlkey=983xyod376rsz9f1qewxzg79o&st=px518c1z&dl=0)
3. Unzip the Unity file and copy the files to .../devel/lib/simulation/
4. Run a test:
  a.) roslaunch simulation simulation.launch
  b.) rosrun controller_pkg controller_node
  
You will see the quadruped in front of you and it is walking forward. The goal is to navigate the quadruped through the parcour. If you take a look to the source-code of the controller_node you will see how to control the quadruped with the inputs Phase, SkewPhase, Amplitude change, Amplitude back and Frequency (of the legs).

## Packages
### teleop_pkg
* teleop_pkg contains two nodes: **teleop_publisher_node** and **teleop_subscriber_node**
* **teleop_publisher_node** reads the user's  keyboard input and publishes it to the topic /teleop
* **teleop_subscriber_node** subscribes to the topic /teleop and perform motion decision based on the user's input. The commands are then published to the topic /commands to move the robot
* The launch file teleop.launch launches both nodes with one single command:

  ```console
  roslaunch teleop_pkg teleop.launch
  ```

## Source
The framework of this project and the Unity Environment were provided by [Prof. Dr. Markus Ryll](https://www.professoren.tum.de/ryll-markus) as part of the course [Introduction To ROS](https://www.moodle.tum.de/course/info.php?id=88252). The initial framework for this project to build upon can be found in the branch **framework**
