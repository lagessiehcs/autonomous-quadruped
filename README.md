# Autonomous Quadruped 

## Getting Started

1. Copy the src-folder to your repository and build it
2. Download the Unity Environment: [AutonomousQuadruped](https://www.dropbox.com/scl/fi/pqou4svk0j5vsj3f3ph6l/AutonomousQuadruped.zip?rlkey=983xyod376rsz9f1qewxzg79o&st=px518c1z&dl=0)
3. Unzip the Unity file and copy the files to .../devel/lib/simulation/
4. Run a test:
  a.) roslaunch simulation simulation.launch
  b.) rosrun controller_pkg controller_node
  
You will see the quadruped in front of you and it is walking forward. The goal is to navigate the quadruped through the parcour. If you take a look to the source-code of the controller_node you will see how to control the quadruped with the inputs Phase, SkewPhase, Amplitude change, Amplitude back and Frequency (of the legs).

## Source
The framework of this project and the Unity Environment were provided by [Prof. Dr. Markus Ryll](https://www.professoren.tum.de/ryll-markus) as part of the course [Introduction To ROS](https://www.moodle.tum.de/course/info.php?id=88252). The initial framework for this project to build upon can be found in the branch **framework**
