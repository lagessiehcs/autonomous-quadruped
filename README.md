## Task Description 
The goal of the project is passing a parkour with your robot in minimal time, while not hitting any of the cones.
The core parts, including but not limited are:
* A Unity simulation environment. A base version will be provided to you.
* A ROS-Simulation-Bridge providing ROS interfaces (topics, services). It will communicate with the simulation via TCP while at the same time providing relevant information to other ROS nodes. A base version will be provided to you. You are free to adjust the code.
* A basic position controller leveraging gait walking.
* A state machine for your robot.
* A perception pipeline that converts the depth image, first to a point cloud and second to a voxel-grid representation of the environment.
* A path planner that generates a path through the environment.
* A trajectory planner that plans a trajectory based on the found path.
* It is required that you at least once implement one of the following ROS elements by yourself:
ROS service - document where you used a ROS service call
* Implement an own message type - document which message you definened by yourself
  
## Getting Started

1. Copy the src-folder to your repository and build it
2. Download the Unity Environment: https://syncandshare.lrz.de/getlink/fiLvgiTXetubiN1i4PRjuR/
3. Unzip the Unity file and copy the files to .../devel/lib/simulation/
4. Run a test:
  a.) roslaunch simulation simulation.launch
  b.) rosrun controller_pkg controller_node
  
You will see the quadruped in front of you and it is walking forward. The goal is to navigate the quadruped through the parcour. If you take a look to the source-code of the controller_node you will see how to control the quadruped with the inputs Phase, SkewPhase, Amplitude change, Amplitude back and Frequency (of the legs).

Part of the task is to play with these control inputs in order to make the robot walk. You will have to experiment with the differend commands to pass the parcour. To give you another hint. A possible combination for rotation in spot is: 

Phase = 0
Skew Phase = 45
Amplitude Change = 0
Amplitude Back = 0
Frequency = 7


# Tips

Here are a couple of hints regarding the implementation. The hints are just suggestions; you are free so solve the task differently:
- Generating point cloud from depth image: use depth_image_proc in http://wiki.ros.org/depth_image_proc.
- Generating occupancy Grid: use Octomap in http://wiki.ros.org/octomap.
- Please ping us in case you have any questions or if you get stuck in some subtasks.
- Use a global map as your voxel grid representation. Use a smart resolution for your voxel grid representation (e.g. 1m).

