#include <ros/ros.h>
#include <mav_msgs/Actuators.h>
#include <geometry_msgs/Twist.h>

struct Parameters
{ 
  double Phase;
  double Skew;
  double Amp;
  double Ampback;
  double Freq;

  Parameters(double phase, double skew, double amp, double ampback, double freq) 
  : Phase(phase), Skew(skew), Amp(amp), Ampback(ampback), Freq(freq) {}
};

enum class Motion {
  FOREWARD,
  BACKWARD,
  LEFT,
  RIGHT,
  JUMP,
  SLOW_JUMP,
  STOP
};

Parameters getParameters(Motion motion);

class controllerNode{
  ros::NodeHandle nh;

  ros::Publisher commands;
  ros::Subscriber velocity;
  ros::Timer timer;

  // Parameters
  Parameters parameters = Parameters(0,0,0,0,0);

  // Motions
  Motion motion = Motion::STOP;

  double hz;             // frequency of the main control loop

public:
  controllerNode():hz(1000.0){
      commands = nh.advertise<mav_msgs::Actuators>("commands", 1);
      velocity = nh.subscribe("cmd_vel",1000, &controllerNode::commandCallback, this); 
      timer = nh.createTimer(ros::Rate(hz), &controllerNode::controlLoop, this);
  }

  void commandCallback(const geometry_msgs::Twist::ConstPtr msg) {
    double linear_velocity = msg->linear.x;
    double angular_velocity = msg->angular.z;
    double linear_velocity_threshold = 0.1;
    double linear_angular_threshold = 0.3;

    if (angular_velocity > linear_angular_threshold) {
      this->motion = Motion::RIGHT;
      std::cout << "Rotating right";

    } else if (angular_velocity < -linear_angular_threshold) {
      this->motion = Motion::LEFT;
      std::cout << "Rotating left";

    } else if (linear_velocity > linear_velocity_threshold) {
      this->motion = Motion::FOREWARD;
      std::cout << "Moving forward";

    } else if (linear_velocity < -linear_velocity_threshold) {
      this->motion = Motion::BACKWARD;
      std::cout << "Moving backward";


    } else {
      this->motion = Motion::STOP;
      std::cout << "Stop";
    }

    parameters = getParameters(this->motion);
  }

  void controlLoop(const ros::TimerEvent& t){

    mav_msgs::Actuators msg;

    msg.angular_velocities.resize(5);
    msg.angular_velocities[0] = this->parameters.Phase; // Phase between front and back legs (in degree)
    msg.angular_velocities[1] = this->parameters.Skew; // Phase between front left + back right legs and front right and left back legs
    msg.angular_velocities[2] = this->parameters.Amp; // Amplitude change of all legs
    msg.angular_velocities[3] = this->parameters.Ampback; // Amplitude change of back legs (added to angular_velocities[2])
    msg.angular_velocities[4] = this->parameters.Freq; // Frequency of legs

    commands.publish(msg);

  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "controller_node");
  ROS_INFO_NAMED("controller", "Controller started!");
  controllerNode n;
  ros::spin();
}

Parameters getParameters(Motion motion) {
  switch (motion) {
    case Motion::FOREWARD:
      return Parameters(0, 90, 0, 0, 12);    

    case Motion::BACKWARD:
      return Parameters(0, 90, 0, 0, 3);

    case Motion::LEFT:
      return Parameters(0, -45, 0, 0, 7);

    case Motion::RIGHT:
      return Parameters(0, 45, 0, 0, 7);

    case Motion::JUMP:
      return Parameters(45, 0, 40, 20, 7);

    case Motion::SLOW_JUMP:
      return Parameters(45, 0, 40, 20, 1);

    case Motion::STOP:
      return Parameters(0, 0, 0, 0, 0);  
    
    default:
      return Parameters(0, 0, 0, 0, 0); 
  }
}
