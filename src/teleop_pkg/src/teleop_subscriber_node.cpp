#include <ros/ros.h>
#include <ros/console.h>
#include <mav_msgs/Actuators.h>
#include <std_msgs/String.h>
#include <iostream>

class teleopControllerNode{
  ros::NodeHandle nh;

  ros::Publisher commands;
  ros::Subscriber teleop;
  ros::Timer timer;

  struct Parameter
{ 
  double Phase;
  double Skew;
  double Amp;
  double Ampback;
  double Freq;

  Parameter(double phase, double skew, double amp, double ampback, double freq) 
  : Phase(phase), Skew(skew), Amp(amp), Ampback(ampback), Freq(freq) {}
};


  double hz; // frequency of the main control loop   
  Parameter parameters = Parameter(0,0,0,0,0); // Parameters

public:
  teleopControllerNode():hz(1000.0){
      
      commands = nh.advertise<mav_msgs::Actuators>("commands", 1);
      teleop = nh.subscribe("teleop", 1000, &teleopControllerNode::commandCallback, this);
      timer = nh.createTimer(ros::Rate(hz), &teleopControllerNode::controlLoop, this);
  }

  void commandCallback(const std_msgs::String::ConstPtr& msg) {
      std::string key = msg->data;
      
      if (key == "w") {
        this->parameters = Parameter(0,90,0,0,12);
        ROS_INFO("Walking forward");

      } else if (key == "s") {
        this->parameters = Parameter(0,90,0,0,2);
        ROS_INFO("Walking backward"); 

      } else if (key == "a") {
        this->parameters = Parameter(0,-45,0,0,7);
        ROS_INFO("Turning left");

      } else if (key == "d") {
        this->parameters = Parameter(0,45,0,0,7);
        ROS_INFO("Turning right");

      } else if (key == "e") { // obstacle mode
        this->parameters = Parameter(45,0,40,20,7);
        ROS_INFO("Overcoming obstacles");

      } else if (key == "q") { // slowly escape obstacle mode
        this->parameters = Parameter(45,0,40,20,1);
        ROS_INFO("Overcoming obstacles");

      } else if (key == "c") { // stop
        this->parameters = Parameter(0,0,0,0,0);
        ROS_INFO("Stop");
      }         
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
    // sleep(period);

  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "teleop_subscriber");
  ROS_INFO_NAMED("teleopController", "TeleopController started!");
  teleopControllerNode n;
  ros::spin();
}
