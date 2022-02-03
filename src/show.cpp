#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <cstdio>
#include <iostream>
#include <vector>

int joints;
double joint_limit_warning; //show joint limit warning when a joint is lower than this angle to the limit
std::vector<double> upper_joint_limit, lower_joint_limit;

static const double RAD2DEG = 57.295779513082323;
static const double DEG2RAD = 0.017453292519943295;
int timer;

void chatterCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  if (timer % 10 == 0) { //plot every once in a while
    system("clear"); //clear terminal

    //check incoming state size
    if ( msg->position.size() != upper_joint_limit.size() )
      ROS_WARN("Incoming joint_states size (%d) is different from joint_limit size (%d)", (int)msg->position.size(), (int)upper_joint_limit.size());

    for (size_t i = 0; i < joints; i++){
      double joint_value = RAD2DEG * msg->position[i]; 

      bool limit_flag = false;
      if (upper_joint_limit[i] - joint_value <  joint_limit_warning ||
          lower_joint_limit[i] - joint_value > -joint_limit_warning) 
      {
        limit_flag = true;
      }

      std::cout << std::setw(1) << std::fixed << std::setprecision(0) << "(" << lower_joint_limit[i] << ")";
      std::cout << std::setw(8) << std::fixed << std::setprecision(1) << joint_value;
      std::cout << std::setw(4) << std::fixed << std::setprecision(0) << "(" << upper_joint_limit[i] << ")";

      //show warning
      if (limit_flag) 
        std::cout << "\033[1;31m Limit!!! \033[0m"; //red colored text
        // std::cout << " Limit!!!";

      std::cout << std::endl;  
    }
  }
  timer++;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  timer = 0;

  n.getParam("show_joints_states/joint_limit_warning", joint_limit_warning);
  n.getParam("show_joints_states/upper_joint_limit", upper_joint_limit);
  n.getParam("show_joints_states/lower_joint_limit", lower_joint_limit);

  if (upper_joint_limit.size() == lower_joint_limit.size()){
    joints = upper_joint_limit.size();
    std::cout << upper_joint_limit.size() << " . " << joints << std::endl;
  }
  else {
    ROS_ERROR("Size of 'upper_joint_limit' not equal to size of 'lower_joint_limit'.");
    return -1;
  }

  ros::Subscriber sub = n.subscribe("joint_states", 1, chatterCallback);

  ros::spin();

  return 0;
}

