#include <ros/ros.h>
#include "RoboticsProject/WheelSpeed.h"
#include "sensor_msgs/JointState.h"
#include <sstream>

void RigidBodyCalc(const RoboticsProject::WheelSpeed &msg){
    
    //publish
    pub.publish(wheelSpeedMsg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("WheelSpeed", 1000, RigidBodyCalc());

  ros::spin();

  return 0;
}
