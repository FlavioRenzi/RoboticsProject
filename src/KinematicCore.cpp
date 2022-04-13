#include <ros/ros.h>
#include "RoboticsProject/WheelSpeed.h"
#include "sensor_msgs/JointState.h"
#include <sstream>

void RigidBodyCalc(const RoboticsProject::WheelSpeed &msg){
    double l1 = 10;
    double l2 = 20;
    double K[3][4] = {{1,1,1,1}, {1,-1,-1,1},{-1/(l1+l2), 1/(l1+l2), -1/(l1+l2), 1/(l1+l2)}};
    vx = 
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
