#include <ros/ros.h>
#include "RoboticsProject/WheelSpeed.h"
#include "sensor_msgs/JointState.h"
#include <sstream>

int main(int argc, char **argv) {
    ros::init(argc, argv, "EncoderParser");
    Pub_sub_encoder_parser encParser;

    //dynamic_reconfigure::Server<project_1::integrationConfig> server;
    //dynamic_reconfigure::Server<project_1::integrationConfig>::CallbackType f;

    //f = boost::bind(&Pub_sub_odometry::setIntegration, &pubSubOdometry, _1);
    //server.setCallback(f);

    
    return 0;
}