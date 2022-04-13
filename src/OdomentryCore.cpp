#include <ros/ros.h>
#include <sstream>

class Pub_sub_odomentry_core{

private:
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher odom_pub;
    ros::Publisher custom_pub;
    ros::ServiceServer resetZeroService;
    ros::ServiceServer resetGeneralService;

    ros::Time lastTime;
    double x,y,th;
    tf::TransformBroadcaster odom_broadcaster;

    int integrationType;

    public:
    Pub_sub_odometry() {
        sub = n.subscribe("/twist", 1, &Pub_sub_odometry::computeOdometry, this);

        odom_pub = n.advertise<nav_msgs::Odometry>("/odometry", 1);
        custom_pub = n.advertise<project_1::CustomOdometry>("/custom", 1);
        resetZeroService = n.advertiseService("reset_zero" , &Pub_sub_odometry::resetZero, this);
        resetGeneralService = n.advertiseService("reset_general" , &Pub_sub_odometry::resetGeneral, this);
        lastTime = ros::Time::now();

        //Reads param from launch file
        n.getParam("/InitialX", x);
        n.getParam("/InitialY", y);
        n.getParam("/InitialTheta", th);
        integrationType = 0;
    }
}