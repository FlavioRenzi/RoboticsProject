#include <ros/ros.h>
#include "RoboticsProject/WheelSpeed.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"
#include <sstream>

class Pub_sub_kinematic{
private:
    ros::NodeHandle n;

    geometry_msgs::TwistStamped kinematicGeometryMessage;

    ros::Subscriber sub;
    ros::Publisher pub;
    
public:
    Pub_sub_kinematic(){
        sub = n.subscribe("/wheel_speed", 1, &Pub_sub_encoder_parser::RigidBodyCalc(), this);
        pub = n.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1);

        lastTime = ros::Time::now();
        
    }

    void RigidBodyCalc(const RoboticsProject::WheelSpeed &msg){

        double l1 = 10;
        double l2 = 20;
        double K[3][4] = {{1,1,1,1}, {1,-1,-1,1},{-1/(l1+l2), 1/(l1+l2), -1/(l1+l2), 1/(l1+l2)}};
        vx = K[0][0]*msg.rpm_fl + K[0][1]*msg.rpm_fr + K[0][2]*msg.rpm_rl + K[0][3] * msg.rpm_rr;
        vy = K[1][0]*msg.rpm_fl + K[1][1]*msg.rpm_fr + K[1][2]*msg.rpm_rl + K[1][3] * msg.rpm_rr;
        wz = K[2][0]*msg.rpm_fl + K[2][1]*msg.rpm_fr + K[2][2]*msg.rpm_rl + K[2][3] * msg.rpm_rr;

        kinematicGeometryMessage.header.stamp = msg.header.stamp;

        kinematicGeometryMessage.

        //publish
        pub.publish();
    }
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "listener");

  Pub_sub_kinematic pub_sub

  ros::spin();

  return 0;
}
