#include <ros/ros.h>
//#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include "RoboticsProject/WheelSpeed.h"
#include <sstream>
//#include <dynamic_reconfigure/server.h>

class Pub_sub_encoder_parser{
private:
    ros::NodeHandle n;

    ros::Subscriber sub;
    ros::Publisher pub;

    ros::Time lastTime;

    
public:
    Pub_sub_encoder_parser(){
        sub = n.subscribe("/twist", 1, &Pub_sub_encoder_parser::calcSpeed, this);
        pub = n.advertise<project_1::CustomOdometry>("/wheel_speed", 1);

        lastTime = ros::Time::now();
    }

    void calcSpeed(const geometry_msgs::TwistStamped::ConstPtr& msg){
        double speed[4];
        double tick[4];
        double dt;
        ros::Time currentTime;

        //Reads currentTime from message's header
        currentTime = msg->header.stamp;
        
        //Computes dt from last message
        dt = (currentTime - lastTime).toSec();

        tick = msg->twist.linear.x; //change msg
        #define RATIO 2
        double adjTime = RATIO/dt
        for(int i=0;i<4;i++){
            speed[i] = tick[i]*adjTime;
        }


        RoboticsProject::WheelSpeed wheelSpeedMsg;

        //set header
        wheelSpeedMsg.header.stamp = currentTime;
        //wheelSpeedMsg.header.frame_id = "wspeed";

        //set velocity
        wheelSpeedMsg.fl = speed[0];
        wheelSpeedMsg.fr = speed[1];
        wheelSpeedMsg.rl = speed[2];
        wheelSpeedMsg.rr = speed[3];

        //publish
        pub.publish(wheelSpeedMsg);
    }
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "EncoderParser");
    Pub_sub_encoder_parser encParser;

    //dynamic_reconfigure::Server<project_1::integrationConfig> server;
    //dynamic_reconfigure::Server<project_1::integrationConfig>::CallbackType f;

    //f = boost::bind(&Pub_sub_odometry::setIntegration, &pubSubOdometry, _1);
    //server.setCallback(f);

    ros::spin();
    return 0;
}