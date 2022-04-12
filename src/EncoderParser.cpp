#include <ros/ros.h>
#include "RoboticsProject/WheelSpeed.h"
#include "sensor_msgs/JointState.h"
#include <sstream>

class Pub_sub_encoder_parser{
private:
    ros::NodeHandle n;

    ros::Subscriber sub;
    ros::Publisher pub;

    ros::Time lastTime;
    double lastTick[4];
    
public:
    Pub_sub_encoder_parser(){
        sub = n.subscribe("/wheel_states", 1, &Pub_sub_encoder_parser::calcSpeed, this);
        pub = n.advertise<RoboticsProject::WheelSpeed>("/wheel_speed", 1);

        lastTime = ros::Time::now();
        
    }

    void calcSpeed(const sensor_msgs::JointState &msg){
        double speed[4];
        
        double dt;
        ros::Time currentTime;

        //Reads currentTime from message's header
        currentTime = msg.header.stamp;
        
        //Computes dt from last message
        dt = (lastTime - currentTime).toSec();
        
        #define RATIO 5.71428571429
        double adjTime = RATIO/dt;

        for(int i=0;i<4;i++){
            speed[i] = (lastTick[i]-msg.position[i])*adjTime;
            lastTick[i] = msg.position[i];
        }
        lastTime = currentTime;

        RoboticsProject::WheelSpeed wheelSpeedMsg;

        //set header
        wheelSpeedMsg.header.stamp = currentTime;
        //wheelSpeedMsg.header.frame_id = "wspeed";

        //set velocity
        wheelSpeedMsg.rpm_fl = speed[0];
        wheelSpeedMsg.rpm_fr = speed[1];
        wheelSpeedMsg.rpm_rl = speed[2];
        wheelSpeedMsg.rpm_rr = speed[3];

        //publish
        pub.publish(wheelSpeedMsg);
    }
};

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