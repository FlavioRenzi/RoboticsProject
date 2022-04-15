#include <ros/ros.h>
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include <sstream>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <dynamic_reconfigure/server.h>
#include "RoboticsProject/parametersConfig.h"
#include "RoboticsProject/reset.h"
#include "RoboticsProject/reset_general.h"




class Pub_sub_odometry_core{

private:
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher odom_pub;
    tf2_ros::TransformBroadcaster odom_broadcaster;
    ros::ServiceServer resetZeroService;
    ros::ServiceServer resetGeneralService;

    ros::Time lastTime;
    double x,y,th;
    

    int integrationType;
    int skip;

public:
    Pub_sub_odometry_core() {
        sub = n.subscribe("/cmd_vel", 1, &Pub_sub_odometry_core::computeOdometry, this);
        odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 1);
        skip = 0;

        resetZeroService = n.advertiseService("reset_zero" , &Pub_sub_odometry_core::resetZero, this);
        resetGeneralService = n.advertiseService("reset_general" , &Pub_sub_odometry_core::resetGeneral, this);

        //Reads param from launch file
        n.getParam("/InitialX", x);
        n.getParam("/InitialY", y);
        n.getParam("/InitialTheta", th);
        integrationType = 0;
    }

    void computeOdometry(const geometry_msgs::TwistStamped::ConstPtr &msg){
        double vx, vy, vth, dt;
        ros::Time currentTime;
        currentTime = msg->header.stamp;

        if(skip){
            vx = msg->twist.linear.x;
            vy = msg->twist.linear.y;
            vth = msg->twist.angular.z;

            //Reads currentTime from message's header
            
            dt = (currentTime - lastTime).toSec();
            //std::cout << dt << std::endl;

            if(integrationType){//runge kutta
                x += (vx * cos(th +  vth * dt / 2) - vy * sin(th +  vth * dt / 2)) * dt;
                y += (vx * sin(th +  vth * dt / 2) + vy * cos(th +  vth * dt / 2)) * dt;
                th += vth * dt;
            }else{//euler
                x += (vx * cos(th) - vy * sin(th)) * dt;
                y += (vx * sin(th) + vy * cos(th)) * dt;
                th += vth * dt;
            }
            

            nav_msgs::Odometry odometry;
            tf2::Quaternion q;
            q.setRPY(0,0,th);

            odometry.header.stamp = currentTime;
            odometry.header.frame_id = "odom";
            odometry.pose.pose.position.x = x;
            odometry.pose.pose.position.y = y;
            odometry.pose.pose.position.z = 0;
            odometry.pose.pose.orientation.z = q.z();

            odometry.child_frame_id = "baseLink";

            odometry.twist.twist.linear.x = vx;
            odometry.twist.twist.linear.y = vy;
            odometry.twist.twist.linear.z = 0;
            
            odometry.twist.twist.angular.z = th;

            geometry_msgs::TransformStamped odometryTransformation;
            geometry_msgs::Quaternion odometryQuaternon2 = tf::createQuaternionMsgFromYaw(th);

            odometryTransformation.header.stamp = currentTime;
            odometryTransformation.header.frame_id = "odom";
            odometryTransformation.child_frame_id = "base_link";
            odometryTransformation.transform.translation.x = x;
            odometryTransformation.transform.translation.y = y;
            odometryTransformation.transform.translation.z = 0;

            odometryTransformation.transform.rotation = odometryQuaternon2;

            odom_broadcaster.sendTransform(odometryTransformation);
            odom_pub.publish(odometry);
        }
        lastTime = currentTime;
        skip = 1;
    }

    void setIntegration(RoboticsProject::parameters &config){
        integrationType = config.integration;
    }

    bool resetZero(RoboticsProject::reset::Request  &req,
                   RoboticsProject::reset::Response &res)
    {
        x = 0;
        y = 0;
        ROS_INFO("x reset to: %f", x);
        ROS_INFO("y reset to: %f", y);
        return true;
    }

    bool resetGeneral(RoboticsProject::reset_general::Request  &req,
                      RoboticsProject::reset_general::Response &res)
    {
        x = req.x;
        y = req.y;
        th = req.th;
        ROS_INFO("x reset to: %f", x);
        ROS_INFO("y reset to: %f", y);
        ROS_INFO("th reset to: %f", th);
        return true;
    }
};
int main(int argc, char **argv) {
    ros::init(argc, argv, "OdomentryCore");
    Pub_sub_odometry_core pubSubOdometry;

    dynamic_reconfigure::Server<RoboticsProject::parameters> server;
    dynamic_reconfigure::Server<RoboticsProject::parameters>::CallbackType f;

    f = boost::bind(&Pub_sub_odometry_core::setIntegration, &pubSubOdometry, _1);
    server.setCallback(f);

    ros::spin();
    return 0;
}

