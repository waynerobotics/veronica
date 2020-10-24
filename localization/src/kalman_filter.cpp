/*
*This is a version of encoder_odom_pub.cpp ROS node that subscribes to encoder count messages and publishes odometry
*on in a simple form where orientation.z is an euler angle, then publishes again
*on a topic that uses the quaternion version of the message. This version
*subscribes to an IMU topic and adjusts the heading of this node with the IMU heading data.
*This is intended to be used with an IMU like a BN0055 with a built-in fusion algorithm
*to improve odometry. This is written to be readable for all levels and accompanies the book Practical Robotics in C++.
*
*Author: Lloyd Brombach (lbrombach2@gmail.com)
*11/7/2019
*/

#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include "sensor_msgs/NavSatFix.h"

ros::Publisher odom_pub;
geometry_msgs::PoseWithCovarianceStamped odom;


using namespace std;

void Calc_Left(const std_msgs::Int16& lCount)
{

}

//calculate distance right wheel has traveled since last cycle
void Calc_Right(const std_msgs::Int16& rCount)
{

}

void update_heading(const sensor_msgs::Imu &imuMsg)
{
}

void update_fix(const sensor_msgs::NavSatFix &fix)
{

}

int main(int argc, char **argv)
{
  

    //handshake with ros master and create node object
    ros::init(argc, argv, "ekf");
    ros::NodeHandle node;

    //Subscribe to topics
    ros::Subscriber subForRightCounts = node.subscribe("rightWheel", 1, Calc_Right,ros::TransportHints().tcpNoDelay());
    ros::Subscriber subForLeftCounts = node.subscribe("leftWheel",1, Calc_Left,ros::TransportHints().tcpNoDelay());
    ros::Subscriber subImu = node.subscribe("imu/data_raw", 1, update_heading);
    ros::Subscriber subFix = node.subscribe("gps/fix", 1, update_fix);

    odom_pub = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("robot_pose_ekf/odom_combined", 10);



    ros::Rate loop_rate(10);
    while(ros::ok)
    {
        ros::spinOnce();
   
        loop_rate.sleep();
    }

    return 0;
}