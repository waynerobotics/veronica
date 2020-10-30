/*
This is node converts raw lane data from lane_detector and publishes
the psuedo laser scan



Subscribes to:
(type and topic name)

nav_msgs/OccupancyGrid  /lanes? 
geometry_msgs::PoseWithCovarianceStamped    robot_pose_ekf/odom_combined

Publishes:
(type and topic name)

sensor_msgs/LaserScan  /visual_scan


*/
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

sensor_msgs::LaserScan visual_scan;

using namespace std;

void update_lanes(const nav_msgs::OccupancyGrid &lanes)
{
}

void update_odom(const geometry_msgs::PoseWithCovarianceStamped &odom)
{
}

int main(int argc, char **argv)
{

    //handshake with ros master and create node object
    ros::init(argc, argv, "lane_to_laser");
    ros::NodeHandle node;

    //Subscribe to topics
    ros::Subscriber subImage = node.subscribe("lanes", 1, update_lanes);
    ros::Subscriber subodom = node.subscribe("robot_pose_ekf/odom_combined", 1, update_odom);

      ros::Publisher pub = node.advertise<sensor_msgs::LaserScan>("visual_scan", 1);

    ros::Rate loop_rate(10);
    while (ros::ok)
    {
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}