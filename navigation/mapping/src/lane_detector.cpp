/*
This is the vision node that detects the lanes. It publishes raw lane data 
lane_to_laser.cpp converts to a psuedo laser scan


Subscribes to:
(type and topic name)

sensor_msgs/Image  /image_raw


Publishes:
(type and topic name)

nav_msgs/OccupancyGrid  /lanes? 


*/

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

ros::Publisher lanes_pub;
nav_msgs::OccupancyGrid lanes;


using namespace std;



void update_image(const sensor_msgs::Image &image)
{
}

void update_odom(const geometry_msgs::PoseWithCovarianceStamped &odom)
{

}

int main(int argc, char **argv)
{
  

    //handshake with ros master and create node object
    ros::init(argc, argv, "lane_detector");
    ros::NodeHandle node;

    //Subscribe to topics
    ros::Subscriber subImage = node.subscribe("usb_cam/image_raw", 1, update_image);
    ros::Subscriber subodom = node.subscribe("robot_pose_ekf/odom_combined", 1, update_odom);

    lanes_pub = node.advertise<nav_msgs::OccupancyGrid>("lanes", 10);



    ros::Rate loop_rate(10);
    while(ros::ok)
    {
        ros::spinOnce();
   
        loop_rate.sleep();
    }

    return 0;
}