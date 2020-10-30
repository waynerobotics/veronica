/*
This is the slam node that creates the map.  


Subscribes to:
(type and topic name)

Sensor_msgs/LaserScan  /merged_scan
Geometry_Msgs/PoseWithCovarianceStamped  /robot_pose_ekf/odom_combined

tf odom -> base_link



Publishes:
(type and topic name)

nav_msgs/OccupancyGrid  /map 

tf map -> odom

*/

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

ros::Publisher ogm_pub;


using namespace std;



void update_scan(const sensor_msgs::LaserScan &scan)
{
}

void update_odom(const geometry_msgs::PoseWithCovarianceStamped &odom)
{

}

int main(int argc, char **argv)
{
  

    //handshake with ros master and create node object
    ros::init(argc, argv, "gmapper");
    ros::NodeHandle node;

    //Subscribe to topics
    ros::Subscriber subScan = node.subscribe("merged_scan", 1, update_scan);
    ros::Subscriber subodom = node.subscribe("robot_pose_ekf/odom_combined", 1, update_odom);

    ogm_pub = node.advertise<nav_msgs::OccupancyGrid>("map", 10);



    ros::Rate loop_rate(10);
    while(ros::ok)
    {
        ros::spinOnce();
   
        loop_rate.sleep();
    }

    return 0;
}