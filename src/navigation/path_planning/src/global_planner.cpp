/*
This node subcribes to or calls the occupancy Grid map and makes our general global path plan/

Subcribes to:
(type and topic name)

nav_msgs/OccupancyGrid     /costmap_2d/costmap
geometry_msgs::PoseWithCovarianceStamped    robot_pose_ekf/odom_combined
geometry_msgs/PoseStamped  /goal_2d


Publishes:
(type and topic name)

nav_msgs/Path  /plan

*/


#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

ros::Publisher plan_pub;


using namespace std;



void update_wpt(const geometry_msgs::PoseStamped &wpt)
{
}

void update_odom(const geometry_msgs::PoseWithCovarianceStamped &odom)
{
}

void update_map(const nav_msgs::OccupancyGrid &map)
{
}


int main(int argc, char **argv)
{
  

    //handshake with ros master and create node object
    ros::init(argc, argv, "global_planner");
    ros::NodeHandle node;

    //Subscribe to topics
    ros::Subscriber subwpt = node.subscribe("goal_2d", 1, update_wpt);
    ros::Subscriber subodom = node.subscribe("robot_pose_ekf/odom_combined", 1, update_odom);
    ros::Subscriber subMap = node.subscribe("costmap", 1, update_map);

    plan_pub = node.advertise<nav_msgs::Path>("plan", 10);



    ros::Rate loop_rate(10);
    while(ros::ok)
    {
        ros::spinOnce();
   
        loop_rate.sleep();
    }

    return 0;
}