/*
This node subcribes to the global plan and also considers local obstacles

Subcribes to:
(type and topic name)

nav_msgs/Path  /plan
nav_msgs/OccupancyGrid  /costmap_2d/costmap
nav_msgs/Odometry       /odom
geometry_msgs/PoseStamped  /goal_2d

Publishes:
(type and topic name)

move_base_msgs/MoveBaseActionGoal  move_base/goal
*/

#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

ros::Publisher goal_pub;

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

void update_plan(const nav_msgs::Path &map)
{
}


int main(int argc, char **argv)
{

    //handshake with ros master and create node object
    ros::init(argc, argv, "local_planner");
    ros::NodeHandle node;

    //Subscribe to topics
    ros::Subscriber subWpt = node.subscribe("goal_2d", 1, update_wpt);
    ros::Subscriber subodom = node.subscribe("robot_pose_ekf/odom_combined", 1, update_odom);
    ros::Subscriber subMap = node.subscribe("costmap", 1, update_map);
    ros::Subscriber subPlan = node.subscribe("plan", 1, update_plan);

    goal_pub = node.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal", 10);

    ros::Rate loop_rate(10);
    while (ros::ok)
    {
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}