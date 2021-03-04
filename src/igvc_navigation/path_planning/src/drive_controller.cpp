/*
this node accepts the goal from the local planner and the current location and calculates 
the appropriate cmd_vel values

Subcribes to:
(type and topic name)

geometry_msgs::PoseWithCovarianceStamped    robot_pose_ekf/odom_combined
move_base_msgs/MoveBaseActionGoal  move_base/goal

Publishes:
(type and topic name)

geometry_msgs/Twist  cmd_vel
*/

#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

ros::Publisher wpt_pub;

using namespace std;

void update_goal(const move_base_msgs::MoveBaseActionGoal &odom)
{
}

void update_odom(const geometry_msgs::PoseWithCovarianceStamped &odom)
{
}

int main(int argc, char **argv)
{

    //handshake with ros master and create node object
    ros::init(argc, argv, "drive_controller");
    ros::NodeHandle node;

    //Subscribe to topics
    ros::Subscriber subodom = node.subscribe("robot_pose_ekf/odom_combined", 1, update_odom);
    ros::Subscriber subGoal = node.subscribe("move_base/goal", 1, update_goal);

    ros::Rate loop_rate(10);
    while (ros::ok)
    {
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}