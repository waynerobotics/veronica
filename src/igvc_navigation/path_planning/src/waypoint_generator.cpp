/*
This node generates our next goal

Subcribes to:
(type and topic name)
???????
???????


Publishes:
(type and topic name)
geometry_msgs/PoseStamped  /goal_2d
*/


#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>

ros::Publisher wpt_pub;


using namespace std;



int main(int argc, char **argv)
{
  

    //handshake with ros master and create node object
    ros::init(argc, argv, "waypoint_gen");
    ros::NodeHandle node;

    //Subscribe to topics
    wpt_pub = node.advertise<geometry_msgs::PoseStamped>("goal_2d", 1);



    ros::Rate loop_rate(10);
    while(ros::ok)
    {
        ros::spinOnce();
   
        loop_rate.sleep();
    }

    return 0;
}