/*
*This mock node to be replaced by the costmap_2d package*
converts raw occupancy grid as supplied by either gmapping or map_server and applies
obstacle inflation to supply a costmap for path planning

Subcribes to:
(type and topic name)

nav_msgs/OccupancyGrid  /map



Publishes:
(type and topic name)

nav_msgs/OccupancyGrid  /costmap_2d/costmap



*/
#include "ros/ros.h"
#include <nav_msgs/OccupancyGrid.h>

ros::Publisher ogm_pub;


using namespace std;



void update_scan(const nav_msgs::OccupancyGrid &map)
{
}



int main(int argc, char **argv)
{
  

    //handshake with ros master and create node object
    ros::init(argc, argv, "costmap_2d");
    ros::NodeHandle node;

    //Subscribe to topics
    ros::Subscriber subMap = node.subscribe("map", 1, update_scan);

    ogm_pub = node.advertise<nav_msgs::OccupancyGrid>("costmap", 10);



    ros::Rate loop_rate(10);
    while(ros::ok)
    {
        ros::spinOnce();
   
        loop_rate.sleep();
    }

    return 0;
}