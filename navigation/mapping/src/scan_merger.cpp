/*
This is node combines both the actual laser scan from the LIDAR and the pseudo scan from lane_detector
into a single scan message - as required by gmapping



Subscribes to:
(type and topic name)

sensor_msgs/LaserScan  /scan
sensor_msgs/LaserScan  /visual_scan

Publishes:
(type and topic name)

sensor_msgs/LaserScan  /merged_scan


*/


#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

sensor_msgs::LaserScan merged_scan;

using namespace std;

void update_scan(const sensor_msgs::LaserScan &scan)
{
}

void update_visual(const sensor_msgs::LaserScan &scan)
{
}

int main(int argc, char **argv)
{

    //handshake with ros master and create node object
    ros::init(argc, argv, "scan_merger");
    ros::NodeHandle node;

    //Subscribe to topics
    ros::Subscriber subScan = node.subscribe("scan", 1, update_scan);
    ros::Subscriber subVisualScan = node.subscribe("visual_scan", 1, update_visual);

      ros::Publisher pub = node.advertise<sensor_msgs::LaserScan>("merged_scan", 1);

    ros::Rate loop_rate(10);
    while (ros::ok)
    {
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}