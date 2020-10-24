/*
This publishes the Sensor_msgs::LaserScan msg with the topic name /scan. 
This is a mock node and scan will be provided by the urg_node on the real robot

Publishes:
(type and topic name)
sensor_msgs/LaserScan   /scan


*/

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <iostream>

//create Imu message object
sensor_msgs::LaserScan scan;

using namespace std;




int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_pub");
  ros::NodeHandle node;
  ros::Publisher pub = node.advertise<sensor_msgs::LaserScan>("scan", 1);

  //set our range message fixed data
  scan.header.frame_id = "laser";

  ros::Rate loop_rate(10);
  while(ros::ok)
  {
    pub.publish(scan);
    loop_rate.sleep();
  }


  return 0;
}