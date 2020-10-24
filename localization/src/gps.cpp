/*
this node reads and publishes GPS data
*/

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include <iostream>

//create Imu message object
sensor_msgs::NavSatFix gps;

using namespace std;




int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_publisher");
  ros::NodeHandle node;
  ros::Publisher pub = node.advertise<sensor_msgs::NavSatFix>("gps/fix", 1);

  

  ros::Rate loop_rate(10);
  while(ros::ok)
  {
    pub.publish(gps);
    loop_rate.sleep();
  }


  return 0;
}