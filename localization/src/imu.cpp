/*
Our IMU publisher node
*/

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <iostream>

//create Imu message object
sensor_msgs::Imu myImu;

using namespace std;




int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_publisher");
  ros::NodeHandle node;
  ros::Publisher pub = node.advertise<sensor_msgs::Imu>("imu/data_raw", 1);

  //set our range message fixed data
  myImu.header.frame_id = "imu";

  ros::Rate loop_rate(10);
  while(ros::ok)
  {
    pub.publish(myImu);
    loop_rate.sleep();
  }


  return 0;
}