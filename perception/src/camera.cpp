/*
This will publish an image topic with the name image_raw for the vision nodes.
This temp/mock cpp file will actually be replaced with the usb_cam node.

Publishes:
(type and topic name)
sensor_msgs/image /usb_cam/image_raw

*/

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <iostream>

//create Imu message object
sensor_msgs::Image image;

using namespace std;




int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_pub");
  ros::NodeHandle node;
  ros::Publisher pub = node.advertise<sensor_msgs::Image>("usb_cam/image_raw", 1);



  ros::Rate loop_rate(10);
  while(ros::ok)
  {
    pub.publish(image);
    loop_rate.sleep();
  }


  return 0;
}