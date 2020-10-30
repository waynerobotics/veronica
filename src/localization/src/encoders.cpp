/*
*This is a ROS node that monitors a pair of hall effect encoders and publishes
*the tick counts for a left wheel and right wheel in ROS. Whether each
*GPIO event is incremented or decremented is determined by check the direction
*signal going to the motor driver. 
*/

#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include <iostream>

using namespace std;


//max and min allowable values
const int encoderMin = -32768;
const int encoderMax = 32768;

std_msgs::Int16 leftCount;
std_msgs::Int16 rightCount;


int main(int argc, char **argv)
{
  
    ros::init(argc, argv, "tick_publisher");
    ros::NodeHandle node;
    ros::Publisher pubLeft = node.advertise<std_msgs::Int16>("leftWheel", 1000);
    ros::Publisher pubRight = node.advertise<std_msgs::Int16>("rightWheel", 1000);


    ros::Rate loop_rate(10);
    while(ros::ok)
    {
        pubLeft.publish(leftCount);
        pubRight.publish(rightCount);
        ros::spinOnce();

        loop_rate.sleep();
    }
 
    return 0;
}