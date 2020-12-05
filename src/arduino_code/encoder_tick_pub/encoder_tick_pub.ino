/*
encoder_tick_pub.ino is a ROS (Robot Operating System) publisher node that runs on
and publishes an std_msgs::Int16 (-32,768 to 32,767) wheel encoder ticks
for both left and right wheels. 

The QuadratureEncoderInt16.h file needs to be copied into a folder called QuadratureEncoderInt16 under the
libraries folder that is used by the Arduino environment so that it becomes accessible to Arduino programs.
Lloyd Brombach, November 2020
lbrombach2@gmail.com
*/

#include <ros.h>
#include "QuadratureEncoderInt16.h"
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

#define PUBLISH_INTERVAL 50
#define LEFT_ENCODER_A 3
#define LEFT_ENCODER_B 2
#define RIGHT_ENCODER_A 6
#define RIGHT_ENCODER_B 7

// The left quadrature encoder uses external interupts 0 and 1 which are associated with pins 2 and 3
QuadratureEncoder leftEncoder(LEFT_ENCODER_A, LEFT_ENCODER_B);
// The right quadrature encoder uses external interupts 5 and 4 which are associated with pins 18 and 19
QuadratureEncoder rightEncoder(RIGHT_ENCODER_A, RIGHT_ENCODER_B);

std_msgs::Int16 leftCount;
std_msgs::Int16 rightCount;

ros::NodeHandle nh;

//***Not clear which of these publisher declarations will work in arduno/rosserial. Try them both?***
//dont forget to comment out the publisher in setup() if changing
//ros::Publisher pubLeft = nh.advertise<std_msgs::Int16>("leftWheel", 1000);
//ros::Publisher pubRight = nh.advertise<std_msgs::Int16>("rightWheel", 1000);
ros::Publisher pubLeft("leftWheel", &leftCount);
ros::Publisher pubRight("rightWheel", &rightCount);

void setup()
{
    nh.initNode();
    nh.advertise(pubLeft);
    nh.advertise(pubRight);

    attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), HandleInterruptLeftA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_B), HandleInterruptLeftB, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), HandleInterruptRightA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_B), HandleInterruptRightB, CHANGE);

    //***If above interrupts don't work, try declaring this way: ***//
    //attachInterrupt(0, HandleInterruptLeftA, CHANGE); // Pin 2
    //attachInterrupt(1, HandleInterruptLeftB, CHANGE); // Pin 3
    //attachInterrupt(5, HandleInterruptRightA, CHANGE); // Pin 18
    //attachInterrupt(4, HandleInterruptRightB, CHANGE); // Pin 19
}

void loop()
{
    static long previousMilliseconds = 0;
    
    unsigned long currentMilliseconds = millis();
    unsigned long milliSecsSinceLastUpdate = abs(currentMilliseconds - previousMilliseconds);
    
    if (milliSecsSinceLastUpdate > PUBLISH_INTERVAL)
    {
        // save the last time we updated
        previousMilliseconds = currentMilliseconds;

        pubLeft.publish(&leftCount);
        pubRight.publish(&rightCount);
    }

    nh.spinOnce();
    delay(20);
}

void HandleInterruptLeftA()
{
    leftEncoder.OnAChanged();
    leftCount.data = (int)leftEncoder.getPosition();
}

void HandleInterruptLeftB()
{
    leftEncoder.OnBChanged();
    leftCount.data = (int)leftEncoder.getPosition();

}
void HandleInterruptRightA()
{
    rightEncoder.OnAChanged();
    rightCount.data = (int)rightEncoder.getPosition();
}

void HandleInterruptRightB()
{
    rightEncoder.OnBChanged();
    rightCount.data = (int)rightEncoder.getPosition();
}
