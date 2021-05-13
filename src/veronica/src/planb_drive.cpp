/*
*planb_drive.cpp is an modified ROS node 
*(modified from p_drive_controller.cpp from the book Practical Robotics in C++).
*
*This node subscribes to the path and odom topics. When a new
*path (which is a vector collection of waypoints) is recieved, this node calculates a straight course to the 
*second waypoint in the list (the first is the starting point) without obstacle avoidance
*and publishes cmd_vel msgs to first turn to the waypoint, then go forward. If the angle drifts
*outside of "close enough" while enroute, the robot will stop and correct is heading before continuing. 
*If the heading is close but not perfect, the node should do some angle correction while driving toward the goal
*
*
*Author: Lloyd Brombach (lbrombach2@gmail.com)
*11/7/2019
*/

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <cstdlib>
#include <math.h>
#include <iostream>

using namespace std;

ros::Publisher pubVelocity;
nav_msgs::Odometry odom;
geometry_msgs::Twist cmdVel;
geometry_msgs::PoseStamped desired;
nav_msgs::Path path;
const double PI = 3.141592;
const double Ka = 0.25;
const double Kb = -.5; 
const double Klv = .5; //.4;
const double initialX = 0.0;
const double initialY = 0.0;
const double ANGULAR_TOLERANCE = .1;
const double ENROUTE_ANGULAR_TOLERANCE = .4; //this is "close enough" to be moving toward goal
const double DISTANCE_TOLERANCE = .1;
const double MAX_LINEAR_VEL = 2;

bool waypointActive = false;
bool odomInitialized = false;

void updatePose(const nav_msgs::Odometry &currentOdom)
{
  odom.pose.pose.position.x = currentOdom.pose.pose.position.x;
  odom.pose.pose.position.y = currentOdom.pose.pose.position.y;
  odom.pose.pose.orientation.x = currentOdom.pose.pose.orientation.x;
  odom.pose.pose.orientation.y = currentOdom.pose.pose.orientation.y;
  odom.pose.pose.orientation.z = currentOdom.pose.pose.orientation.z;
  odom.pose.pose.orientation.w = currentOdom.pose.pose.orientation.w;
  odomInitialized = true;
}

void updatePath(const nav_msgs::Path &_path)
{

  path.header.frame_id = _path.header.frame_id;
  path.header.stamp = _path.header.stamp;
  path.poses.resize(_path.poses.size());
  path.poses = _path.poses;

  //todo This might be where to add a curved trajectory - might be nudging around curve
  if (path.poses.size() > 1)
  {
    desired.pose.position.x = path.poses[1].pose.position.x;
    desired.pose.position.y = path.poses[1].pose.position.y;
    desired.pose.orientation.x = path.poses[1].pose.orientation.x;
    desired.pose.orientation.y = path.poses[1].pose.orientation.y;
    desired.pose.orientation.z = path.poses[1].pose.orientation.z;
    desired.pose.orientation.w = path.poses[1].pose.orientation.w;
  }
  else
  {
    desired.pose.position.x = path.poses[0].pose.position.x;
    desired.pose.position.y = path.poses[0].pose.position.y;
    desired.pose.orientation.x = path.poses[0].pose.orientation.x;
    desired.pose.orientation.y = path.poses[0].pose.orientation.y;
    desired.pose.orientation.z = path.poses[0].pose.orientation.z;
    desired.pose.orientation.w = path.poses[0].pose.orientation.w;
  }

  waypointActive = true;
  cout << "waypoint active set true" << endl;
}

double getDistanceError()
{
  double deltaX = desired.pose.position.x - odom.pose.pose.position.x;
  double deltaY = desired.pose.position.y - odom.pose.pose.position.y;
  cout << "Distance error = " << sqrt(pow(deltaX, 2) + pow(deltaY, 2)) << endl;
  return sqrt(pow(deltaX, 2) + pow(deltaY, 2));
}

double getEuler(const geometry_msgs::Pose &pose)
{
  double euler;
  tf::Quaternion q(
    odom.pose.pose.orientation.x,
    odom.pose.pose.orientation.y,
    odom.pose.pose.orientation.z, 
    odom.pose.pose.orientation.w);

  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  return yaw;
}

double getAngularError()
{
  double odomEuler = getEuler(odom.pose.pose);

  double deltaX = desired.pose.position.x - odom.pose.pose.position.x;
  double deltaY = desired.pose.position.y - odom.pose.pose.position.y;
  double thetaBearing = atan2(deltaY, deltaX);
  double angularError = thetaBearing - odomEuler;
  angularError = (angularError > PI) ? angularError - (2 * PI) : angularError;
  angularError = (angularError < -PI) ? angularError + (2 * PI) : angularError;
  cout << "odomHeading: "<<odomEuler<< "  ...  angular error = " << angularError << endl;
  return angularError;
}

void set_velocity()
{
  cmdVel.linear.x = 0;
  cmdVel.linear.y = 0;
  cmdVel.linear.z = 0;
  cmdVel.angular.x = 0;
  cmdVel.angular.y = 0;
  cmdVel.angular.z = 0;

  static bool angle_met = true;
  static bool location_met = true;
  static bool got_zero = false;
  double final_desired_heading_error = getEuler(desired.pose) - getEuler(odom.pose.pose);

  cout << "set_vel step1: distError = " << getDistanceError() <<"  and FINAL desired heading error: " << final_desired_heading_error<< endl;
  if (abs(getDistanceError()) >= DISTANCE_TOLERANCE / 2 && got_zero == false) //got_zero is a flag that I have made it the waypoint and have stopped - it is time to pivot to goal pose
  {
    location_met = false;
  }
  else if (abs(getDistanceError()) < DISTANCE_TOLERANCE/3)
  {
    location_met = true;
    if (got_zero == false)
    {
      pubVelocity.publish(cmdVel); //publish stop cmd
      if (odom.twist.twist.linear.x == 0 && odom.twist.twist.angular.z == 0)
      {
        got_zero = true;
      }
      else
      {
        return;
      }
    }
  }

  cout << "set_vel step1: angularError = " << getAngularError() << endl;

  double angularError = (location_met == false) ? getAngularError() : final_desired_heading_error;
  if (abs(angularError) > ANGULAR_TOLERANCE)
  {
    angle_met = false;
  }
  else if (abs(angularError) < ANGULAR_TOLERANCE/2)
  {
    angle_met = true;
  }

  if (waypointActive == true && angle_met == false)
  {
    cout << "anglestuff 1" << endl;
    cmdVel.angular.z = Ka * angularError;
    cmdVel.linear.x = (abs(angularError) > ENROUTE_ANGULAR_TOLERANCE) ? 0 : Klv * getDistanceError() / 2;
  }
  else if (waypointActive == true && abs(getDistanceError()) >= DISTANCE_TOLERANCE && location_met == false)
  {
        cout << "anglestuff 2" << endl;

    cmdVel.linear.x = Klv * getDistanceError();
    cmdVel.angular.z = (Ka * angularError);
  }
  else
  {
    cout << "********I'm HERE, now set final desired heading! **********" << endl;
    location_met = true;
  }

  if (location_met && abs(final_desired_heading_error) < ANGULAR_TOLERANCE)
  {
    cout << "Target Achieved desired heading : odomHeading = " << getEuler(desired.pose)<<" : " << getEuler(odom.pose.pose) <<endl;
    waypointActive = false;
    got_zero = false;
  }

  pubVelocity.publish(cmdVel);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planb_drive_controller");
  ros::NodeHandle node;

  //Subscribe to topics
  ros::Subscriber subCurrentPose = node.subscribe("odom", 10, updatePose, ros::TransportHints().tcpNoDelay());
  ros::Subscriber subDesiredPose = node.subscribe("planb_path", 1, updatePath, ros::TransportHints().tcpNoDelay());
  pubVelocity = node.advertise<geometry_msgs::Twist>("planb_cmd_vel", 1);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    if (odomInitialized && waypointActive)
    {
      set_velocity();
    }
    cout << "goal = " << desired.pose.position.x << ", " << desired.pose.position.y << endl
         << "current x,y = " << odom.pose.pose.position.x << ", " << odom.pose.pose.position.y << endl
         << "  Distance error = " << getDistanceError() << endl;
    cout << "cmd_vel = " << cmdVel.linear.x << " ,  " << cmdVel.angular.z << endl;
    loop_rate.sleep();
  }

  return 0;
}
