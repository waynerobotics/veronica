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
ros::Publisher pubGoal; //see pubNewGoal() comments

nav_msgs::Odometry odom;
geometry_msgs::Twist cmdVel;
geometry_msgs::PoseStamped desired;
geometry_msgs::PoseStamped lastGoal;
nav_msgs::Path path;
const double PI = 3.141592;
const double Ka = 0.45;
const double Kb = -.5;
const double Klv = .5; //.4;
const double initialX = 0.0;
const double initialY = 0.0;
const double ANGULAR_TOLERANCE = .15;
const double ENROUTE_ANGULAR_TOLERANCE = .35; //this is "close enough" to be moving toward goal
const double DISTANCE_TOLERANCE = .15;
const double MAX_LINEAR_VEL = 1.5;
const double MIN_PIVOT_VELOCITY = 0.1;
const double MAX_ANGULAR_VELOCITY = 0.15;


bool waypointActive = false;
bool odomInitialized = false;
bool gotNewGoal = false;

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

//after reaching a waypoint, this is called to publish the original final goal in the path so
//the global planner and path optimizer provide a new path
void getNewPath()
{
  pubGoal.publish(path.poses.back());
  cout << "ASKING FOR NEW PATH TO: " << path.poses.back().pose.position.x << ", "<<
  path.poses.back().pose.position.y << endl;

}

void updatePath(const nav_msgs::Path &_path)
{

  path.header.frame_id = _path.header.frame_id;
  path.header.stamp = _path.header.stamp;
  path.poses.resize(_path.poses.size());
  path.poses = _path.poses;

  if(path.poses.back().pose.position.x != lastGoal.pose.position.x || 
    path.poses.back().pose.position.y != lastGoal.pose.position.y ){
    gotNewGoal = true;
  }else{
    gotNewGoal = false;
  }

  if (path.poses.size() > 1)
  {
    desired.pose.position.x = path.poses[1].pose.position.x;
    desired.pose.position.y = path.poses[1].pose.position.y;
    desired.pose.orientation.x = path.poses[1].pose.orientation.x;
    desired.pose.orientation.y = path.poses[1].pose.orientation.y;
    desired.pose.orientation.z = path.poses[1].pose.orientation.z;
    desired.pose.orientation.w = path.poses[1].pose.orientation.w;
    waypointActive = true;
    cout << "waypoint active set true" << endl;
  }
  else if (path.poses.size() == 1)
  {
    desired.pose.position.x = path.poses[0].pose.position.x;
    desired.pose.position.y = path.poses[0].pose.position.y;
    desired.pose.orientation.x = path.poses[0].pose.orientation.x;
    desired.pose.orientation.y = path.poses[0].pose.orientation.y;
    desired.pose.orientation.z = path.poses[0].pose.orientation.z;
    desired.pose.orientation.w = path.poses[0].pose.orientation.w;
    waypointActive = true;
    cout << "waypoint active set true" << endl;
  }
}

double getDistanceError()
{
  double deltaX = desired.pose.position.x - odom.pose.pose.position.x;
  double deltaY = desired.pose.position.y - odom.pose.pose.position.y;
  cout << "Distance error = " << sqrt(pow(deltaX, 2) + pow(deltaY, 2)) << endl;
  return sqrt(pow(deltaX, 2) + pow(deltaY, 2));
}

//returns Euler angle of the prodived pose orientation quaternion
double getEuler(const geometry_msgs::Pose &pose)
{
  double euler;
  tf::Quaternion q(
      //odom.pose.pose.orientation.x,  //oops, I think.
      //odom.pose.pose.orientation.y,
      //odom.pose.pose.orientation.z,
      //odom.pose.pose.orientation.w
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z,
      pose.orientation.w);

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
  cout << "odomHeading: " << odomEuler << "  ...  angular error = " << angularError << endl;
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

  //if our path size is greater than 2 (start and goal), then we ask the global planner
  //and path optimizer to recalculate and give us a new one.
  static double lastPathRequest = ros::Time::now().toSec();
  if (path.poses.size() > 2  && ros::Time::now().toSec() - lastPathRequest > 0.5
        && gotNewGoal == false )
  {
    getNewPath();
    lastPathRequest = ros::Time::now().toSec();
  }

  cout << "set_vel step1: distError = " << getDistanceError() << "  and FINAL desired heading error: " << final_desired_heading_error << endl;
  //not at position - keep moving
  if (abs(getDistanceError()) >= DISTANCE_TOLERANCE*2  && got_zero == false) //got_zero is a flag that I have made it the waypoint and have stopped - it is time to pivot to goal pose
  {
    location_met = false;
  }
  //close enough to goal. issue stop
  else if (abs(getDistanceError()) < DISTANCE_TOLERANCE )
  {
    location_met = true;
    if (got_zero == false)
    {
      pubVelocity.publish(cmdVel); //publish stop cmd

      if (odom.twist.twist.linear.x == 0 && odom.twist.twist.angular.z == 0)
      {
        got_zero = true; //wheels have stopped.
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
  else if (abs(angularError) < ANGULAR_TOLERANCE / 2)
  {
    angle_met = true;
  }

  //if not at waypoint AND not aiming toward waypoint
  if (waypointActive == true && angle_met == false)
  {
    cout << "anglestuff 1" << endl;
    cmdVel.angular.z = Ka * angularError;
    //if heading way off, pivot. If heading sorta close, move forward while turning
    cmdVel.linear.x = (abs(angularError) > ENROUTE_ANGULAR_TOLERANCE) ? 0 : Klv * getDistanceError() / 2;
  }
  //else if not at waypoint but heading is within tolerance (robot is aiming at waypoint)
  else if (waypointActive == true && abs(getDistanceError()) >= DISTANCE_TOLERANCE && location_met == false)
  {
    cout << "anglestuff 2" << endl;
  //go forward fully and keep tweaking heading toward it as well
    cmdVel.linear.x = Klv * getDistanceError();
    cmdVel.angular.z = (Ka * angularError / 2);
  }
  else //at waypoint, set final heading *****DISABLE SET FINAL HEADING FOR THIS VERSION -> NO HEADING DATA IN STANDARD PATH MSG *****
  {
    cout << "********I'm HERE, now set final desired heading! **********" << endl;
    location_met = true;
    waypointActive = false; // remove from here and uncomment if section directly below to reenable final heading pivots
    got_zero = false;       // remove from here and uncomment section below to reenable final heading pivots
  }

  // uncomment to reenable setting final heading, but need to somehow pass final heading because it is not present in path msg
  //if (location_met && abs(final_desired_heading_error) < ANGULAR_TOLERANCE)
 // {
  //  cout << "Target Achieved desired heading : odomHeading = " << getEuler(desired.pose) << " : " << getEuler(odom.pose.pose) << endl;
  //  waypointActive = false;
  //  got_zero = false;
  //}

    if(cmdVel.angular.z != 0 && abs(cmdVel.angular.z) < MIN_PIVOT_VELOCITY){
      cmdVel.angular.z = (cmdVel.angular.z > 0) ? MIN_PIVOT_VELOCITY : 0 - MIN_PIVOT_VELOCITY;
    }
    if(abs(cmdVel.angular.z) > MAX_ANGULAR_VELOCITY){
      cmdVel.angular.z = (cmdVel.angular.z > 0) ? MAX_ANGULAR_VELOCITY : 0 - MAX_ANGULAR_VELOCITY;
    }
  pubVelocity.publish(cmdVel);
}

void updateGoal(const geometry_msgs::PoseStamped &_goal)
{
  if(lastGoal.pose.position.x != _goal.pose.position.x ||
       lastGoal.pose.position.y != _goal.pose.position.y ){
    gotNewGoal = true;
    lastGoal.pose.position.x = _goal.pose.position.x;
    lastGoal.pose.position.y = _goal.pose.position.y;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planb_drive_controller");
  ros::NodeHandle node;
  lastGoal.pose.position.x = 99999;
  lastGoal.pose.position.y = 99999;
  //Subscribe to topics
  ros::Subscriber subCurrentPose = node.subscribe("odom", 10, updatePose, ros::TransportHints().tcpNoDelay());
  ros::Subscriber subDesiredPose = node.subscribe("planb_path", 1, updatePath, ros::TransportHints().tcpNoDelay());
  ros::Subscriber subGoal = node.subscribe("move_base_simple/goal", 1, updateGoal, ros::TransportHints().tcpNoDelay());
  pubVelocity = node.advertise<geometry_msgs::Twist>("planb_cmd_vel", 1);
  pubGoal = node.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);

  ros::Rate loop_rate(20);
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
