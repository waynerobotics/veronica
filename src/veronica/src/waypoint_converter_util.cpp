/*
* converts waypoints in waypoints.txt to from Map x, y to UTM or UTM to Map x, y
* and prints both to screen (entire list). 
* ***Requires GPS and navsat_transform running (for map->UTM transform)***
* todo - add read user input for instant conversions
*/

#include "ros/ros.h"
#include "plan_b.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include <tf/transform_listener.h>
#include <vector>
#include <math.h>
#include <string.h>
#include <iostream>
#include <iomanip>
#include <fstream>

using namespace std;

const double CLOSE_ENOUGH_DISTANCE = .5; //meters
//const string FILE_PATH = "/home/warriorrobotics/veronica/src/veronica/params/waypoints.txt";
const string FILE_PATH = "/home/warriorrobotics/veronica/src/veronica/params/waypoints_to_convert.txt";
ros::Publisher wptPub;
ros::Subscriber subMap;
tf::StampedTransform map_base_tf;
tf::StampedTransform map_odom_tf;
tf::StampedTransform utm_map_tf;

vector<geometry_msgs::PoseStamped> waypoints;
bool waypointReached = false;

nav_msgs::OccupancyGrid::Ptr _map(new nav_msgs::OccupancyGrid());
int originX;
int originY;




bool update_utm_map_tf(){

    static tf::TransformListener listener;

    if(listener.canTransform("utm","map", ros::Time(0), NULL))
    {
        listener.lookupTransform("utm", "map", ros::Time(0), utm_map_tf);
        return true;
    }
    else
    {
        cout<<"WAYPOINT SERVER UNABLE TO LOOKUP UTM -> MAP TRANSFORM "<<endl;
        return false;
    }
}



//converts utm frame pose coordinates to map frame coordinates
geometry_msgs::PoseStamped utmToMap(geometry_msgs::PoseStamped utmPose){
  geometry_msgs::PoseStamped mapPose = utmPose;
  mapPose.header.frame_id = "map";
  double deltaX = utmPose.pose.position.x - utm_map_tf.getOrigin().getX();
  double deltaY = utmPose.pose.position.y - utm_map_tf.getOrigin().getY();
  double distance = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
  double tfRads = utm_map_tf.getRotation().getAngle();
  double thetaBearing = atan2(deltaY, deltaX) - 1.57;  //this 1.57 we are subtracting should possibly be the tfRads angle.. ??
  thetaBearing = (thetaBearing < -3.14) ? thetaBearing + 6.283184 : thetaBearing;
  cout << "thetaBearing = "<<thetaBearing<< " .. ";
  // cout<<"in deltax, deltay                 == "<<deltaX<<" .. "<<deltaY<<endl;
  // cout<<"in tfConversion dis, thetaBearing == "<<distance<<" .. "<<thetaBearing<<endl;

  mapPose.pose.position.x = distance * cos(thetaBearing);
  mapPose.pose.position.y = distance * sin(thetaBearing);

 // cout<< "utm->map tf = "<<tfRads<<"rads..." <<utm_map_tf.getOrigin().getX()<<", "<<utm_map_tf.getOrigin().getY()<<endl;
  return mapPose;
}

//converts utm frame pose coordinates to map frame coordinates
geometry_msgs::PoseStamped mapToUtm(geometry_msgs::PoseStamped mapPose){
  geometry_msgs::PoseStamped utmPose = mapPose;
  utmPose.header.frame_id = "utm";
  double deltaX = mapPose.pose.position.x;
  double deltaY = mapPose.pose.position.y;
  double distance = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
  double tfRads = utm_map_tf.getRotation().getAngle();
  double thetaBearing = atan2(deltaY, deltaX) + 1.57;  //this 1.57 we are subtracting should possibly be the tfRads angle.. ??
  thetaBearing = (thetaBearing > 3.14) ? thetaBearing - 6.283184 : thetaBearing;
  cout << "thetaBearing = "<<thetaBearing<< " .. ";

                         utmPose.pose.position.x = distance * cos(thetaBearing) + utm_map_tf.getOrigin().getX();
  utmPose.pose.position.y = distance * sin(thetaBearing) + utm_map_tf.getOrigin().getY();

 // cout<< "utm->map tf = "<<tfRads<<"rads..." <<utm_map_tf.getOrigin().getX()<<", "<<utm_map_tf.getOrigin().getY()<<endl;
  return utmPose;
}

//call method to get whatever 
void convert(geometry_msgs::PoseStamped temp){
  geometry_msgs::PoseStamped map;
  geometry_msgs::PoseStamped utm;
  static int i = 1;
  if (temp.header.frame_id == "utm")
  {
    utm = temp;
    map = utmToMap(temp);
  }
  else if (temp.header.frame_id == "map")
  {
    map = temp;
    utm = mapToUtm(temp);
  }
  else
  {
    cout << "INVALID FRAME ID: "<<temp.header.frame_id << endl;
  }
  cout <<setprecision(8) << i++<< ". Original frame: " << temp.header.frame_id << "  map coordinates : "
       << map.pose.position.x << ", " << map.pose.position.y << "   and UTM : "
       << utm.pose.position.x << ", " << utm.pose.position.y << endl;

  if(temp.header.frame_id == "map"){
    convert(utm);
  }
}



bool readFile()
{
    geometry_msgs::PoseStamped temp;

    ifstream inFile;
    inFile.open(FILE_PATH.c_str());
    if(inFile.fail())
    {
    cout<<"waypoints.txt file not found."<<endl;
    system("pause");
    return false;
    }

    while(1) 
    {
      inFile.ignore(50, '#');
      inFile.ignore(50, '\n');
      inFile>>temp.header.frame_id;
      inFile.ignore();
      inFile >> temp.pose.position.x;
      inFile >> temp.pose.position.y;
      temp.pose.position.z = 0;
      temp.pose.orientation.x = 0;
      temp.pose.orientation.y = 0;
      temp.pose.orientation.z = 0;
      temp.pose.orientation.w = 1;
      if (inFile.eof())
      {
        break;
      }
      
      waypoints.insert(waypoints.begin(), temp);
    }
    inFile.close();
    return true;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint_server");
    ros::NodeHandle node;
    update_utm_map_tf();

    readFile();

 
   ros::Rate loop_rate(2);
   while(!update_utm_map_tf() && ros::ok()){loop_rate.sleep();}

   cout << "Current map - > UTM transform :  "
        << utm_map_tf.getOrigin().getX() << ", "
        << utm_map_tf.getOrigin().getY() << "  theta from due east: "
        <<utm_map_tf.getRotation().getAngle() << endl;

   geometry_msgs::PoseStamped nextWaypoint;
   while (!waypoints.empty())
   {

       nextWaypoint = waypoints.back();
       convert(nextWaypoint);
       waypoints.pop_back();
   }
   cout << "fin." << endl << endl;
   return 0;
}