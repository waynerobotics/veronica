#include "ros/ros.h"
#include "plan_b.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include <tf/transform_listener.h>
#include <vector>
#include <math.h>
#include <string.h>
#include <iostream>
#include <fstream>

using namespace std;

const double CLOSE_ENOUGH_DISTANCE = 2; //meters
const string FILE_PATH = "/home/warriorrobotics/veronica/src/veronica/params/waypoints_south.txt";
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


void update_map_base_link_tf(){

    static tf::TransformListener listener;

    if(listener.canTransform("map","base_link", ros::Time(0), NULL))
    {
        listener.lookupTransform("map", "base_link", ros::Time(0), map_base_tf);
    }
    else
    {
        cout<<"WAYPOINT SERVER UNABLE TO LOOKUP MAP -> BASE_LINK TRANSFORM "<<endl;
    }
}

void update_map_odom_link_tf(){

    static tf::TransformListener listener;

    if(listener.canTransform("map","odom", ros::Time(0), NULL))
    {
        listener.lookupTransform("map", "odom", ros::Time(0), map_odom_tf);
    }
    else
    {
        cout<<"WAYPOINT SERVER UNABLE TO LOOKUP MAP -> ODOM TRANSFORM "<<endl;
    }
}

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
  update_utm_map_tf();
  geometry_msgs::PoseStamped mapPose = utmPose;
  mapPose.header.frame_id = "map";
  double deltaX = utmPose.pose.position.x - utm_map_tf.getOrigin().getX();
  double deltaY = utmPose.pose.position.y - utm_map_tf.getOrigin().getY();
  double distance = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
  double tfRads = utm_map_tf.getRotation().getAngle();
  double thetaBearing = atan2(deltaY, deltaX) - 1.57;
  //double mapX = distance * cos(thetaBearing);
  //double mapY = distance * sin(thetaBearing);
  cout<<"in deltax, deltay                 == "<<deltaX<<" .. "<<deltaY<<endl;
  cout<<"in tfConversion dis, thetaBearing == "<<distance<<" .. "<<thetaBearing<<endl;

  mapPose.pose.position.x = distance * cos(thetaBearing);
  mapPose.pose.position.y = distance * sin(thetaBearing);

  cout<< "utm->map tf = "<<tfRads<<"rads..." <<utm_map_tf.getOrigin().getX()<<", "<<utm_map_tf.getOrigin().getY()<<endl;
  return mapPose;
}

//todo make this lookup transform and return waypoint in map frame
geometry_msgs::PoseStamped getWaypointInMapFrame(geometry_msgs::PoseStamped temp){

  if(temp.header.frame_id == "utm"){
    return utmToMap(temp);
  }
  return temp;
}

bool isCloseENough(geometry_msgs::PoseStamped waypoint)
{
  double deltaX = waypoint.pose.position.x - map_base_tf.getOrigin().getX();
  double deltaY = waypoint.pose.position.y - map_base_tf.getOrigin().getY();
  cout << "waypoint_server Distance error = " << sqrt(pow(deltaX, 2) + pow(deltaY, 2)) << endl;
  return sqrt(pow(deltaX, 2) + pow(deltaY, 2)) < CLOSE_ENOUGH_DISTANCE;
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

//copy the supplied costmap to a new _map we can access freely
void map_handler(const nav_msgs::OccupancyGridPtr &costmap)
{
    static bool init_complete = false;
    //only do this stuff the first time a map is recieved.
 //   if (init_complete == false)
 //   {
    _map = costmap;
    _map->header.frame_id = costmap->header.frame_id;
    _map->info.resolution = costmap->info.resolution;
    _map->info.width = costmap->info.width;
    _map->info.height = costmap->info.height;
    _map->info.origin.position.x = costmap->info.origin.position.x;
    _map->info.origin.position.y = costmap->info.origin.position.y;
    _map->info.origin.orientation.x = costmap->info.origin.orientation.x;
    _map->info.origin.orientation.y = costmap->info.origin.orientation.y;
    _map->info.origin.orientation.z = costmap->info.origin.orientation.z;
    _map->info.origin.orientation.w = costmap->info.origin.orientation.w;
    if (_map->data.size() != costmap->data.size())
    {
        _map->data.resize(costmap->data.size());
        }
        _map->data = costmap->data;


            //origins in cells instead of meters.
            originX = _map->info.origin.position.x / _map->info.resolution;
            originY = _map->info.origin.position.y / _map->info.resolution;

            if (!init_complete)
            {
                std::cout << "Map recieved. Initializing _map size "
                     << _map->info.width << " x " << _map->info.height << " = " << costmap->data.size() << "  at resolution "
                     << _map->info.resolution << "\nOrigin: "
                     << _map->info.origin.position.x << ", " << _map->info.origin.position.y << endl
                     << "originx,y: " << originX << ", " << originY << endl;

                init_complete = true;
              }

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint_server");
    ros::NodeHandle node;
    update_utm_map_tf();

    readFile();

 //   for (int i = waypoints.size() -1; i >=0 ; i--){
  //      geometry_msgs::PoseStamped wpt = waypoints.back();
 //       waypoints.pop_back();
 //       cout << "ID:  " << wpt.header.frame_id << "   " << wpt.pose.position.x << ", " << wpt.pose.position.y << endl;
 //   }
    //start tf::listener early so it has time to fill buffer
    //update_start_cell();

    //subscribe to _map and pose
    subMap = node.subscribe("costmap", 1, map_handler);
    //subPath = node.subscribe("/move_base/TrajectoryPlannerROS/global_plan", 0, optimize);

    //advertise publisher
    wptPub = node.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
    //pathPub = node.advertise<nav_msgs::Path>("planb_path", 1);
   // mapPub = node.advertise<nav_msgs::OccupancyGrid>("copied_map", 1);
   ros::Rate loop_rate(2);
   while(!update_utm_map_tf()){loop_rate.sleep();}

    geometry_msgs::PoseStamped nextWaypoint;
 //   ros::Rate loop_rate(2);
    while (ros::ok() && !waypoints.empty() )
    {
        ros::spinOnce();
        update_map_base_link_tf();
        update_map_odom_link_tf();
        static bool toggled = false;

        if (!waypoints.empty())
        {
          nextWaypoint = waypoints.back();
          
          if(toggled){
            nextWaypoint.pose.position.x = nextWaypoint.pose.position.x + 0.11;
          }else{
            nextWaypoint.pose.position.x = nextWaypoint.pose.position.x - 0.11;            
          }
          toggled = !toggled;

          cout << "NEXT WPT at Frame_ID:  " << nextWaypoint.header.frame_id << "   " << nextWaypoint.pose.position.x << ", " << nextWaypoint.pose.position.y << endl;

          nextWaypoint = getWaypointInMapFrame(nextWaypoint);
          wptPub.publish(nextWaypoint);
          cout << "Publishing WPT at Frame_ID:  " << nextWaypoint.header.frame_id << "   " << nextWaypoint.pose.position.x << ", " << nextWaypoint.pose.position.y << endl;

          if(isCloseENough(nextWaypoint)){
           waypoints.pop_back();
           waypointReached = false;
          }
        }

        loop_rate.sleep();
    }
    cout<<"Waypoints list empty - exiting"<<endl;
    return 0;
}