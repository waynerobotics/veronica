/*
*path_optimizer.cpp
*
*This node is part of an alternative local/trajectory planner implementation (planb).
*This node will subsribe to the published global path and costmap, then eliminate points
*that lie in between the robots current location and the furthest point on the path where there 
*are no obstacles. It will then publish the new path as well as a waypoint that is the furthest point from 
*robot that can be reached by straight line travel
*
*Author: Lloyd Brombach (lbrombach2@gmail.com)
*11/7/2019
*/
#include "ros/ros.h"
#include "plan_b.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include <tf/transform_listener.h>
#include <vector>
#include <math.h>
#include <iostream>

using namespace std;

//create our subscriber and publisher.
ros::Subscriber subMap, subPath;
ros::Publisher wptPub;
ros::Publisher pathPub;
ros::Publisher mapPub;

//this is where we'll keep the working _map data and recieved global path
nav_msgs::OccupancyGrid::Ptr _map(new nav_msgs::OccupancyGrid());
nav_msgs::Path myPath;

const int LOOKAHEAD_NUM_CELLS = 16; //actual lookahead seems to be 1/2 of this number in cells?
int originX;
int originY;

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

            
            //this part we can do every time to ensure we see updates.
            //copy the contents of the costmap into the occupancy grid path_planner uses internally
            //starting at 0, 0. Data at x<0 or y<0 in the gmap is lost, so set start position accordingly
            //    for(int row = originY; row < _map->info.height ; row++)
            //    {
            //        for(int col = originX; col < _map->info.width; col++)
            //        {
            //            _map->data[getIndex(col-originY, row-originX, costmap)]
            //                        = costmap->data[getIndex(col, row, costmap)];
            //        }
            //    }
}

//optimize path with the latest maps we have
void optimize(const nav_msgs::Path &path){
    std::cout << "in optimize() 1 -  GOT NEW PATH" << endl;
    nav_msgs::Path newPath;
    newPath.header.frame_id = "map";
    newPath.header.stamp = ros::Time::now();
    newPath.poses.resize(path.poses.size());
    newPath.poses = path.poses;

    //is there an obstacle between start and cell we're checking
    bool obstacle_on_line = true;

    if(newPath.poses.size() < 2)
    {
        pathPub.publish(newPath);
    }
    else{

    
    // path[0] = index of goal cell in path[] (not a map index - just index in the vector)
    int furthestFreeCell = min((int)(newPath.poses.size()-1), LOOKAHEAD_NUM_CELLS);

 //   for (int i = 0; i < path.poses.size(); i++){
 //       _map->data[getIndex(originX, originY, path.poses[i].pose.position.x / _map->info.resolution, 
 //                           path.poses[i].pose.position.y / _map->info.resolution, _map)] = 100;
 //   }
    std::cout << "in optimize() 2: Path size and checking cell x:x  " << newPath.poses.size() << " : " << furthestFreeCell<< endl;

    //starting at last goal (path[0]) and checking each waypoint until we find clear straight line to a cell
   while (obstacle_on_line == true && furthestFreeCell > 0 && _map != NULL)
   {
       std::cout << "furthest free cell = " << furthestFreeCell << " and val = "
                 << (int)_map->data[getIndex(originX, originY, newPath.poses[furthestFreeCell].pose, _map)] << endl;
       std::cout << "   and x, y = " << getX(newPath.poses[furthestFreeCell].pose, _map)
       << ", " << getY(newPath.poses[furthestFreeCell].pose, _map) << endl;

       //we're going to iterate between points. set our start and endpoints for iterating
       int startX, endX, startY, endY;
       if (getX(newPath.poses[0].pose, _map) <= getX(newPath.poses[furthestFreeCell].pose, _map))
       //    newPath.poses[0].pose.position.x <= path.poses[furthestFreeCell].pose.position.x)
       {
           cout << "DEBUG 1" << endl;
           startX = getX(newPath.poses[0].pose, _map);
           endX = getX(newPath.poses[furthestFreeCell].pose, _map);
       }
       else
       {
           cout << "DEBUG 2" << endl;
           startX = getX(newPath.poses[furthestFreeCell].pose, _map);
           endX = getX(newPath.poses[0].pose, _map);
       }
           cout << "DEBUG 3" << endl;

       if (newPath.poses[0].pose.position.y <= path.poses[furthestFreeCell].pose.position.y)
       {           cout << "DEBUG 4" << endl;

           startY = getY(newPath.poses[0].pose, _map);
           endY = getY(newPath.poses[furthestFreeCell].pose, _map);
       }
       else
       {           cout << "DEBUG 5" << endl;

           startY = getY(newPath.poses[furthestFreeCell].pose, _map);
           endY = getY(newPath.poses[0].pose, _map);
       }
           cout << "DEBUG 6" << endl;

       //if any one thing in for loop detects an obstacle, this will be set back to true
       //if no obstacles detected, we have found an unobstructed straight line to a waypoint
       obstacle_on_line = false;
       //check every y for every x in range
       for (int x = startX; x != endX && obstacle_on_line == false; x++)
       {
           //make sure we don't try to calculate slope of vertical line where y is always the same
           if (startY == endY || newPath.poses[0].pose.position.y == newPath.poses[furthestFreeCell].pose.position.y)
           {
                cout << "DEBUG 7" << endl;
               obstacle_on_line = is_obstacle(originX, originY, x, startY, _map);
                cout << "DEBUG 7.2" << endl;                        
           }
           else
           {
                          cout << "DEBUG 8" << endl;

               obstacle_on_line = is_obstacle(originX, originY, x,
                                              (int)get_y_intercept(
                                                  newPath.poses[0].pose.position.x/_map->info.resolution,
                                                  newPath.poses[0].pose.position.y/_map->info.resolution,
                                                  newPath.poses[furthestFreeCell].pose.position.x,
                                                  newPath.poses[furthestFreeCell].pose.position.y,
                                                  x),
                                              _map);
           }
       }
                  cout << "DEBUG99" << endl;

       //check every x for every y in range
       for (int y = startY; y != endY && obstacle_on_line == false; y++)
       {
           //handle horizontal lines
           if (startX == endX)
           {
               obstacle_on_line = is_obstacle(originX, originY, startX, y, _map);
           }
           else
           {
               obstacle_on_line = is_obstacle(originX, originY,
                                              (int)get_x_intercept(
                                                  newPath.poses[0].pose.position.x,
                                                  newPath.poses[0].pose.position.y,
                                                  newPath.poses[furthestFreeCell].pose.position.x,
                                                  newPath.poses[furthestFreeCell].pose.position.y, y),
                                              y, _map);
           }
       }
       furthestFreeCell--;
    }

    //
    //make sure at least one cell remains or planner will simply publish the start (current) and the robot won't move
    if(furthestFreeCell <= 1 && newPath.poses.size() > 2)
    {
        furthestFreeCell = (newPath.poses.size() > 3) ? 3 : 1;
    }

  

    //erase cells between start and the first obstacle-on-path encounter 
    //(leaving element 0 is for visual purposes - serve the furthest free cell to the drive controller (now element[1]))
    if(furthestFreeCell > 1){
        newPath.poses.erase(newPath.poses.begin()+1, newPath.poses.begin()+furthestFreeCell);
    }
      std::cout << "FOUND FURTHEST STRAIGHT LINE FROM START TO waypoint publishing path starting at x, y = " 
    << path.poses[furthestFreeCell].pose.position.x << ", " 
    << path.poses[furthestFreeCell].pose.position.y << endl;
    
    pathPub.publish(newPath);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planb_path_optimizer");
    ros::NodeHandle node;

    //start tf::listener early so it has time to fill buffer
    //update_start_cell();

    //subscribe to _map and goal location
    subMap = node.subscribe("costmap", 1, map_handler);
    subPath = node.subscribe("/move_base/TrajectoryPlannerROS/global_plan", 1, optimize);

    //advertise publisher
    wptPub = node.advertise<geometry_msgs::PoseStamped>("waypoint_2d", 1);
    pathPub = node.advertise<nav_msgs::Path>("planb_path", 1);
    mapPub = node.advertise<nav_msgs::OccupancyGrid>("copied_map", 1);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {

        ros::spinOnce();
     //   cout << "Loop 1" << endl;
        if(_map != nullptr){
            mapPub.publish(_map);
        }
      //  cout << "Loop2" << endl;
        loop_rate.sleep();
    }

    return 0;
}