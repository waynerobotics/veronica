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

int originX;
int originY;

//copy the supplied costmap to a new _map we can access freely
void map_handler(const nav_msgs::OccupancyGridPtr &costmap)
{
    static bool init_complete = false;
    //only do this stuff the first time a map is recieved.
 //   if (init_complete == false)
 //   {
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
        if(_map->data.size() !=  costmap->data.size()){
            _map->data.resize(costmap->data.size());
        }
        _map->data = costmap->data;


            //origins in cells instead of meters.
            originX = _map->info.origin.position.x / _map->info.resolution;
            originY = _map->info.origin.position.y / _map->info.resolution;



            if (!init_complete)
            {
                cout << "Map recieved. Initializing _map size "
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
    nav_msgs::Path newPath;
    newPath.header.frame_id = "map";
    newPath.header.stamp = ros::Time::now();
    newPath.poses.resize(path.poses.size());
    newPath.poses = path.poses;

    //is there an obstacle between start and cell we're checking
    bool obstacle_on_line = true;

    // path[0] = index of goal cell in path[] (not a map index - just index in the vector)
    int furthestFreeCell = newPath.poses.size()-1;

 //   for (int i = 0; i < path.poses.size(); i++){
 //       _map->data[getIndex(originX, originY, path.poses[i].pose.position.x / _map->info.resolution, 
 //                           path.poses[i].pose.position.y / _map->info.resolution, _map)] = 100;
 //   }

        //starting at last goal (path[0]) and checking each waypoint until we find clear straight line to a cell
    while (obstacle_on_line == true &&   newPath.poses[furthestFreeCell--] != path.poses.front())
    {
        cout<<"furthest free cell = "<<furthestFreeCell<<" and val = "
            <<(int)_map->data[getIndex(originX, originY, newPath.poses[furthestFreeCell].pose, _map)] <<endl;

        //we're going to iterate between points. set our start and endpoints for iterating
        int startX, endX, startY, endY;
        if (getX(newPath.poses[0].pose, _map) <= getX(newPath.poses[furthestFreeCell].pose, _map) )
        //    newPath.poses[0].pose.position.x <= path.poses[furthestFreeCell].pose.position.x)
        {
            startX = getX(newPath.poses[0].pose, _map);
            endX = getX(newPath.poses[furthestFreeCell].pose, _map);
        }
        else
        {
            startX = getX(newPath.poses[furthestFreeCell].pose, _map);
            endX = getX(newPath.poses[0].pose, _map);
        }

        if (newPath.poses[0].pose.position.y <= path.poses[furthestFreeCell].pose.position.y)
        {
            startY = getY(newPath.poses[0].pose, _map);
            endY = getY(newPath.poses[furthestFreeCell].pose, _map);
        }
        else
        {
            startY = getY(newPath.poses[furthestFreeCell].pose, _map);
            endY = getY(newPath.poses[0].pose, _map);
        }

        //if any one thing in for loop detects an obstacle, this will be set back to true
        //if no obstacles detected, we have found an unobstructed straight line to a waypoint
        obstacle_on_line = false;
        //check every y for every x in range
        for (int x = startX; x != endX && obstacle_on_line == false; x++)
        {
            //make sure we don't try to calculate slope of vertical line where y is always the same
            if (startY == endY)
            {
                obstacle_on_line = is_obstacle(originX,  originY, x, startY, _map);
            }
            else
            {
                obstacle_on_line = is_obstacle(originX,  originY, x, 
                    (int)get_y_intercept(
                    newPath.poses[0].pose.position.x, 
                    newPath.poses[0].pose.position.y, 
                    newPath.poses[furthestFreeCell].pose.position.x, 
                    newPath.poses[furthestFreeCell].pose.position.y, 
                    x),  _map);
            }
        }
        //check every x for every y in range
        for (int y = startY; y != endY && obstacle_on_line == false; y++)
        {
            //handle horizontal lines
            if (startX == endX)
            {
                obstacle_on_line = is_obstacle(originX,  originY, startX, y, _map);
            }
            else
            {
                obstacle_on_line = is_obstacle(originX,  originY, 
                (int)get_x_intercept(
                    newPath.poses[0].pose.position.x, 
                newPath.poses[0].pose.position.y, 
                newPath.poses[furthestFreeCell].pose.position.x, 
                newPath.poses[furthestFreeCell].pose.position.y, y),
                y, _map);
            }
        }
    }
    //if in a gray area, optimize might think the closest straight path is the current location
    //make sure at least one cell remains or planner will simply publish the start (current) and the robot won't move
    if(furthestFreeCell == 0)
    {
        furthestFreeCell = 1;
    }
  //  cout << "FOUND FURTHEST STRAIGHT LINE FROM START TO waypoint at x, y = " << path[furthestFreeCell].x << ", " << path[furthestFreeCell].y << endl;

    //pop cells off waypoint list until we get to furthestFreeCell
    while (furthestFreeCell !=0)
    {
      //  ###################erase elements here
        newPath.poses.erase(newPath.poses.begin()+ --furthestFreeCell);
    }
    //put start back for visualization waypoint list
  //  path.push_back(cell(start));
    pathPub.publish(newPath);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planb_path_optimizer");
    ros::NodeHandle node;

    //start tf::listener early so it has time to fill buffer
    //update_start_cell();

    //subscribe to _map and goal location
    subMap = node.subscribe("costmap", 1, map_handler);
    subPath = node.subscribe("/move_base/TrajectoryPlannerROS/global_plan", 0, optimize);

    //advertise publisher
    wptPub = node.advertise<geometry_msgs::PoseStamped>("waypoint_2d", 0);
    pathPub = node.advertise<nav_msgs::Path>("planb_path", 0);
    mapPub = node.advertise<nav_msgs::OccupancyGrid>("copied_map", 0);

    ros::Rate loop_rate(1);
    while (ros::ok())
    {

        ros::spinOnce();
        mapPub.publish(_map);
        loop_rate.sleep();
    }

    return 0;
}