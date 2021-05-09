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
#include "practical_nav/practical_common.h"
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

//this is where we'll keep the working _map data
nav_msgs::OccupancyGrid::Ptr _map(new nav_msgs::OccupancyGrid());


//copy the supplied costmap to a new _map we can access freely
void map_handler(const nav_msgs::OccupancyGridPtr &costmap)
{
    static bool init_complete = false;
    //only do this stuff the first time a map is recieved.
    if (init_complete == false)
    {
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
        _map->data.resize(costmap->data.size());

        cout << "Map recieved. Initializing _map size "
             << _map->info.width << " x " << _map->info.height<<" = "<<costmap->data.size() <<"  at resolution "
             << _map->info.resolution<<"\nOrigin: "
             << _map->info.origin.position.x<<", "<< _map->info.origin.position.x<<endl;


        init_complete = true;
    }

    //origins in cells instead of meters.
    int originX = 1 - (_map->info.origin.position.x / _map->info.resolution);
    int originY = 1 - (_map->info.origin.position.y / _map->info.resolution);


    //this part we can do every time to ensure we see updates.
    //copy the contents of the costmap into the occupancy grid path_planner uses internally
    //starting at 0, 0. Data at x<0 or y<0 in the gmap is lost, so set start position accordingly
        for(int row = originY; row < _map->info.height ; row++)
        {
            for(int col = originX; col < _map->info.width; col++)
            {
                _map->data[getIndex(col-originY, row-originX, costmap)]
                            = costmap->data[getIndex(col, row, costmap)];
            }
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
    //subGoal = node.subscribe("goal_2d", 0, set_goal);

    //advertise publisher
    wptPub = node.advertise<geometry_msgs::PoseStamped>("waypoint_2d", 0);
    pathPub = node.advertise<nav_msgs::Path>("planb_path", 0);
    mapPub = node.advertise<nav_msgs::OccupancyGrid>("copied_map", 0);

    ros::Rate loop_rate(1);
    while (ros::ok())
    {

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}