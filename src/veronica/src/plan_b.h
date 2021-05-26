#ifndef PLAN_B_H_
#define PLAN_B_H_
#include "nav_msgs/OccupancyGrid.h"
#include <math.h>
#include <iostream>

//cells from other sources set above this will be considered 100% occupied
const int OCCUPIED_THRESHOLD = 55;


//x coordinate from index based on  index = ogm.info.width * (y + abs(originY)) + (x + abs(originX))
int getX(int originX, int index, const nav_msgs::OccupancyGridPtr &map)
{
  return (index % map->info.width) + abs(originX);
}
//occupancy grid x from coordinate pair in form of pose message based on  index = ogm.info.width * (y + abs(originY)) + (x + abs(originX))
int getX( geometry_msgs::Pose pose, const nav_msgs::OccupancyGridPtr &map){
  return (int) (pose.position.x / map->info.resolution);
}

//y coordinate from index based on  index = ogm.info.width * (y + abs(originY)) + (x + abs(originX))
int getY(int originY, int index, const nav_msgs::OccupancyGridPtr &map)
{
  return index / map->info.width + abs(originY);
}
//occupancy grid y from coordinate pair in form of pose message based on  index = ogm.info.width * (y + abs(originY)) + (x + abs(originX))
int getY( geometry_msgs::Pose pose, const nav_msgs::OccupancyGridPtr &map){
  return (int) (pose.position.y / map->info.resolution);
}


//occupancy grid vector index from x, y coordinates based on  index = ogm.info.width * (y + abs(originY)) + (x + abs(originX))
int getIndex(int originX, int originY, int x, int y, const nav_msgs::OccupancyGridPtr &map)
{
  return map->info.width * (y + abs(originY)) + (x + abs(originX)) ;
}

//occupancy grid vector index from coordinate pair (in pose msg) based on  index = ogm.info.width * (y + abs(originY)) + (x + abs(originX))
int getIndex(int originX, int originY, geometry_msgs::Pose pose, const nav_msgs::OccupancyGridPtr &map){
  return map->info.width * (getY(pose, map) + abs(originY)) + (getX(pose, map) + abs(originX)) ;
}



//helper to check if cell is marked unknown
bool is_unknown(int originX, int originY, int x, int y, const nav_msgs::OccupancyGridPtr &map)
{
  return ((int)map->data[getIndex( originX,  originY, x, y, map)] == -1);
}

//helper to check if cell is to be considered an obstacle
bool is_obstacle(int originX, int originY, int x, int y, const nav_msgs::OccupancyGridPtr &map)
{
  std::cout << "In isobstacle()  " << x << ", " << y  << std::endl;
  std::cout<< "index: "<<getIndex(originX, originY, x, y, map)<< "  and value = "<<(int)map->data[getIndex(originX, originY, x, y, map)] << std::endl;

  return ((int)map->data[getIndex( originX,  originY, x, y, map)] > OCCUPIED_THRESHOLD);
}

bool is_in_bounds(){
  //dsjfnakjngr;kargne;
 // awegrlnrewagkn
 //     awefgnkja;
 // gern;raggknh
}

//helper to return map resolution
double map_resolution(const nav_msgs::OccupancyGridPtr &map)
{
  return map->info.resolution;
}

//returns slope m from slope intercept formula y=m*x+b from two coordinate pairs
//don't forget all coordinates must be either pose in meters or grid cells numbers
// ( grid cell number = pose(in meters) / map_resolution  )
//DO NOT MIX POSE COORDINATES WITH GRID CELL COORDINATES - MAKE ALL THE SAME
double get_m(double x1, double y1, double x2, double y2)
{
  //****CAUTION< WILL THROW ERROR IF WE DIVIDE BY ZERO
  return (y1 - y2) / (x1 - x2);
}

// b as is the offset from slope intercept formula y=m*x+b
//for b = y-(m*x)
//DO NOT MIX POSE COORDINATES WITH GRID CELL COORDINATES - MAKE ALL THE SAME
double get_b(double x1, double y1, double x2, double y2)
{
  //  cannot divide by zer0
  if (x1 != x2)
  {
    return y1 - (get_m(x1, y1, x2, y2) * x1);
  }
  else
    return x1; //line is vertical, so b = x1
}

//returns where y falls on a line between two supplied points, for the given x
//returns Y from slope intercept forumula y=m*x+b, given x
//DO NOT MIX POSE COORDINATES WITH GRID CELL COORDINATES - MAKE ALL THE SAME
//****DOES NOT HANDLE VERTICAL LINES****
double get_y_intercept(double x1, double y1, double x2, double y2, double checkX)
{
  std::cout << "IN get_y_intercept, checking " << x1 << ", " << y1 << " .. " << x2 << ", " << y2 << " and checking X= " << checkX<< std::endl;
  double m = get_m(x1, y1, x2, y2);
  double b = get_b(x1, y1, x2, y2);
  return m * checkX + b;
}


//returns where y falls on a line between two supplied points, for the given y
//returns x from slope intercept forumula y=m*x+b, given y. for x= (y-b)/m
//DO NOT MIX POSE COORDINATES WITH GRID CELL COORDINATES - MAKE ALL THE SAME
//****DOES NOT HANDLE VERTICAL LINES****
double get_x_intercept(double x1, double y1, double x2, double y2, double checkY)
{

  double m = get_m(x1, y1, x2, y2);
  double b = get_b(x1, y1, x2, y2);
  return (checkY - b) / m;
}

#endif /* #ifndef PLAN_B_H_ */