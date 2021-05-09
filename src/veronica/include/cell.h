#ifndef PRACTICAL_COMMON_H_
#define PRACTICAL_COMMON_H_

#include <stdint.h>


//we'll use these points internally for our lists
struct cell
{
    cell() : index(-1), x(-1), y(-1), theta(-1), F(INT32_MAX), G(INT32_MAX), H(INT32_MAX),
             prevX(-1), prevY(-1) {}
    cell(const cell &incoming);

    int index; //the index in the nav_msgs::OccupancyGrid
    int x;     //x, y as grid cells are pose in meters/mapResolution (10)
    int y;
    double theta; //the final waypoint is the goal and requires heading theta
    int F;        //this cells total cost, will be calculated as G + H
    int G;        //cost (distancetraveled) from start(current position)
    int H;        //manhatten distance distance to target
    int prevX;    //map grid coordinates of previous cell
    int prevY;
};

//copy constructor
cell::cell(const cell &incoming)
{
    index = incoming.index;
    x = incoming.x;
    y = incoming.y;
    theta = incoming.theta;
    F = incoming.F;
    G = incoming.G;
    H = incoming.H;
    prevX = incoming.prevX;
    prevY = incoming.prevY;
}


#endif /* PRACTICAL_COMMON_H_ */