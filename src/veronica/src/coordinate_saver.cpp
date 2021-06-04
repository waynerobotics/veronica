#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Path.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <cstdlib>
#include <math.h>
#include <iostream>
#include <iomanip>
#include <fstream>

using namespace std;

const string FILE_PATH = "/home/warriorrobotics/veronica/src/veronica/params/saved_coordinates.txt";
ofstream outFile;

sensor_msgs::NavSatFix fix;

tf::StampedTransform map_utm_tf;
nav_msgs::Path path;
bool gotMapOdomTransform = false;

bool update_map_utm_tf(){

    static tf::TransformListener listener;

    if(listener.canTransform("map","utm", ros::Time(0), NULL))
    {
        listener.lookupTransform("map", "utm", ros::Time(0), map_utm_tf);
        gotMapOdomTransform = true;
        return true;
    }
    else
    {
        cout<<"WAYPOINT SERVER UNABLE TO LOOKUP MAP -> UTM TRANSFORM "<<endl;
        gotMapOdomTransform = false;
        return false;
    }
}

void update_fix(sensor_msgs::NavSatFix newFix){
  fix = newFix;
}

void record(){
  static int i =1;
  outFile << i <<setprecision(8) <<" status " << fix.status.STATUS_GBAS_FIX << " .. service " << fix.status.service
          << " lat/long: " << fix.latitude << ", " << fix.longitude << endl
          << "          UTM->map transform " << map_utm_tf.getOrigin().getX() << ", " << map_utm_tf.getOrigin().getY() << endl;
  std::cout << i++ <<setprecision(8) << " status " << fix.status.STATUS_GBAS_FIX << " .. service " << fix.status.service
          << " lat/long: " << fix.latitude << ", " << fix.longitude << endl
          << "          UTM->map transform " << map_utm_tf.getOrigin().getX() << ", " << map_utm_tf.getOrigin().getY() << endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "coordinate_saver");
  ros::NodeHandle node;

    outFile.open(FILE_PATH.c_str());
    if(outFile.fail())
    {
    cout<<"waypoints.txt file not found."<<endl;
    system("pause");
    }

  update_map_utm_tf();
  //Subscribe to topics
  ros::Subscriber subDesiredPose = node.subscribe("fix", 1, update_fix, ros::TransportHints().tcpNoDelay());

  char x = 'n';
  ros::Rate loop_rate(10);
  while (ros::ok() && x!= 'q' &&x!='Q')
  {
    ros::spinOnce();
    update_map_utm_tf();

    cin >> x;

    if (gotMapOdomTransform)
    {
      record(); 
    }
     
    loop_rate.sleep();
  }

  outFile.close();
  return 0;
}
