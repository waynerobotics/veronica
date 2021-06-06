/*
this node reads and publishes GPS data
*/

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include <iostream>

//create Imu message object
sensor_msgs::NavSatFix fix;

using namespace std;

void updateFix(const sensor_msgs::NavSatFix &msg){
  fix = msg;
}

bool datumSet(){
  if(fix.status.status == 2){
    cout<<"GOT AUGMENTED FIX, PUBLISHING DATUM AT lat/long"<< setprecision(10) << fix.latitude<<", "<<fix.longitude<<endl;
  }
  else{
    cout<<"UNABLE TO PUBLISH AUGMENTED DATUM"<<endl;
  }


}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "gps_datum_setter");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("fix", 0, updateFix);

  

  ros::Rate loop_rate(2);
  while(ros::ok() && !datumSet())
  {
    ros::spinOnce();


    loop_rate.sleep();
  }


  return 0;
}

/*
  ros::NodeHandle n;
  //set_datum service  send lat long yaw.. yaw probably zero as offset is set in launch file
  ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("/move_base_node/clear_costmaps"); 
  std_srvs::Empty srv;
  if (client.call(srv))
  {
    ROS_INFO("Success!");
  }
  else
  {
    ROS_ERROR("Failed to call service move_base_node/clear_costmaps");
    return 1;
  }


        try
      {
        double datum_lat;
        double datum_lon;
        double datum_yaw;

        nh_priv.getParam("datum", datum_config);

        // Handle datum specification. Users should always specify a baseLinkFrameId_ in the
        // datum config, but we had a release where it wasn't used, so we'll maintain compatibility.
        ROS_ASSERT(datum_config.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(datum_config.size() >= 3);

        if (datum_config.size() > 3)
        {
          ROS_WARN_STREAM("Deprecated datum parameter configuration detected. Only the first three parameters "
              "(latitude, longitude, yaw) will be used. frame_ids will be derived from odometry and navsat inputs.");
        }

        std::ostringstream ostr;
        ostr << datum_config[0] << " " << datum_config[1] << " " << datum_config[2];
        std::istringstream istr(ostr.str());
        istr >> datum_lat >> datum_lon >> datum_yaw;

        // Try to resolve tf_prefix
        std::string tf_prefix = "";
        std::string tf_prefix_path = "";
        if (nh_priv.searchParam("tf_prefix", tf_prefix_path))
        {
          nh_priv.getParam(tf_prefix_path, tf_prefix);
        }

        // Append the tf prefix in a tf2-friendly manner
        FilterUtilities::appendPrefix(tf_prefix, world_frame_id_);
        FilterUtilities::appendPrefix(tf_prefix, base_link_frame_id_);

        robot_localization::SetDatum::Request request;
        request.geo_pose.position.latitude = datum_lat;
        request.geo_pose.position.longitude = datum_lon;
        request.geo_pose.position.altitude = 0.0;
        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, datum_yaw);
        request.geo_pose.orientation = tf2::toMsg(quat);
        robot_localization::SetDatum::Response response;
        datumCallback(request, response);
      }
      */