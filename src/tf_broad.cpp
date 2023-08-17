//
// Created by dudu on 23-8-16.
//

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "tf_viz/VecMarker.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle nh;

  // pub the 1st tf
  tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped tf_msg1;
  tf_msg1.header.frame_id = "world";
  tf_msg1.header.stamp = ros::Time::now();
  tf_msg1.child_frame_id = "tf_msg1";
  tf_msg1.transform.translation.x = 10.0;
  tf_msg1.transform.translation.y = -10.0;
  tf_msg1.transform.translation.z = 0.0;
  tf_msg1.transform.rotation.w = 1.0;


  ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("markers", 10);
  visualization_msgs::MarkerArray markers;

  VecMarker g("g", 0);
  g.set_end_point({-0.29, 0.32, 9.78});
  g.set_color({1.0, 0.0, 0.0});
  VecMarker g_norm("g_norm", 1);
  g_norm.set_end_point({-0.029661, 0.032343, 0.999037});
  g_norm.set_color({0.0, 1.0, 0.0});

  markers.markers.push_back(g.get_vec());
  markers.markers.push_back(g_norm.get_vec());

  ros::Rate rate(10.0);
  while (ros::ok())
  {
    static_broadcaster.sendTransform(tf_msg1);

    for(auto & marker : markers.markers){
      marker.header.stamp = ros::Time();
    }

    marker_pub.publish(markers);

    ros::spinOnce();
    rate.sleep();
  }


  return 0;
};
