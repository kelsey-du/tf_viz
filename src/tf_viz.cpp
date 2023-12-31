//
// Created by dudu on 23-8-16.
//

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "tf_viz/VecMarker.h"
#include "tf_viz/AXESMarker.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle nh;

  // world to camera, an example
  std::vector<double> t_CinW = {10.0, 10.0, 0.0};
  Eigen::Matrix3d R_WtoC;
  R_WtoC << 0, -1, 0,
          0, 0, 1,
          1, 0, 0;
  Eigen::Quaterniond q_WtoC;
  q_WtoC = Eigen::Quaterniond(R_WtoC);

  // world to inertial
  std::vector<double> t_IinW = {0.0, 0.0, 0.0};
  Eigen::Quaterniond q_WtoI(1,0,0,0);

  // gravity vector in inertial frame
  std::vector<double> vec_ginI = {-0.29, 0.32, 9.78};
  // std::vector<double> vec_ginI = {0, 0, 9.78};

  // gravity to inertial
  std::vector<double> t_GinI = {10, -10, 0};
  Eigen::Matrix3d R_GtoI;
  R_GtoI << -0.999559552, -0.000959829, -0.029661106,
            0.000000000, -0.999476831, 0.032342910,
            -0.029676632, 0.032328665, 0.999036613;
  Eigen::Quaterniond q_GtoI(R_GtoI);

  // tf broadcaster
  tf2_ros::StaticTransformBroadcaster static_broadcaster;
  // marker array publisher
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("markers", 10);
  visualization_msgs::MarkerArray markers;

  // inertial frame marker
  AXESMarker imu_axes("inertial", "world", t_IinW, q_WtoI);
  markers.markers.insert(markers.markers.end(), {imu_axes.get_axes().markers.at(0),
                                                 imu_axes.get_axes().markers.at(1),
                                                 imu_axes.get_axes().markers.at(2),
                                                 imu_axes.get_axes().markers.at(3)});

  // g vec in inertial frame marker
  VecMarker g_inI("g_inI", 0, "inertial", vec_ginI);
  markers.markers.push_back(g_inI.get_vec());

  // camera frame marker
  AXESMarker cam_axes("camera", "world", t_CinW, q_WtoC);
  markers.markers.insert(markers.markers.end(), {cam_axes.get_axes().markers.at(0),
                                                 cam_axes.get_axes().markers.at(1),
                                                 cam_axes.get_axes().markers.at(2),
                                                 cam_axes.get_axes().markers.at(3)});

  // gravity frame marker
  AXESMarker g_axes("gravity", "inertial", t_GinI, q_GtoI.inverse());
  markers.markers.insert(markers.markers.end(), {g_axes.get_axes().markers.at(0),
                                                 g_axes.get_axes().markers.at(1),
                                                 g_axes.get_axes().markers.at(2),
                                                 g_axes.get_axes().markers.at(3)});

  // g vec in inertial frame marker translate to gravity frame
  VecMarker g_inI_toG("g_inI", 1, "inertial", vec_ginI);
  g_inI_toG.set_origin(t_GinI);
  markers.markers.push_back(g_inI_toG.get_vec());

  ros::Rate rate(10.0);
  while (ros::ok())
  {
    static_broadcaster.sendTransform(imu_axes.get_tf_msg());
    static_broadcaster.sendTransform(cam_axes.get_tf_msg());
    static_broadcaster.sendTransform(g_axes.get_tf_msg());

    for(auto & marker : markers.markers){
      marker.header.stamp = ros::Time();
    }

    marker_pub.publish(markers);

    ros::spinOnce();
    rate.sleep();
  }


  return 0;
};
