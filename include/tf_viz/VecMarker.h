//
// Created by dudu on 23-8-16.
//

#ifndef SRC_VECMARKER_H
#define SRC_VECMARKER_H

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
class VecMarker{
public:
    explicit VecMarker(std::string ns = "ns", int id = 0, std::string frame = "world"){
      _vec.header.frame_id = frame;
      _vec.header.stamp = ros::Time();

      _vec.ns = ns;
      _vec.id = id;
      _vec.type = visualization_msgs::Marker::ARROW;
      _vec.action = visualization_msgs::Marker::ADD;

      _vec.pose.position.x = 0;
      _vec.pose.position.y = 0;
      _vec.pose.position.z = 0;

      _vec.pose.orientation.x = 0.0;
      _vec.pose.orientation.y = 0.0;
      _vec.pose.orientation.z = 0.0;
      _vec.pose.orientation.w = 1.0;

      _vec.scale.x = 0.1;
      _vec.scale.y = 0.1;
      _vec.scale.z = 0.1;

      _vec.color.r = 1.0;
      _vec.color.g = 0.0;
      _vec.color.b = 0.0;
      _vec.color.a = 1.0;

      _p_start.x = 0;
      _p_start.y = 0;
      _p_start.z = 0;
      _vec.points.push_back(_p_start);
    }

    void set_end_point(std::vector<double> p_end){
      _p_end.x = p_end[0];
      _p_end.y = p_end[1];
      _p_end.z = p_end[2];
      _vec.points.push_back(_p_end);
    }

    void set_color(std::vector<float> color){
      _vec.color.r = color[0];
      _vec.color.g = color[1];
      _vec.color.b = color[2];
    }

    visualization_msgs::Marker get_vec(){
      return _vec;
    }

private:
    visualization_msgs::Marker _vec;
    geometry_msgs::Point _p_start;
    geometry_msgs::Point _p_end;

};


#endif //SRC_VECMARKER_H
