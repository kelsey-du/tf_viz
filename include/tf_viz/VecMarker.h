//
// Created by dudu on 23-8-16.
//

#ifndef SRC_VECMARKER_H
#define SRC_VECMARKER_H

#include <visualization_msgs/Marker.h>
class VecMarker{
public:
    explicit VecMarker(std::string ns = "ns", int id = 0, std::string frame = "world",
            std::vector<double> p_end = {1.0, 0.0, 0.0}){
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

    void set_scale(std::vector<double> scale){
      _vec.scale.x = scale[0];
      _vec.scale.y = scale[1];
      _vec.scale.z = scale[2];
    }

    void set_origin(std::vector<double> origin){
      _vec.pose.position.x = origin[0];
      _vec.pose.position.y = origin[1];
      _vec.pose.position.z = origin[2];
    }

    void set_rotation(Eigen::Quaterniond rotation){
      _vec.pose.orientation.x = rotation.x();
      _vec.pose.orientation.y = rotation.y();
      _vec.pose.orientation.z = rotation.z();
      _vec.pose.orientation.w = rotation.w();
    }

    void set_p_end(std::vector<double> p_end){
      _p_end.x = p_end[0];
      _p_end.y = p_end[1];
      _p_end.z = p_end[2];
      _vec.points[1] = _p_end;
    }

    void set_ns(std::string ns){
      _vec.ns = ns;
    }

    void set_id(int id){
      _vec.id = id;
    }

    void set_frame(std::string frame){
      _vec.header.frame_id = frame;
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
