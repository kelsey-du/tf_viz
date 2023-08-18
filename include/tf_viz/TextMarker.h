//
// Created by dudu on 23-8-16.
//

#ifndef SRC_TextMARKER_H
#define SRC_TextMARKER_H

#include <visualization_msgs/Marker.h>
class TextMARKER{
public:
    explicit TextMARKER(std::string ns = "ns", int id = 0, std::string frame = "world", std::string text = "test"){
      _vec.header.frame_id = frame;
      _vec.header.stamp = ros::Time();

      _vec.ns = ns;
      _vec.id = id;
      _vec.text = text;
      _vec.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      _vec.action = visualization_msgs::Marker::ADD;

      _vec.pose.position.x = 0;
      _vec.pose.position.y = 0;
      _vec.pose.position.z = -0.3;

      _vec.pose.orientation.x = 0.0;
      _vec.pose.orientation.y = 0.0;
      _vec.pose.orientation.z = 0.0;
      _vec.pose.orientation.w = 1.0;

      _vec.scale.x = 0.1;
      _vec.scale.y = 0.1;
      _vec.scale.z = 0.5;

      _vec.color.r = 1.0;
      _vec.color.g = 1.0;
      _vec.color.b = 1.0;
      _vec.color.a = 1.0;

    }

    void set_color(std::vector<float> color){
      _vec.color.r = color[0];
      _vec.color.g = color[1];
      _vec.color.b = color[2];
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

    void set_ns(std::string ns){
      _vec.ns = ns;
    }

    void set_id(int id){
      _vec.id = id;
    }

    void set_frame(std::string frame){
      _vec.header.frame_id = frame;
    }

    void set_text(std::string text){
      _vec.text = text;
    }

    visualization_msgs::Marker get_vec(){
      return _vec;
    }

private:
    visualization_msgs::Marker _vec;

};


#endif //SRC_TextMARKER_H
