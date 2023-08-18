//
// Created by dudu on 23-8-16.
//

#ifndef SRC_AXESMARKER_H
#define SRC_AXESMARKER_H

#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/TransformStamped.h>
#include "TextMarker.h"
class AXESMarker{
public:
    explicit AXESMarker(std::string ns = "axes", std::string frame = "world",
                        std::vector<double> origin = {0.0, 0.0, 0.0},
                        Eigen::Quaterniond rotation = {1.0, 0.0, 0.0, 0.0}){
      _x_axis.set_ns(ns);
      _x_axis.set_frame(frame);
      _x_axis.set_id(0);
      _x_axis.set_color({1.0, 0.0, 0.0});
      _x_axis.set_p_end({1.0, 0.0, 0.0});
      _x_axis.set_origin(origin);
      _x_axis.set_rotation(rotation);

      _y_axis.set_ns(ns);
      _y_axis.set_frame(frame);
      _y_axis.set_id(1);
      _y_axis.set_color({0.0, 1.0, 0.0});
      _y_axis.set_p_end({0.0, 1.0, 0.0});
      _y_axis.set_origin(origin);
      _y_axis.set_rotation(rotation);

      _z_axis.set_ns(ns);
      _z_axis.set_frame(frame);
      _z_axis.set_id(2);
      _z_axis.set_color({0.0, 0.0, 1.0});
      _z_axis.set_p_end({0.0, 0.0, 1.0});
      _z_axis.set_origin(origin);
      _z_axis.set_rotation(rotation);

      _name.set_ns(ns);
      _name.set_frame(frame);
      _name.set_id(3);
      _name.set_text(ns);
      _name.set_origin(origin);
      _name.set_rotation(rotation);

      _tf.header.frame_id = frame;
      _tf.header.stamp = ros::Time::now();
      _tf.child_frame_id = ns;
      _tf.transform.translation.x = origin[0];
      _tf.transform.translation.y = origin[1];
      _tf.transform.translation.z = origin[2];
      _tf.transform.rotation.w = rotation.w();
      _tf.transform.rotation.x = rotation.x();
      _tf.transform.rotation.y = rotation.y();
      _tf.transform.rotation.z = rotation.z();
    }

    void set_origin(std::vector<double> origin){
      _x_axis.set_origin(origin);
      _y_axis.set_origin(origin);
      _z_axis.set_origin(origin);
      _name.set_origin(origin);
    }

    void set_scale(std::vector<double> scale){
      _x_axis.set_scale(scale);
      _y_axis.set_scale(scale);
      _z_axis.set_scale(scale);
    }

    visualization_msgs::MarkerArray get_axes(){
      visualization_msgs::MarkerArray axes;
      axes.markers.push_back(_x_axis.get_vec());
      axes.markers.push_back(_y_axis.get_vec());
      axes.markers.push_back(_z_axis.get_vec());
      axes.markers.push_back(_name.get_vec());
      return axes;
    }

    geometry_msgs::TransformStamped get_tf_msg(){
      return _tf;
    }



private:
    VecMarker _x_axis;
    VecMarker _y_axis;
    VecMarker _z_axis;
    TextMARKER _name;

    /// tf msg from "frame" to "ns" frame
    geometry_msgs::TransformStamped _tf;

};


#endif //SRC_AXESMARKER_H
