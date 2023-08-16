#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class VecMarker{
public:
  VecMarker(std::string ns = "ns", int id = 0, std::string frame = "world"){
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
