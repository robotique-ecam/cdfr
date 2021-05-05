#include <geometrix.hpp>

Geometrix::Geometrix(Assurancetourix* node){
  this->node = node;
  z_axis = tf2::Vector3(0,0,1);
tf2::Vector3 Geometrix::get_perpandicular_vector_from_markers(visualization_msgs::msg::Marker &m1, visualization_msgs::msg::Marker &m2, bool reverse){
  double angle = reverse ? (-M_PI/2) : (M_PI/2);
  tf2::Vector3 vec = get_vector_from_markers(m1, m2);
  return vec.rotate(z_axis, angle);
}

tf2::Vector3 Geometrix::get_vector_from_markers(visualization_msgs::msg::Marker &m1, visualization_msgs::msg::Marker &m2){
  geometry_msgs::msg::Point pt1 = point_from_marker(m1),
                            pt2 = point_from_marker(m2);
  tf2::Vector3 vec;
  if (pt1.x < pt2.x) vec = tf2::Vector3(pt2.x - pt1.x, pt2.y - pt1.y, 0);
  else vec = tf2::Vector3(pt1.x - pt2.x, pt1.y - pt2.y, 0);
  return vec.normalize();
}

geometry_msgs::msg::Point Geometrix::middle_point(visualization_msgs::msg::Marker &m1, visualization_msgs::msg::Marker &m2){
  geometry_msgs::msg::Point pt1 = point_from_marker(m1),
                            pt2 = point_from_marker(m2),
                            to_return;
  to_return.x = (pt1.x+pt2.x)/2;
  to_return.y = (pt1.y+pt2.y)/2;
  return to_return;
}

geometry_msgs::msg::Point Geometrix::point_from_marker(visualization_msgs::msg::Marker &m){
  return m.pose.position;
}

Geometrix::~Geometrix(){

}
