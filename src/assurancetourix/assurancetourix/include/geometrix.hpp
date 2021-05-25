#ifndef GEOMETRIX_HPP
#define GEOMETRIX_HPP

#include "assurancetourix.hpp"

class Assurancetourix;

struct Ally {
  double front_x_offset;
  double front_y_offset;
  double side_y_offset;
  int arucos[6];
};

struct Enemies {
  double x_y_offset;
  std::vector<int> arucos;
};

class Geometrix {
public:
  Geometrix(Assurancetourix* node);
  ~Geometrix();
  void compute_and_send_markers(visualization_msgs::msg::MarkerArray &marker_array_ennemies, visualization_msgs::msg::MarkerArray &marker_array_allies);
  int ally_or_enemy(int id);

  Ally asterix;
  Ally obelix;

  Enemies enemies;

private:

  void init_parameters();
  tf2::Vector3 get_perpandicular_vector_from_markers(visualization_msgs::msg::Marker &m1, visualization_msgs::msg::Marker &m2, bool reverse);
  tf2::Vector3 get_vector_from_markers(visualization_msgs::msg::Marker &m1, visualization_msgs::msg::Marker &m2);
  geometry_msgs::msg::Point middle_point(visualization_msgs::msg::Marker &m1, visualization_msgs::msg::Marker &m2);
  geometry_msgs::msg::Point point_from_marker(visualization_msgs::msg::Marker &m);
  double distance_2d_2_points(geometry_msgs::msg::Point pt1, geometry_msgs::msg::Point pt2);
  double distance_2d_2_points(double x1, double x2, double y1, double y2);
  bool is_this_unknown_marker_ally(visualization_msgs::msg::Marker &m, Ally ally, visualization_msgs::msg::MarkerArray &ally_marker_array);
  bool is_this_unknown_marker_enemy(visualization_msgs::msg::Marker &m, visualization_msgs::msg::MarkerArray &enemy_marker_array);
  bool is_ally(int id, Ally ally);
  bool is_enemy(int id, bool first);
  void remove_top_marker_if_necessary(visualization_msgs::msg::MarkerArray &marker_array);
  void compute_enemy_position(visualization_msgs::msg::MarkerArray &enemy_marker_array, visualization_msgs::msg::MarkerArray &ennemies_markers_to_publish, bool is_first_enemy);
  void compute_ally_position(visualization_msgs::msg::MarkerArray &ally_marker_array, Ally &ally, visualization_msgs::msg::MarkerArray &allies_markers_to_publish);
  double get_yaw_from_quaternion(geometry_msgs::msg::Quaternion &q);
  double normalize_angle(double angle);
  double mean_angle(std::vector<double> &angles);

  Assurancetourix* node;

  tf2::Vector3 z_axis;
};

#endif
