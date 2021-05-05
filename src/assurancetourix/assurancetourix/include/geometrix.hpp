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

  Ally asterix;
  Ally obelix;

  Enemies enemies;

private:

  void init_parameters();
  tf2::Vector3 get_perpandicular_vector_from_markers(visualization_msgs::msg::Marker &m1, visualization_msgs::msg::Marker &m2, bool reverse);
  tf2::Vector3 get_vector_from_markers(visualization_msgs::msg::Marker &m1, visualization_msgs::msg::Marker &m2);
  geometry_msgs::msg::Point middle_point(visualization_msgs::msg::Marker &m1, visualization_msgs::msg::Marker &m2);
  geometry_msgs::msg::Point point_from_marker(visualization_msgs::msg::Marker &m);

  Assurancetourix* node;

  tf2::Vector3 z_axis;
};

#endif
