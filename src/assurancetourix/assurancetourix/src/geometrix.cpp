#include <geometrix.hpp>

Geometrix::Geometrix(Assurancetourix* node){
  this->node = node;
  z_axis = tf2::Vector3(0,0,1);
  init_parameters();
}

void Geometrix::init_parameters(){
  node->declare_parameter("arucos.asterix.side_right");
  node->declare_parameter("arucos.asterix.front_right");
  node->declare_parameter("arucos.asterix.front_left");
  node->declare_parameter("arucos.asterix.side_left");
  node->declare_parameter("arucos.asterix.back_left");
  node->declare_parameter("arucos.asterix.back_right");
  node->declare_parameter("arucos.asterix.front_x_offset");
  node->declare_parameter("arucos.asterix.front_y_offset");
  node->declare_parameter("arucos.asterix.side_y_offset");

  node->declare_parameter("arucos.obelix.side_right");
  node->declare_parameter("arucos.obelix.front_right");
  node->declare_parameter("arucos.obelix.front_left");
  node->declare_parameter("arucos.obelix.side_left");
  node->declare_parameter("arucos.obelix.back_left");
  node->declare_parameter("arucos.obelix.back_right");

  node->declare_parameter("arucos.x_y_offset");
  node->declare_parameter("arucos.enemies");

  node->get_parameter_or<int>("arucos.asterix.side_right", asterix.arucos[0], 172);
  node->get_parameter_or<int>("arucos.asterix.front_right", asterix.arucos[1], 173);
  node->get_parameter_or<int>("arucos.asterix.front_left", asterix.arucos[2], 174);
  node->get_parameter_or<int>("arucos.asterix.side_left", asterix.arucos[3], 175);
  node->get_parameter_or<int>("arucos.asterix.back_left", asterix.arucos[4], 176);
  node->get_parameter_or<int>("arucos.asterix.back_right", asterix.arucos[5], 177);
  node->get_parameter_or<double>("arucos.asterix.front_x_offset", asterix.front_x_offset, 0.09);
  node->get_parameter_or<double>("arucos.asterix.front_y_offset", asterix.front_y_offset, 0.06);
  node->get_parameter_or<double>("arucos.asterix.side_y_offset", asterix.side_y_offset, 0.18);

  node->get_parameter_or<int>("arucos.obelix.side_right", obelix.arucos[0], 182);
  node->get_parameter_or<int>("arucos.obelix.front_right", obelix.arucos[1], 183);
  node->get_parameter_or<int>("arucos.obelix.front_left", obelix.arucos[2], 184);
  node->get_parameter_or<int>("arucos.obelix.side_left", obelix.arucos[3], 185);
  node->get_parameter_or<int>("arucos.obelix.back_left", obelix.arucos[4], 186);
  node->get_parameter_or<int>("arucos.obelix.back_right", obelix.arucos[5], 187);
  node->get_parameter_or<double>("arucos.obelix.front_x_offset", obelix.front_x_offset, 0.09);
  node->get_parameter_or<double>("arucos.obelix.front_y_offset", obelix.front_y_offset, 0.06);
  node->get_parameter_or<double>("arucos.obelix.side_y_offset", obelix.side_y_offset, 0.18);

  node->get_parameter_or<double>("arucos.x_y_offset", enemies.x_y_offset, 0.05);
  std::vector<double> arucos;
  node->get_parameter_or<std::vector<double>>("arucos.enemies", arucos, {0.0});
  for (int i = 0; i<(int)arucos.size(); i++) enemies.arucos.push_back((int)arucos[i]);
}

void Geometrix::compute_and_send_markers(visualization_msgs::msg::MarkerArray &marker_array_ennemies, visualization_msgs::msg::MarkerArray &marker_array_allies){
  visualization_msgs::msg::MarkerArray actual_asterix, actual_obelix, actual_first_enemy, actual_second_enemy, unknown_allies, unknown_enemies;

  for (int i = 0; i<(int)marker_array_allies.markers.size(); i++){
    if (is_ally(marker_array_allies.markers[i].id, asterix)) actual_asterix.markers.push_back(marker_array_allies.markers[i]);
    else if (is_ally(marker_array_allies.markers[i].id, obelix)) actual_obelix.markers.push_back(marker_array_allies.markers[i]);
    else if (marker_array_allies.markers[i].id<=10) unknown_allies.markers.push_back(marker_array_allies.markers[i]);
  }

  for (int i = 0; i<(int)marker_array_ennemies.markers.size(); i++){
    if (is_enemy(marker_array_ennemies.markers[i].id, true)) actual_first_enemy.markers.push_back(marker_array_ennemies.markers[i]);
    else if (is_enemy(marker_array_ennemies.markers[i].id, false)) actual_second_enemy.markers.push_back(marker_array_ennemies.markers[i]);
    else if (marker_array_ennemies.markers[i].id<=10) unknown_enemies.markers.push_back(marker_array_ennemies.markers[i]);
  }

  for (int i = 0; i<(int)unknown_allies.markers.size(); i++){
    if (is_this_unknown_marker_ally(unknown_allies.markers[i], asterix, actual_asterix)) actual_asterix.markers.push_back(unknown_allies.markers[i]);
    else if (is_this_unknown_marker_ally(unknown_allies.markers[i], obelix, actual_obelix)) actual_obelix.markers.push_back(unknown_allies.markers[i]);
  }

  for (int i = 0; i<(int)unknown_enemies.markers.size(); i++){
    if (is_this_unknown_marker_enemy(unknown_enemies.markers[i], actual_first_enemy)) actual_first_enemy.markers.push_back(unknown_enemies.markers[i]);
    else if (is_this_unknown_marker_enemy(unknown_enemies.markers[i], actual_second_enemy)) actual_second_enemy.markers.push_back(unknown_enemies.markers[i]);
  }

  RCLCPP_INFO(node->get_logger(), "\nAsterix markers");
  for (int i = 0; i<(int)actual_asterix.markers.size(); i++){
    RCLCPP_INFO(node->get_logger(), "%d", actual_asterix.markers[i].id);
  }

  RCLCPP_INFO(node->get_logger(), "\nObelix markers");
  for (int i = 0; i<(int)actual_obelix.markers.size(); i++){
    RCLCPP_INFO(node->get_logger(), "%d", actual_obelix.markers[i].id);
  }

  RCLCPP_INFO(node->get_logger(), "\nFirst enemy markers");
  for (int i = 0; i<(int)actual_first_enemy.markers.size(); i++){
    RCLCPP_INFO(node->get_logger(), "%d", actual_first_enemy.markers[i].id);
  }

  RCLCPP_INFO(node->get_logger(), "\nSecond enemy markers");
  for (int i = 0; i<(int)actual_second_enemy.markers.size(); i++){
    RCLCPP_INFO(node->get_logger(), "%d", actual_second_enemy.markers[i].id);
  }

  RCLCPP_INFO(node->get_logger(), "\nUnknown enemy markers");
  for (int i = 0; i<(int)unknown_enemies.markers.size(); i++){
    RCLCPP_INFO(node->get_logger(), "%d", unknown_enemies.markers[i].id);
  }

  RCLCPP_INFO(node->get_logger(), "\nUnknown ally markers");
  for (int i = 0; i<(int)unknown_allies.markers.size(); i++){
    RCLCPP_INFO(node->get_logger(), "%d", unknown_allies.markers[i].id);
  }

}

bool Geometrix::is_this_unknown_marker_ally(visualization_msgs::msg::Marker &m, Ally ally, visualization_msgs::msg::MarkerArray &ally_marker_array){
  if (ally_marker_array.markers.size()==0) return false;
  geometry_msgs::msg::Point radius_point = m.pose.position;
  radius_point.x += ally.front_x_offset;
  radius_point.y += ally.side_y_offset;
  double radius = distance_2d_2_points(m.pose.position, radius_point);
  for (int i = 0; i<(int)ally_marker_array.markers.size(); i ++){
    if ( distance_2d_2_points(m.pose.position, ally_marker_array.markers[i].pose.position) > radius) return false;
  }
  return true;
}

bool Geometrix::is_this_unknown_marker_enemy(visualization_msgs::msg::Marker &m, visualization_msgs::msg::MarkerArray &enemy_marker_array){
  if (enemy_marker_array.markers.size()==0) return false;

  geometry_msgs::msg::Point radius_point = m.pose.position;
  radius_point.x += enemies.x_y_offset;
  radius_point.y += enemies.x_y_offset;
  double radius = distance_2d_2_points(m.pose.position, radius_point);
  for (int i = 0; i<(int)enemy_marker_array.markers.size(); i ++){
    if ( distance_2d_2_points(m.pose.position, enemy_marker_array.markers[i].pose.position) > radius) return false;
  }
  return true;
}

bool Geometrix::is_ally(int id, Ally ally){
  for (int i = 0; i<6; i++) if (id == ally.arucos[i]) return true;
  return false;
}

bool Geometrix::is_enemy(int id, bool first){
  if (first && (id == enemies.arucos[0] || id == enemies.arucos[1])) return true;
  if (!first && (id == enemies.arucos[2] || id == enemies.arucos[3])) return true;
  return false;
}

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

double Geometrix::distance_2d_2_points(geometry_msgs::msg::Point pt1, geometry_msgs::msg::Point pt2){
  return sqrt( (pt1.x - pt2.x)*(pt1.x - pt2.x) + (pt1.y - pt2.y)*(pt1.y - pt2.y) );
}

Geometrix::~Geometrix(){

}
