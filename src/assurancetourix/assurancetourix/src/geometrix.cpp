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

  node->declare_parameter("arucos.enemies.x_y_offset");
  node->declare_parameter("arucos.enemies.arucos");

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

  node->get_parameter_or<double>("arucos.enemies.x_y_offset", enemies.x_y_offset, 0.05);
  std::vector<double> arucos;
  node->get_parameter_or<std::vector<double>>("arucos.enemies.arucos", arucos, {0.0});
  for (int i = 0; i<(int)arucos.size(); i++) enemies.arucos.push_back((int)arucos[i]);
}

int Geometrix::ally_or_enemy(int id){
  if (id>50){
    for (int i = 0; i<6; i++) if (id == asterix.arucos[i]) return 0;
    for (int i = 0; i<6; i++) if (id == obelix.arucos[i]) return 0;
    for (int i = 0; i<int(enemies.arucos.size()); i++) if (id == enemies.arucos[i]) return 1;
  } else {
    if (node->side.compare("yellow") == 0){
      if (id <= 10 && 6 <= id) return 0;
      else if (id <= 5 && 1 <= id) return 1;
    } else {
      if (id <= 10 && 6 <= id) return 1;
      else if (id <= 5 && 1 <= id) return 0;
    }
  }
  return -1;
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

  visualization_msgs::msg::MarkerArray allies_markers_to_publish, enemies_markers_to_publish;
  compute_ally_position(actual_asterix, asterix, allies_markers_to_publish);
  compute_enemy_position(actual_first_enemy, enemies_markers_to_publish);
}

void Geometrix::compute_ally_position(visualization_msgs::msg::MarkerArray &ally_marker_array, Ally &ally, visualization_msgs::msg::MarkerArray &allies_markers_to_publish){
  remove_top_marker_if_necessary(ally_marker_array);

  visualization_msgs::msg::MarkerArray front, back, top, side;
  geometry_msgs::msg::Point avg_point;
  std::vector<double> avg_angle;
  int considered = 0;

  for (int i = 0; i<(int)ally_marker_array.markers.size(); i++){
    if (ally_marker_array.markers[i].id == ally.arucos[1] || ally_marker_array.markers[i].id == ally.arucos[2]) front.markers.push_back(ally_marker_array.markers[i]);
    else if (ally_marker_array.markers[i].id == ally.arucos[4] || ally_marker_array.markers[i].id == ally.arucos[5]) back.markers.push_back(ally_marker_array.markers[i]);
    else if (ally_marker_array.markers[i].id == ally.arucos[0] || ally_marker_array.markers[i].id == ally.arucos[3]) side.markers.push_back(ally_marker_array.markers[i]);
    else if (ally_marker_array.markers[i].id<10) top.markers.push_back(ally_marker_array.markers[i]);
  }

  if (front.markers.size() == 2){
    considered += 1;
    int front_right_index = front.markers[0].id==ally.arucos[1] ? 0 : 1;
    int front_left_index = (front_right_index+1) % 2;
    tf2::Vector3 vec = get_perpandicular_vector_from_markers(front.markers[front_left_index], front.markers[front_right_index], true);
    geometry_msgs::msg::Point middle = middle_point(front.markers[0], front.markers[1]);
    RCLCPP_INFO(node->get_logger(), "\nfront");
    RCLCPP_INFO(node->get_logger(), "\nx: %f, y; %f", middle.x + ally.front_x_offset*vec.x(), middle.y + ally.front_x_offset*vec.y());
    avg_point.x += (middle.x + ally.front_x_offset*vec.x());
    avg_point.y += (middle.y + ally.front_x_offset*vec.y());
    avg_angle.push_back( ( get_yaw_from_quaternion(front.markers[0].pose.orientation) + get_yaw_from_quaternion(front.markers[1].pose.orientation) )/2);
    //avg_angle += normalize_angle((get_yaw_from_quaternion(front.markers[0].pose.orientation) + get_yaw_from_quaternion(front.markers[1].pose.orientation))/2);
    RCLCPP_INFO(node->get_logger(), "\nangle: %f", ((get_yaw_from_quaternion(front.markers[0].pose.orientation) + get_yaw_from_quaternion(front.markers[1].pose.orientation))/2)*180/M_PI);

  }

  if (back.markers.size() == 2){
    considered += 1;
    int back_right_index = back.markers[0].id==ally.arucos[5] ? 0 : 1;
    int back_left_index = (back_right_index+1) % 2;
    tf2::Vector3 vec = get_perpandicular_vector_from_markers(back.markers[back_right_index], back.markers[back_left_index], true);
    geometry_msgs::msg::Point middle = middle_point(back.markers[0], back.markers[1]);
    avg_point.x += (middle.x + ally.front_x_offset*vec.x());
    avg_point.y += (middle.y + ally.front_x_offset*vec.y());
    RCLCPP_INFO(node->get_logger(), "\nback");
    RCLCPP_INFO(node->get_logger(), "\nx: %f, y; %f", middle.x + ally.front_x_offset*vec.x(), middle.y + ally.front_x_offset*vec.y());
    avg_angle.push_back((get_yaw_from_quaternion(back.markers[0].pose.orientation) + get_yaw_from_quaternion(back.markers[1].pose.orientation))/2 - M_PI);
    //avg_angle += normalize_angle((get_yaw_from_quaternion(back.markers[0].pose.orientation) + get_yaw_from_quaternion(back.markers[1].pose.orientation))/2 - M_PI);
    RCLCPP_INFO(node->get_logger(), "\nangle: %f", ((get_yaw_from_quaternion(back.markers[0].pose.orientation) + get_yaw_from_quaternion(back.markers[1].pose.orientation))/2 - M_PI)*180/M_PI);
  }

  if (side.markers.size() == 1){
    considered++;
    double marker_yaw = get_yaw_from_quaternion(side.markers[0].pose.orientation);
    avg_point.x += side.markers[0].pose.position.x + ally.side_y_offset*cos(marker_yaw + M_PI);
    avg_point.y += side.markers[0].pose.position.y + ally.side_y_offset*sin(marker_yaw + M_PI);
    RCLCPP_INFO(node->get_logger(), "\nside");
    RCLCPP_INFO(node->get_logger(), "\nx: %f, y; %f", side.markers[0].pose.position.x + ally.side_y_offset*cos(marker_yaw + M_PI), side.markers[0].pose.position.y + ally.side_y_offset*sin(marker_yaw + M_PI));
    if (side.markers[0].id == ally.arucos[0]){
      RCLCPP_INFO(node->get_logger(), "\nangle: %f", (marker_yaw + M_PI/2)*180/M_PI);
      avg_angle.push_back(marker_yaw + M_PI/2);
      //avg_angle += normalize_angle(marker_yaw + M_PI/2);
    }
    else{
      RCLCPP_INFO(node->get_logger(), "\nangle: %f", (marker_yaw - M_PI/2)*180/M_PI);
      avg_angle.push_back(marker_yaw - M_PI/2);
      //avg_angle += normalize_angle(marker_yaw - M_PI/2);
    }
  }

  if (top.markers.size() == 1){
    if (front.markers.size() > 0){
      int front_right_index = -1,
          front_left_index = -1;
      if (front.markers[0].id==ally.arucos[1]) front_right_index = 0;
      if (front.markers[0].id==ally.arucos[2]) front_left_index = 0;
      if (front.markers.size() > 1) {
        if (front.markers[1].id==ally.arucos[1]) front_right_index = 1;
        if (front.markers[1].id==ally.arucos[2]) front_left_index = 1;
      }

      if (front_right_index != -1){
        considered +=1;
        avg_point.x += top.markers[0].pose.position.x;
        avg_point.y += top.markers[0].pose.position.y;
        avg_angle.push_back(get_yaw_from_quaternion(front.markers[front_right_index].pose.orientation));
        //avg_angle += normalize_angle(get_yaw_from_quaternion(front.markers[front_right_index].pose.orientation));
        RCLCPP_INFO(node->get_logger(), "\ntop+front_right");
        RCLCPP_INFO(node->get_logger(), "\nx: %f, y; %f", top.markers[0].pose.position.x, top.markers[0].pose.position.y);
        RCLCPP_INFO(node->get_logger(), "\nangle: %f", (normalize_angle(get_yaw_from_quaternion(front.markers[front_right_index].pose.orientation)))*180/M_PI);
      }
      if (front_left_index != -1){
        considered +=1;
        avg_point.x += top.markers[0].pose.position.x;
        avg_point.y += top.markers[0].pose.position.y;
        avg_angle.push_back(get_yaw_from_quaternion(front.markers[front_left_index].pose.orientation));
        //avg_angle += normalize_angle(get_yaw_from_quaternion(front.markers[front_left_index].pose.orientation));
        RCLCPP_INFO(node->get_logger(), "\ntop+front_left");
        RCLCPP_INFO(node->get_logger(), "\nx: %f, y; %f", top.markers[0].pose.position.x, top.markers[0].pose.position.y);
        RCLCPP_INFO(node->get_logger(), "\nangle: %f", (normalize_angle(get_yaw_from_quaternion(front.markers[front_left_index].pose.orientation)))*180/M_PI);
      }
    }

    if (back.markers.size() > 0){
      int back_right_index = -1,
          back_left_index = -1;
      if (back.markers[0].id==ally.arucos[5]) back_right_index = 0;
      if (back.markers[0].id==ally.arucos[4]) back_left_index = 0;
      if (back.markers.size() > 1) {
        if (back.markers[1].id==ally.arucos[5]) back_right_index = 1;
        if (back.markers[1].id==ally.arucos[4]) back_left_index = 1;
      }

      if (back_right_index != -1){
        considered +=1;
        avg_point.x += top.markers[0].pose.position.x;
        avg_point.y += top.markers[0].pose.position.y;
        avg_angle.push_back(get_yaw_from_quaternion(back.markers[back_right_index].pose.orientation) - M_PI);
        //avg_angle += normalize_angle(get_yaw_from_quaternion(back.markers[back_right_index].pose.orientation) - M_PI);
        RCLCPP_INFO(node->get_logger(), "\ntop+back_right");
        RCLCPP_INFO(node->get_logger(), "\nx: %f, y; %f", top.markers[0].pose.position.x, top.markers[0].pose.position.y);
        RCLCPP_INFO(node->get_logger(), "\nangle: %f", (normalize_angle(get_yaw_from_quaternion(back.markers[back_right_index].pose.orientation)- M_PI))*180/M_PI);
      }
      if (back_left_index != -1){
        considered +=1;
        avg_point.x += top.markers[0].pose.position.x;
        avg_point.y += top.markers[0].pose.position.y;
        avg_angle.push_back(get_yaw_from_quaternion(back.markers[back_left_index].pose.orientation) - M_PI);
        //avg_angle += normalize_angle(get_yaw_from_quaternion(back.markers[back_left_index].pose.orientation) - M_PI);
        RCLCPP_INFO(node->get_logger(), "\ntop+back_left");
        RCLCPP_INFO(node->get_logger(), "\nx: %f, y; %f", top.markers[0].pose.position.x, top.markers[0].pose.position.y);
        RCLCPP_INFO(node->get_logger(), "\nangle: %f", (normalize_angle(get_yaw_from_quaternion(back.markers[back_left_index].pose.orientation)- M_PI))*180/M_PI);
      }
    }

    if (side.markers.size() > 0){
      //TODO
    }
  }

  if (considered > 0){
    avg_point.x = avg_point.x/considered;
    avg_point.y = avg_point.y/considered;
    ally_marker_array.markers[0].pose.position = avg_point;

    double angle = mean_angle(avg_angle);
    tf2::Quaternion q;
    q.setRotation(z_axis, angle);
    ally_marker_array.markers[0].pose.orientation = tf2::toMsg(q);

    if (ally.arucos[0] == asterix.arucos[0]){
      ally_marker_array.markers[0].text = "ASTERIX";
      ally_marker_array.markers[0].id = 1;
    } else {
      ally_marker_array.markers[0].text = "OBELIX";
      ally_marker_array.markers[0].id = 2;
    }
    allies_markers_to_publish.markers.push_back(ally_marker_array.markers[0]);
    RCLCPP_INFO(node->get_logger(), "\nx: %f, y; %f", avg_point.x, avg_point.y);
    RCLCPP_INFO(node->get_logger(), "\nangle: %f", angle*180/M_PI);
  }

}

void Geometrix::compute_enemy_position(visualization_msgs::msg::MarkerArray &enemy_marker_array, visualization_msgs::msg::MarkerArray &enemies_markers_to_publish){
  remove_top_marker_if_necessary(enemy_marker_array);

  if (enemy_marker_array.markers.size() == 1){
    geometry_msgs::msg::Point point;
    if (enemy_marker_array.markers[0].id < 10) point = enemy_marker_array.markers[0].pose.position;
    else{
      double angle = get_yaw_from_quaternion(enemy_marker_array.markers[0].pose.orientation);
      point.x += enemy_marker_array.markers[0].pose.position.x - enemies.x_y_offset * cos(angle);
      point.y += enemy_marker_array.markers[0].pose.position.y - enemies.x_y_offset * sin(angle);
    }
    RCLCPP_INFO(node->get_logger(), "\nenemy 1 markers");
    RCLCPP_INFO(node->get_logger(), "\nx: %f, y; %f", point.x, point.y);
    enemy_marker_array.markers[0].pose.position = point;
    enemies_markers_to_publish.markers.push_back(enemy_marker_array.markers[0]);
  }
  else if (enemy_marker_array.markers.size() == 2){

    for (int i = 0; i<2; i++){
      if (enemy_marker_array.markers[i].id<=10) {
        enemies_markers_to_publish.markers.push_back(enemy_marker_array.markers[i]);
        return;
      }
    }
    geometry_msgs::msg::Point avg_point;
    for (int i=0; i<2; i++){
      double angle = get_yaw_from_quaternion(enemy_marker_array.markers[i].pose.orientation);
      avg_point.x += (enemy_marker_array.markers[i].pose.position.x - enemies.x_y_offset * cos(angle))/2;
      avg_point.y += (enemy_marker_array.markers[i].pose.position.y - enemies.x_y_offset * sin(angle))/2;
    }
    RCLCPP_INFO(node->get_logger(), "\nenemy 2 markers");
    RCLCPP_INFO(node->get_logger(), "\nx: %f, y; %f", avg_point.x, avg_point.y);
    enemy_marker_array.markers[0].pose.position = avg_point;
    enemies_markers_to_publish.markers.push_back(enemy_marker_array.markers[0]);
  }
  else if (enemy_marker_array.markers.size() == 3){
    geometry_msgs::msg::Point top, avg_point, lat_avg;
    visualization_msgs::msg::MarkerArray lat;

    for (int i = 0; i<3; i++){
      if (enemy_marker_array.markers[i].id<=10) top = enemy_marker_array.markers[i].pose.position;
      else lat.markers.push_back(enemy_marker_array.markers[i]);
    }

    if (lat.markers.size() != 2) avg_point = top;
    else {
      for (int i=0; i<2; i++){
        double angle = get_yaw_from_quaternion(lat.markers[i].pose.orientation);
        lat_avg.x += lat.markers[i].pose.position.x - enemies.x_y_offset * cos(angle);
        lat_avg.y += lat.markers[i].pose.position.y - enemies.x_y_offset * sin(angle);
      }

      avg_point.x = (lat_avg.x + top.x)/3;
      avg_point.y = (lat_avg.y + top.y)/3;
    }
    RCLCPP_INFO(node->get_logger(), "\nenemy 3 markers");
    RCLCPP_INFO(node->get_logger(), "\nx: %f, y; %f", avg_point.x, avg_point.y);

    enemy_marker_array.markers[0].pose.position = avg_point;
    enemies_markers_to_publish.markers.push_back(enemy_marker_array.markers[0]);
  }
}

void Geometrix::remove_top_marker_if_necessary(visualization_msgs::msg::MarkerArray &marker_array){
  for (int i = 0; i<(int)marker_array.markers.size(); i++){
    if (marker_array.markers[i].id < 10){
      if (abs(marker_array.markers[i].pose.position.x  - node->assurancetourix_to_map_transformation.transform.translation.x) > 1 ||
          marker_array.markers[i].pose.position.y < 1) marker_array.markers.erase(marker_array.markers.begin()+i);
    }
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

tf2::Vector3 Geometrix::get_vector_from_markers(visualization_msgs::msg::Marker &origin, visualization_msgs::msg::Marker &m2){
  geometry_msgs::msg::Point pt1 = point_from_marker(origin),
                            pt2 = point_from_marker(m2);
  tf2::Vector3 vec = tf2::Vector3(pt2.x - pt1.x, pt2.y - pt1.y, 0);
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
  return distance_2d_2_points(pt1.x, pt2.x, pt1.y, pt2.y);
}

double Geometrix::distance_2d_2_points(double x1, double x2, double y1, double y2){
  return sqrt( (x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) );
}

double Geometrix::get_yaw_from_quaternion(geometry_msgs::msg::Quaternion &q){
  tf2::Quaternion q_tf2;
  tf2::fromMsg(q, q_tf2);
  tf2::Matrix3x3 m(q_tf2);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw + M_PI/2;
}

double Geometrix::normalize_angle(double angle){
  if (angle > M_PI) return -2*M_PI+angle;
  if (angle < -M_PI) return 2*M_PI+angle;
  return angle;
}

double Geometrix::mean_angle(std::vector<double> &angles){
  double x = 0,
         y = 0;
  for (int i = 0; i<(int)angles.size(); i++){
    x += cos(angles[i]);
    y += sin(angles[i]);
  }
  return atan2(y, x);
}

Geometrix::~Geometrix(){

}
