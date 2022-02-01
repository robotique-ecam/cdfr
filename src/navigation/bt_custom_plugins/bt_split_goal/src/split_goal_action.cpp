#include <split_goal_action.hpp>

namespace nav2_behavior_tree
{

SplitGoal::SplitGoal(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
  getInput("goal", goal_);
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  node_->get_parameter("transform_tolerance", transform_tolerance_);

  nav2_util::declare_parameter_if_not_declared(
    node_, "bt_split_goal.radius_accurate", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node_, "bt_split_goal.distance_from_walls", rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(
    node_, "bt_split_goal.specific_area_coords", rclcpp::ParameterValue("NULL"));
  nav2_util::declare_parameter_if_not_declared(
    node_, "bt_split_goal.exit_area_coords", rclcpp::ParameterValue("NULL"));
  nav2_util::declare_parameter_if_not_declared(
    node_, "bt_split_goal.visualize_zones", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node_, "bt_split_goal.exit_area_type", rclcpp::ParameterValue("NULL"));

  node_->get_parameter("bt_split_goal.radius_accurate", radius_accurate_);
  node_->get_parameter("bt_split_goal.distance_from_walls", distance_from_walls_);

  std::string specific_area_coords_string;
  node_->get_parameter("bt_split_goal.specific_area_coords", specific_area_coords_string);

  if (specific_area_coords_string.compare("NULL") != 0) {
    std::string error_return;
    specific_area_coords = nav2_costmap_2d::parseVVF(specific_area_coords_string, error_return);
  }
  else
  {
    specific_area_coords = {{0.0, 0.0, 0.0, 0.0}};
    RCLCPP_WARN(rclcpp::get_logger("bt_split_goal"), "No specific_area_coords in yaml, considering no specific area");
  }

  std::string exit_area_coords_string;
  std::vector<std::vector<float>> exit_area_vect;
  node_->get_parameter("bt_split_goal.exit_area_coords", exit_area_coords_string);

  if (exit_area_coords_string.compare("NULL") != 0) {
    std::string error_return;
    exit_area_vect = nav2_costmap_2d::parseVVF(exit_area_coords_string, error_return);
  }
  else
  {
    exit_area_vect = {{0.0}};
    RCLCPP_WARN(rclcpp::get_logger("bt_split_goal"), "No exit_area_coords in yaml, considering no exit area");
  }
  exit_area_coords = exit_area_vect[0];

  RCLCPP_WARN(rclcpp::get_logger("bt_split_goal"), "1");

  std::string exit_area_types_string;
  std::vector<std::vector<float>> exit_area_types_vect;
  node_->get_parameter("bt_split_goal.exit_area_type", exit_area_types_string);

  RCLCPP_WARN(rclcpp::get_logger("bt_split_goal"), "2");

  if (exit_area_types_string.compare("NULL") != 0) {
    std::string error_return;
    exit_area_types_vect = nav2_costmap_2d::parseVVF(exit_area_types_string, error_return);
    RCLCPP_WARN(rclcpp::get_logger("bt_split_goal"), "3");
  }
  else
  {
    exit_area_types_vect = {{0.0}};
    RCLCPP_WARN(rclcpp::get_logger("bt_split_goal"), "No exit_area_types in yaml, considering no exit area");
  }
  exit_area_types = exit_area_types_vect[0];

  RCLCPP_WARN(rclcpp::get_logger("bt_split_goal"), "4");

  bool visualize;
  node_->get_parameter("bt_split_goal.visualize_zones", visualize);

  if (visualize){
    RCLCPP_WARN(rclcpp::get_logger("bt_split_goal"), "visualization on");

    specific_area_visualization_pub = node_->create_publisher<visualization_msgs::msg::MarkerArray>("nav_specific_area", rclcpp::QoS(rclcpp::KeepLast(10)));
    specific_exit_visualization_pub = node_->create_publisher<visualization_msgs::msg::MarkerArray>("nav_specific_exit", rclcpp::QoS(rclcpp::KeepLast(10)));

    visualization_msgs::msg::MarkerArray specific_area_visualization;
    visualization_msgs::msg::MarkerArray specific_exit_visualization;

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = node_->get_clock()->now();
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.color.a = 0.2;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.scale.z = 0.05;
    marker.lifetime.sec = 200;

    //specific_areas
    if (specific_area_coords_string.compare("NULL") != 0) {
      for(int i = 0; i < (int)specific_area_coords.size(); i++){
          marker.id = i;
          marker.scale.x = abs(specific_area_coords[i][0] - specific_area_coords[i][2]);
          marker.scale.y = abs(specific_area_coords[i][1] - specific_area_coords[i][3]);
          marker.pose.position.x = (specific_area_coords[i][0] + specific_area_coords[i][2])/2;
          marker.pose.position.y = (specific_area_coords[i][1] + specific_area_coords[i][3])/2;
          specific_area_visualization.markers.push_back(marker);
      }
    }
    //walls_areas
    marker.id = marker.id + 1;
    marker.scale.x = 3.0;
    marker.scale.y = distance_from_walls_;
    marker.pose.position.x = 1.5;
    marker.pose.position.y = distance_from_walls_/2;
    specific_area_visualization.markers.push_back(marker);

    marker.id = marker.id + 1;
    marker.pose.position.y = 2.0-distance_from_walls_/2;
    specific_area_visualization.markers.push_back(marker);

    marker.id = marker.id + 1;
    marker.scale.x = distance_from_walls_;
    marker.scale.y = 2.0;
    marker.pose.position.x = distance_from_walls_/2;
    marker.pose.position.y = 1.0;
    specific_area_visualization.markers.push_back(marker);

    marker.id = marker.id + 1;
    marker.pose.position.x = 3.0-distance_from_walls_/2;
    specific_area_visualization.markers.push_back(marker);

    specific_area_visualization_pub->publish(specific_area_visualization);

    marker.color.g = 1.0;

    //exit areas
    if (exit_area_coords_string.compare("NULL") != 0) {
      for(int i = 0; i < (int)exit_area_coords.size(); i++){
          marker.id = i;
          if (exit_area_types[i] == 0.0) {
            //y
            marker.scale.x = abs(specific_area_coords[i][0] - specific_area_coords[i][2]);
            marker.scale.y = 0.05;
            marker.pose.position.x = (specific_area_coords[i][0] + specific_area_coords[i][2])/2;
            marker.pose.position.y = exit_area_coords[i];
          } else if (exit_area_types[i] == 1.0){
            //x
            marker.scale.x = 0.05;
            marker.scale.y = abs(specific_area_coords[i][1] - specific_area_coords[i][3]);
            marker.pose.position.x = exit_area_coords[i];
            marker.pose.position.y = (specific_area_coords[i][1] + specific_area_coords[i][3])/2;
          } else {
            //other angle
          }
          specific_exit_visualization.markers.push_back(marker);
      }
    }

    //exit
    marker.id = marker.id + 1;
    marker.scale.x = 3.0;
    marker.scale.y = 0.05;
    marker.pose.position.x = 1.5;
    marker.pose.position.y = distance_from_walls_;
    specific_exit_visualization.markers.push_back(marker);

    marker.id = marker.id + 1;
    marker.pose.position.y = 2.0-distance_from_walls_;
    specific_exit_visualization.markers.push_back(marker);

    marker.id = marker.id + 1;
    marker.scale.x = 0.05;
    marker.scale.y = 2.0;
    marker.pose.position.x = distance_from_walls_;
    marker.pose.position.y = 1.0;
    specific_exit_visualization.markers.push_back(marker);

    marker.id = marker.id + 1;
    marker.pose.position.x = 3.0-distance_from_walls_;
    specific_exit_visualization.markers.push_back(marker);

    specific_exit_visualization_pub->publish(specific_exit_visualization);
  }
}

inline BT::NodeStatus SplitGoal::tick()
{
  setStatus(BT::NodeStatus::RUNNING);
  getInput("goal", goal_);

  if (goal_.pose.position.z == NOMINAL_FORCED) mode_ = NOMINAL_FORCED;
  else if (goal_.pose.position.z == SLOW_FORCED) mode_ = SLOW_FORCED;
  else mode_ = AUTOMATIC;
  goal_.pose.position.z = 0.0;

  nominal_goal_ = goal_;
  get_in_goal_ = goal_;
  get_out_need_ = false;
  get_in_need_ = false;
  nominal_need_ = false;

  if (mode_ == NOMINAL_FORCED){
    nominal_need_ = true;
    setOutputPort();
    return BT::NodeStatus::SUCCESS;
  }

  if (mode_ == SLOW_FORCED){
    get_in_need_ = true;
    setOutputPort();
    return BT::NodeStatus::SUCCESS;
  }

  if (!nav2_util::getCurrentPose(
      current_pose_, *tf_, "map", "base_link",
      transform_tolerance_))
  {
    RCLCPP_WARN(node_->get_logger(), "Current robot pose is not available.");
    return BT::NodeStatus::FAILURE;
  }

  nominal_need_ = true;
  get_out_goal_ = current_pose_;

  get_out_area = intoSpecificZone(current_pose_);
  if (nearWalls(current_pose_) || get_out_area != (int)specific_area_coords.size()) get_out_need_ = true;

  get_in_area = intoSpecificZone(goal_);
  if (nearWalls(goal_) || get_in_area != (int)specific_area_coords.size()) get_in_need_ = true;

  setAutoQuaternions();

  setAutoPositions();

  if (!get_in_need_) nominalSplitter();

  setOutputPort();

  return BT::NodeStatus::SUCCESS;
}

void SplitGoal::nominalSplitter(){
    nominal_goal_ = goal_;
}

void SplitGoal::setOutputPort(){
  setOutput("get_out_teb_need", get_out_need_);
  setOutput("nominal_teb_need", nominal_need_);
  setOutput("get_in_teb_need", get_in_need_);

  setOutput("get_out_teb_goal", get_out_goal_);
  setOutput("nominal_teb_goal", nominal_goal_);
  setOutput("get_in_teb_goal", get_in_goal_);
}

bool SplitGoal::nearWalls(geometry_msgs::msg::PoseStamped & pose){
  return pose.pose.position.x < distance_from_walls_ || pose.pose.position.x > 3.0 - distance_from_walls_
    || pose.pose.position.y < distance_from_walls_ || pose.pose.position.y > 2.0 - distance_from_walls_;
}

int SplitGoal::intoSpecificZone(geometry_msgs::msg::PoseStamped & pose){
  for(int i = 0; i < (int)specific_area_coords.size(); i++){
    if (pose.pose.position.x > specific_area_coords[i][0] && pose.pose.position.x < specific_area_coords[i][2]
      && pose.pose.position.y > specific_area_coords[i][1] && pose.pose.position.y < specific_area_coords[i][3]) return i;
  }
  return (int)specific_area_coords.size();
}

void SplitGoal::setAutoQuaternions(){
  geometry_msgs::msg::Quaternion q;

  if (get_out_need_){
    geometry_msgs::msg::Quaternion q_out = current_pose_.pose.orientation;
    if ( (abs(q_out.z - sqrt2by2)<quaternionDiff &&  abs(q_out.w - sqrt2by2)<quaternionDiff)
        || (abs(q_out.z + sqrt2by2)<quaternionDiff &&  abs(q_out.w + sqrt2by2)<quaternionDiff) ){
      q.z = q.w = sqrt2by2;
    }
    else if ( (abs(q_out.z + sqrt2by2)<quaternionDiff &&  abs(q_out.w - sqrt2by2)<quaternionDiff)
        || (abs(q_out.z - sqrt2by2)<quaternionDiff &&  abs(q_out.w + sqrt2by2)<quaternionDiff) ){
      q.z = -sqrt2by2;
      q.w = sqrt2by2;
    }
    else if ( (abs(q_out.z)<quaternionDiff &&  abs(q_out.w - 1.0)<quaternionDiff)
        || (abs(q_out.z)<quaternionDiff &&  abs(q_out.w + 1.0)<quaternionDiff) ){
      q.z = 0.0;
      q.w = 1.0;
    }
    else if ( (abs(q_out.z - 1.0)<quaternionDiff &&  abs(q_out.w)<quaternionDiff)
        || (abs(q_out.z + 1.0)<quaternionDiff &&  abs(q_out.w)<quaternionDiff) ){
      q.z = 1.0;
      q.w = 0.0;
    }
    else q = q_out;
    get_out_goal_.pose.orientation = q;
  }

  nominal_goal_. pose.orientation = goal_.pose.orientation;
}

void SplitGoal::setAutoPositions(){
  if (get_out_need_){
    if (get_out_area != (int)specific_area_coords.size()) {
      if (exit_area_types[get_out_area] == 0.0) get_out_goal_.pose.position.y = exit_area_coords[get_out_area];
      else get_out_goal_.pose.position.x = exit_area_coords[get_out_area];
    } else {
      setPositionNearWall(get_out_goal_);
    }
  }
  if (get_in_need_){
    if (get_in_area != (int)specific_area_coords.size()) {
      if (exit_area_types[get_out_area] == 0.0) nominal_goal_.pose.position.y = exit_area_coords[get_in_area];
      else nominal_goal_.pose.position.x = exit_area_coords[get_in_area];
    } else {
      setPositionNearWall(nominal_goal_);
    }
  }
}

void SplitGoal::setPositionNearWall(geometry_msgs::msg::PoseStamped & pose){
  if (pose.pose.position.x < distance_from_walls_) pose.pose.position.x = distance_from_walls_;
  else if (pose.pose.position.x > 3.0 - distance_from_walls_) pose.pose.position.x = 3.0 - distance_from_walls_;
  else if (pose.pose.position.y < distance_from_walls_) pose.pose.position.y = distance_from_walls_;
  else pose.pose.position.y = 2.0 - distance_from_walls_;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::SplitGoal>("SplitGoal");
}
