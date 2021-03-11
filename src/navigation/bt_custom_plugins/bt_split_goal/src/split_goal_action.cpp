#include <string>
#include <memory>
#include <limits>

#include "nav_msgs/msg/path.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "behaviortree_cpp_v3/decorator_node.h"

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
    node_, "bt_split_goal.distance_from_walls", rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(
    node_, "bt_split_goal.specific_area_coords", rclcpp::ParameterValue("NULL"));
  nav2_util::declare_parameter_if_not_declared(
    node_, "bt_split_goal.exit_area_coords", rclcpp::ParameterValue("NULL"));

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
  if (nearWalls(current_pose_) || get_out_area != 10) get_out_need_ = true;

  get_in_area = intoSpecificZone(goal_);
  if (nearWalls(goal_) || get_in_area != 10) get_in_need_ = true;

  setAutoQuaternions();

  setAutoPositions();

  setOutputPort();

  return BT::NodeStatus::SUCCESS;
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
  return 10;
}

void SplitGoal::setAutoQuaternions(){
  geometry_msgs::msg::Quaternion q;

  if (get_out_need_){
    geometry_msgs::msg::Quaternion q_out = current_pose_.pose.orientation;
    if ( (abs(q_out.z - 0.7071068)<0.22 &&  abs(q_out.w - 0.7071068)<0.22)
        || (abs(q_out.z + 0.7071068)<0.22 &&  abs(q_out.w + 0.7071068)<0.22) ){
      q.z = q.w = 0.7071068;
    }
    else if ( (abs(q_out.z + 0.7071068)<0.22 &&  abs(q_out.w - 0.7071068)<0.22)
        || (abs(q_out.z - 0.7071068)<0.22 &&  abs(q_out.w + 0.7071068)<0.22) ){
      q.z = -0.7071068;
      q.w = 0.7071068;
    }
    else if ( (abs(q_out.z)<0.22 &&  abs(q_out.w - 1.0)<0.22)
        || (abs(q_out.z)<0.22 &&  abs(q_out.w + 1.0)<0.22) ){
      q.z = 0.0;
      q.w = 1.0;
    }
    else if ( (abs(q_out.z - 1.0)<0.22 &&  abs(q_out.w)<0.22)
        || (abs(q_out.z + 1.0)<0.22 &&  abs(q_out.w)<0.22) ){
      q.z = 1.0;
      q.w = 0.0;
    }
    else q = q_out;
    get_out_goal_.pose.orientation = q;
  }

  //if get_in_need
  // check orientation ?
  nominal_goal_. pose.orientation = goal_.pose.orientation;
}

void SplitGoal::setAutoPositions(){
  if (get_out_need_){
    if (get_out_area != 10) get_out_goal_.pose.position.y = exit_area_coords[get_out_area];
    else {
      setPositionNearWall(get_out_goal_);
    }
  }
  if (get_in_need_){
    if (get_in_area != 10) nominal_goal_.pose.position.y = exit_area_coords[get_in_area];
    else {
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
