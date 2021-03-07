#include <string>
#include <memory>
#include <limits>

#include "nav_msgs/msg/path.hpp"
#include "nav2_util/geometry_utils.hpp"
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
  getInput("global_path", path_);
}

inline BT::NodeStatus SplitGoal::tick()
{
  setStatus(BT::NodeStatus::RUNNING);
  getInput("goal", goal_);
  getInput("global_path", path_);

  RCLCPP_WARN(
    config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
    "Initial orientation z: %f, w: %f", path_.poses.size(), path_.poses[0].pose.orientation.w
  );

  if (goal_.pose.position.z == NOMINAL_FORCED) mode_ = NOMINAL_FORCED;
  else if (goal_.pose.position.z == SLOW_FORCED) mode_ = SLOW_FORCED;
  else mode_ = AUTOMATIC;
  goal_.pose.position.z = 0.0;

  get_out_goal_ = goal_;
  get_in_goal_ = goal_;
  nominal_goal_ = goal_;
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

  return BT::NodeStatus::FAILURE;
/*
  nav_msgs::msg::Path input_path;

  getInput("input_path", input_path);

  if (input_path.poses.empty()) {
    setOutput("output_path", input_path);
    return BT::NodeStatus::SUCCESS;
  }

  geometry_msgs::msg::PoseStamped final_pose = input_path.poses.back();

  double distance_to_goal = nav2_util::geometry_utils::euclidean_distance(
    input_path.poses.back(), final_pose);

  while (distance_to_goal < distance_ && input_path.poses.size() > 2) {
    input_path.poses.pop_back();
    distance_to_goal = nav2_util::geometry_utils::euclidean_distance(
      input_path.poses.back(), final_pose);
  }

  double dx = final_pose.pose.position.x - input_path.poses.back().pose.position.x;
  double dy = final_pose.pose.position.y - input_path.poses.back().pose.position.y;

  double final_angle = atan2(dy, dx);

  if (std::isnan(final_angle) || std::isinf(final_angle)) {
    RCLCPP_WARN(
      config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
      "Final angle is not valid while truncating path. Setting to 0.0");
    final_angle = 0.0;
  }

  input_path.poses.back().pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
    final_angle);

  setOutput("output_path", input_path);

  return BT::NodeStatus::SUCCESS;
  */
}

void SplitGoal::setOutputPort(){
  setOutput("get_out_teb_need", get_out_need_);
  setOutput("nominal_teb_need", nominal_need_);
  setOutput("get_in_teb_need", get_in_need_);

  setOutput("get_out_teb_goal", get_out_goal_);
  setOutput("nominal_teb_goal", nominal_goal_);
  setOutput("get_in_teb_goal", get_in_goal_);
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::SplitGoal>("SplitGoal");
}
