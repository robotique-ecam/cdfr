#include <memory>
#include <string>
#include "custom_goal_checker.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "angles/angles.h"
#include "nav2_util/node_utils.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

namespace nav2_controller
{

CustomGoalChecker::CustomGoalChecker()
: xy_goal_tolerance_nominal_(0.03),
  yaw_goal_tolerance_nominal_(0.05),
  xy_goal_tolerance_accurate_(0.01),
  yaw_goal_tolerance_accurate_(0.01),
  stateful_(true),
  check_xy_(true),
  xy_goal_tolerance_sq_nominal_(0.0625),
  xy_goal_tolerance_sq_accurate_(1e-4)
{
}

void CustomGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & nh,
  const std::string & plugin_name)
{
  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".xy_goal_tolerance_nominal", rclcpp::ParameterValue(0.03));
  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".yaw_goal_tolerance_nominal", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".xy_goal_tolerance_accurate", rclcpp::ParameterValue(0.01));
  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".yaw_goal_tolerance_accurate", rclcpp::ParameterValue(0.01));
  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".stateful", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".specific_area_coords", rclcpp::ParameterValue("NULL"));
  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".distance_from_walls", rclcpp::ParameterValue(0.15));

  nh->get_parameter(plugin_name + ".xy_goal_tolerance_nominal", xy_goal_tolerance_nominal_);
  nh->get_parameter(plugin_name + ".yaw_goal_tolerance_nominal", yaw_goal_tolerance_nominal_);
  nh->get_parameter(plugin_name + ".xy_goal_tolerance_accurate", xy_goal_tolerance_accurate_);
  nh->get_parameter(plugin_name + ".yaw_goal_tolerance_accurate", yaw_goal_tolerance_accurate_);
  nh->get_parameter(plugin_name + ".stateful", stateful_);
  nh->get_parameter(plugin_name + ".distance_from_walls", distance_from_walls_);

  std::string specific_area_coords_string;
  nh->get_parameter(plugin_name + ".specific_area_coords", specific_area_coords_string);

  if (specific_area_coords_string.compare("NULL") != 0) {
    std::string error_return;
    specific_area_coords = nav2_costmap_2d::parseVVF(specific_area_coords_string, error_return);
  }
  else
  {
    specific_area_coords = {{0.0, 0.0, 0.0, 0.0}};
    RCLCPP_WARN(rclcpp::get_logger("custom_goal_checker"), "No specific_area_coords in yaml, considering no specific area");
  }

  xy_goal_tolerance_sq_nominal_ = xy_goal_tolerance_nominal_ * yaw_goal_tolerance_nominal_;
  xy_goal_tolerance_sq_accurate_ = xy_goal_tolerance_accurate_ * yaw_goal_tolerance_accurate_;
}

void CustomGoalChecker::reset()
{
  check_xy_ = true;
}

bool CustomGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist &)
{
  double tolerance_sq, yaw_tolerance;

  if (nearWalls(goal_pose) || intoSpecificZone(goal_pose)){
    tolerance_sq = xy_goal_tolerance_sq_accurate_;
    yaw_tolerance = yaw_goal_tolerance_accurate_;
  }
  else
  {
    tolerance_sq = xy_goal_tolerance_sq_nominal_;
    yaw_tolerance = yaw_goal_tolerance_nominal_;
  }

  if (check_xy_) {
    double dx = query_pose.position.x - goal_pose.position.x,
      dy = query_pose.position.y - goal_pose.position.y;
    if (dx * dx + dy * dy > tolerance_sq) {
      return false;
    }
    // We are within the window
    // If we are stateful, change the state.
    if (stateful_) {
      check_xy_ = false;
    }
  }
  double dyaw = angles::shortest_angular_distance(
    tf2::getYaw(query_pose.orientation),
    tf2::getYaw(goal_pose.orientation));
  return fabs(dyaw) < yaw_tolerance;
}

bool CustomGoalChecker::nearWalls(const geometry_msgs::msg::Pose & pose){
  return pose.position.x < distance_from_walls_ || pose.position.x > 3.0 - distance_from_walls_
    || pose.position.y < distance_from_walls_ || pose.position.y > 2.0 - distance_from_walls_;
}

bool CustomGoalChecker::intoSpecificZone(const geometry_msgs::msg::Pose & pose){
  for(int i = 0; i < (int)specific_area_coords.size(); i++){
    if (pose.position.x > specific_area_coords[i][0] && pose.position.x < specific_area_coords[i][2]
      && pose.position.y > specific_area_coords[i][1] && pose.position.y < specific_area_coords[i][3]) return true;
  }
  return false;
}

}  // namespace nav2_controller

PLUGINLIB_EXPORT_CLASS(nav2_controller::CustomGoalChecker, nav2_core::GoalChecker)
