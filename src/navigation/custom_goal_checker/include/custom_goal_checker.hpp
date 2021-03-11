#ifndef CUSTOM_GOAL_CHECKER_HPP_
#define CUSTOM_GOAL_CHECKER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/goal_checker.hpp"
#include <nav2_costmap_2d/array_parser.hpp>

namespace nav2_controller
{

class CustomGoalChecker : public nav2_core::GoalChecker
{
public:
  CustomGoalChecker();
  // Standard GoalChecker Interface
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & nh,
    const std::string & plugin_name) override;
  void reset() override;
  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) override;

protected:
  bool nearWalls(const geometry_msgs::msg::Pose & pose);
  bool intoSpecificZone(const geometry_msgs::msg::Pose & pose);

  double xy_goal_tolerance_nominal_, yaw_goal_tolerance_nominal_, xy_goal_tolerance_accurate_, yaw_goal_tolerance_accurate_;
  bool stateful_, check_xy_;
  double xy_goal_tolerance_sq_nominal_, xy_goal_tolerance_sq_accurate_;

  double distance_from_walls_;
  std::vector<std::vector<float>> specific_area_coords;
};

}  // namespace nav2_controller

#endif  // CUSTOM_GOAL_CHECKER_HPP_
