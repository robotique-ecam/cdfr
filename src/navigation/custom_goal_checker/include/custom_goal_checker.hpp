#ifndef CUSTOM_GOAL_CHECKER_HPP_
#define CUSTOM_GOAL_CHECKER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/goal_checker.hpp"

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

  const double distance_from_walls = 0.25;
  const double distance_ = 0.15;
  double specific_area_coords[10][4] = {
    {0.9, 0.0, 1.5, 0.3 + distance_},
    {1.5, 0.0, 2.1, 0.3 + distance_},
    {0.0, 0.885 - distance_, 0.4 + distance_, 0.885},
    {0.0, 0.885, 0.4 + distance_, 0.885 + distance_},
    {0.0, 1.485 - distance_, 0.4 + distance_, 1.485},
    {0.0, 1.485, 0.4 + distance_, 1.485 + distance_},
    {2.6 - distance_, 0.885 - distance_, 3.0, 0.885},
    {2.6 - distance_, 0.885, 3.0, 0.885 + distance_},
    {2.6 - distance_, 1.485 - distance_, 3.0, 1.485},
    {2.6 - distance_, 1.485, 3.0, 1.485 + distance_}
  };
};

}  // namespace nav2_controller

#endif  // CUSTOM_GOAL_CHECKER_HPP_
