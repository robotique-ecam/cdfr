#ifndef SPLIT_GOAL_ACTION_HPP_
#define SPLIT_GOAL_ACTION_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_util/robot_utils.hpp"

#include "behaviortree_cpp_v3/action_node.h"

#define AUTOMATIC 0
#define NOMINAL_FORCED 1
#define SLOW_FORCED 2

namespace nav2_behavior_tree
{

class SplitGoal : public BT::ActionNodeBase
{
public:
  SplitGoal(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);


  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination to plan to"),

      BT::OutputPort<bool>("get_out_teb_need", "Is get_out of specific area teb controller needed"),
      BT::OutputPort<bool>("nominal_teb_need", "Is nominal teb controller needed"),
      BT::OutputPort<bool>("get_in_teb_need", "Is get_in of specific area teb controller needed"),

      BT::OutputPort<geometry_msgs::msg::PoseStamped>("get_out_teb_goal", "Goal for teb controller that gets out a specific area"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("nominal_teb_goal", "Goal for nominal teb controller"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("get_in_teb_goal", "Goal for teb controller that gets in a specific area"),
    };
  }

private:
  void halt() override {}
  BT::NodeStatus tick() override;
  void setOutputPort();
  bool nearWalls(geometry_msgs::msg::PoseStamped & pose);
  void setAutoQuaternions();

  geometry_msgs::msg::PoseStamped goal_, get_out_goal_, get_in_goal_, nominal_goal_, current_pose_;
  bool get_out_need_, get_in_need_, nominal_need_;
  int mode_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Node::SharedPtr node_;
  double transform_tolerance_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__TRUNCATE_PATH_ACTION_HPP_
