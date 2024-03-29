#ifndef SPLIT_GOAL_ACTION_HPP_
#define SPLIT_GOAL_ACTION_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/buffer.h"

#include "nav2_util/robot_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/array_parser.hpp"

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
  void setAutoPositions();
  int intoSpecificZone(geometry_msgs::msg::PoseStamped & pose);
  void setPositionNearWall(geometry_msgs::msg::PoseStamped & pose);
  void nominalSplitter();
  geometry_msgs::msg::Point findNearestPoint(geometry_msgs::msg::Point p, float x1, float y1, float x2, float y2);
  bool belowLine(geometry_msgs::msg::Point p, float x1, float y1, float x2, float y2);

  geometry_msgs::msg::PoseStamped goal_, get_out_goal_, get_in_goal_, nominal_goal_, current_pose_;
  bool get_out_need_, get_in_need_, nominal_need_;
  int mode_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Node::SharedPtr node_;
  double transform_tolerance_;

  double distance_from_walls_;
  std::vector<std::vector<float>> specific_area_coords;
  std::vector<float> exit_area_coords, exit_area_types;

  int get_out_area, get_in_area;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr specific_area_visualization_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr specific_exit_visualization_pub;

  float radius_accurate_;
  const float sqrt2by2 = 0.7071068,
              quaternionDiff = 0.22,
              q_z_45 = 0.3826834,
              q_w_45 = 0.9238795;

};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__TRUNCATE_PATH_ACTION_HPP_
