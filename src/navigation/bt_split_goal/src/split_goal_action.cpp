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
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  node_->get_parameter("transform_tolerance", transform_tolerance_);
}

inline BT::NodeStatus SplitGoal::tick()
{
  setStatus(BT::NodeStatus::RUNNING);
  getInput("goal", goal_);

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

  if (!nav2_util::getCurrentPose(
      current_pose_, *tf_, "map", "base_link",
      transform_tolerance_))
  {
    RCLCPP_WARN(node_->get_logger(), "Current robot pose is not available.");
    return BT::NodeStatus::FAILURE;
  }


  }

  }



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
  return pose.pose.position.x < distance_from_walls || pose.pose.position.x > 3.0 - distance_from_walls
    || pose.pose.position.y < distance_from_walls || pose.pose.position.y > 2.0 - distance_from_walls;
}

void SplitGoal::setAutoQuaternions(){
  geometry_msgs::msg::Quaternion q;

  if (get_out_need_){
    geometry_msgs::msg::Quaternion q_out = current_pose_.pose.orientation;
    RCLCPP_WARN(node_->get_logger(), "q_out z: %f, w: %f", q_out.z, q_out.w);
    if ( (abs(q_out.z - 0.7071068)<0.35 &&  abs(q_out.w - 0.7071068)<0.35)
        || (abs(q_out.z + 0.7071068)<0.35 &&  abs(q_out.w + 0.7071068)<0.35) ){
      q.z = q.w = 0.7071068;
    }
    else if ( (abs(q_out.z + 0.7071068)<0.35 &&  abs(q_out.w - 0.7071068)<0.35)
        || (abs(q_out.z - 0.7071068)<0.35 &&  abs(q_out.w + 0.7071068)<0.35) ){
      q.z = -0.7071068;
      q.w = 0.7071068;
    }
    else q = q_out;
    get_out_goal_.pose.orientation = q;
  }

  //if get_in_need
  // check orientation ?
  nominal_goal_. pose.orientation = goal_.pose.orientation;
}
}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::SplitGoal>("SplitGoal");
}
