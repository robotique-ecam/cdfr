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

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::SplitGoal>("SplitGoal");
}
