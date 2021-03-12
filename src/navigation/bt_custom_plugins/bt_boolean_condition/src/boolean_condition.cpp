#include <boolean_condition.hpp>

namespace nav2_behavior_tree
{

BooleanCondition::BooleanCondition(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
  getInput("boolean", bool_);
}

inline BT::NodeStatus BooleanCondition::tick()
{
  setStatus(BT::NodeStatus::RUNNING);
  getInput("boolean", bool_);

  if (bool_) return BT::NodeStatus::SUCCESS;

  return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::BooleanCondition>("BooleanCondition");
}
