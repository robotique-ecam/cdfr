#ifndef BOOLEAN_CONDITION_ACTION_HPP_
#define BOOLEAN_CONDITION_ACTION_HPP_

#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{

class BooleanCondition : public BT::ActionNodeBase
{
public:
  BooleanCondition(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);


  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("boolean", "SUCCESS if true, else FAILURE"),
    };
  }

private:
  void halt() override {}
  BT::NodeStatus tick() override;

  bool bool_;
};

}  // namespace nav2_behavior_tree

#endif  // BOOLEAN_CONDITION_ACTION_HPP_
