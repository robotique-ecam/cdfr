#include <unistd.h>
#include <serial/serial.h>
#include <rclcpp/rclcpp.hpp>


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("drive_node");

  RCLCPP_INFO(node->get_logger(), "Hello drive !");

  rclcpp::shutdown();
  return 0;
}
