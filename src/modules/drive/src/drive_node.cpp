#include <drive/drive.hpp>


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto drive_node = std::make_shared<Drive>();

  RCLCPP_INFO(drive_node->get_logger(), "Hello drive !");

  rclcpp::shutdown();
  return 0;
}
