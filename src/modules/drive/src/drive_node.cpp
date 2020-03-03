#include <drive.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto drive_node = std::make_shared<Drive>();

  rclcpp::spin(drive_node);
  rclcpp::shutdown();
  return 0;
}
