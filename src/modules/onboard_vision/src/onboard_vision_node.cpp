#include <onboard_vision.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto vision_node = std::make_shared<OnboardVision>();

  rclcpp::spin(vision_node);
  rclcpp::shutdown();
  return 0;
}
